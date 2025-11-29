/*
  MTZ Shift Detector — ESP32-S2 (LOLIN S2 mini) v1.1

  Two AS5600 angle sensors (one per axis) -> classify tractor gear position
  Adds grid-based shifter model with range latch (I/II) and hysteresis.
  Manual override page allows forcing gear code for remote testing.

  Exposes Modbus TCP holding registers and a minimal Wi-Fi HTTP UI for
  calibration, status, manual override, grid calibration, and network settings.

  ───────────────────────────────────────────────────────────────────────────
  Hardware notes (quick schematic snippet)
  ───────────────────────────────────────────────────────────────────────────
  • MCU: LOLIN S2 mini (ESP32-S2). Power via USB-C 5 V; on-board 3V3 LDO.
  • Sensors: 2 × AS5600 (address 0x36 each) on SEPARATE I²C controllers.
      I²C-X: SDA=GPIO33, SCL=GPIO35
      I²C-Y: SDA=GPIO9,  SCL=GPIO11
  • AS5600 configured for 12-bit ANGLE register (0..4095). MAG and AGC also read.
  • Modbus TCP on Wi-Fi STA or AP: port 502.
  • Discrete outputs for each gear (1..9, selector, N, etc.) *mirrored* via Modbus
    discrete inputs (1xxxx) and HTTP JSON status.

  ───────────────────────────────────────────────────────────────────────────
  Safety / behavior
  ───────────────────────────────────────────────────────────────────────────
  • If either AS5600 is out of valid MAG range, status bits flag fault_X/Y.
  • If Wi-Fi is down, Modbus still works on local (softAP) IP.
  • Manual override can force arbitrary gear code via HTTP or Modbus; status bit
    flags override mode.
  • Gear classification uses polygon-based 2D plane model with debounce:
       - Current filtered point (filtX,filtY) → node (GridSlot)
       - Node → gear code via gearFromGrid(...) + range latch I/II.
    This replaces the older grid/ellipse model.

*/

#include <WiFi.h>
#include <WebServer.h>
#include <ModbusIP_ESP8266.h>
#include <Preferences.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdint.h>

// --- Forward declarations to satisfy Arduino function auto-prototypes ---
#include <stdint.h>

// Early forward decls (avoid Arduino auto-prototype pitfalls)
enum RangeSel : uint8_t; // grid (declared later)
enum GridSlot : uint8_t; // grid (declared later)
struct PolyNode;         // polygon node (declared later)

// Function prototypes actually used
uint16_t gearFromGrid(GridSlot gs, RangeSel r);
GridSlot classifyGrid(uint16_t x, uint16_t y);

// ---------------- Configuration ----------------
// Pins: 2 independent I²C buses
constexpr int SDA_X = 33;  // I2C for X axis AS5600
constexpr int SCL_X = 35;
constexpr int SDA_Y = 9;   // I2C for Y axis AS5600
constexpr int SCL_Y = 11;

// AS5600 registers
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t REG_ZMCO    = 0x00;
constexpr uint8_t REG_ZPOS_H  = 0x01;
constexpr uint8_t REG_ZPOS_L  = 0x02;
constexpr uint8_t REG_MPOS_H  = 0x03;
constexpr uint8_t REG_MPOS_L  = 0x04;
constexpr uint8_t REG_MANG_H  = 0x05;
constexpr uint8_t REG_MANG_L  = 0x06;
constexpr uint8_t REG_CONF_H  = 0x07;
constexpr uint8_t REG_CONF_L  = 0x08;
constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;
constexpr uint8_t REG_RAW_ANGLE_L = 0x0D;
constexpr uint8_t REG_ANG_H   = 0x0E;   // ANGLE (12 bit)
constexpr uint8_t REG_ANG_L   = 0x0F;
constexpr uint8_t REG_STATUS  = 0x0B;
constexpr uint8_t REG_AGC     = 0x1A;
constexpr uint8_t REG_MAG_H   = 0x1B;
constexpr uint8_t REG_MAG_L   = 0x1C;

// Preferences namespace
Preferences prefs;

// Wi-Fi credentials (loaded from NVS or defaults)
String wifiSsid   = "Tracktor";
String wifiPass   = "ujooCeeph9@$";
String wifiAPSsid   = "MTZ_SHIFT";
String wifiAPPass   = "mtzshift";

// Network config
IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

// Manual override
bool     manualOverride     = false;
uint16_t manualOverrideCode = 0;

// Debounce / timing
uint32_t lastRangeChangeMs = 0;
uint32_t lastGearChangeMs  = 0;
uint32_t lastPrintMs       = 0;

// Tunables (also exposed via HTTP + Modbus)
uint32_t GRID_STABLE_MS    = 120;  // candidate must be stable this long to accept
uint32_t GEAR_DEBOUNCE_MS  = 300;  // gear code debounce
uint32_t RANGE_DEBOUNCE_MS = 200;  // range latch debounce

// Live sensor and classifier state
uint16_t rawX=0, rawY=0;
uint16_t filtX=0, filtY=0;
uint16_t agcX=0, agcY=0;
uint16_t magX=0, magY=0;
uint8_t  errorCode=0;
uint16_t gearCode=0;
bool     inSelectorLane=false;
uint16_t statusBits=0;
bool     prevMagOkX=false, prevMagOkY=false;
uint8_t  prevErrorLogged=255;

// Grid classifier debug state
int        gridBest      = 0;   // instantaneous best (from classifyGrid)
int        gridCandidate = 0;   // currently considered node
int        gridDebounced = 0;   // accepted node after debounce
uint32_t   gridCandSince = 0;

// Range latch
enum RangeSel : uint8_t { R_UNKNOWN=0, R_I=1, R_II=2 };
RangeSel rangeLatched = R_I;

// Range names
const char* rangeName(RangeSel r){
  switch(r){
    case R_I:  return "I";
    case R_II: return "II";
    default:   return "?";
  }
}

// Logging helpers
static inline void LOG_BEGIN(){ Serial.begin(115200); delay(200); }
static inline void logln(const char* s){ Serial.println(s); }
static inline void logf(const char* fmt, ...){
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

// ---------------- Global objects ----------------
TwoWire WireX = TwoWire(0);
TwoWire WireY = TwoWire(1);
bool   wifiUp   = false;

// ---------------- HTTP server ----------------
WebServer http(80);

// ---------------- Modbus TCP ----------------
ModbusIP mb;

// ---------------- I²C helpers ----------------
bool i2cRead8(TwoWire& W, uint8_t reg, uint8_t& out){
  W.beginTransmission(AS5600_ADDR); W.write(reg);
  if (W.endTransmission(false) != 0) return false;
  if (W.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return false;
  out = W.read();
  return true;
}

bool i2cRead16_12b(TwoWire& W, uint8_t regH, uint8_t regL, uint16_t& out){
  uint8_t hi, lo;
  if (!i2cRead8(W, regH, hi)) return false;
  if (!i2cRead8(W, regL, lo)) return false;
  uint16_t v = ((uint16_t)hi << 8) | lo;
  out = v & 0x0FFF; // 12 bits
  return true;
}

bool i2cRead16(TwoWire& W, uint8_t regH, uint8_t regL, uint16_t& out){
  uint8_t hi, lo;
  if (!i2cRead8(W, regH, hi)) return false;
  if (!i2cRead8(W, regL, lo)) return false;
  out = ((uint16_t)hi << 8) | lo;
  return true;
}

// ---------------- Legacy per-gear mapping via GridSlot ----------------

// Gear code validity (for manual override safety)
bool isValidGearCode(uint16_t g){
  if (g == 0) return true;      // Neutral
  if (g >= 1 && g <= 9) return true;
  if (g == 101 || g == 102) return true; // special if any
  return false;
}

// Manual override control via Modbus (called from loop)
void pollOverrideFromModbus(){
  uint16_t oc = mb.Hreg(14);
  bool     reqEn   = (oc & 0x0001) != 0;
  uint16_t reqCode = mb.Hreg(15);
  if (reqEn != manualOverride) {
    manualOverride = reqEn;
    logf("[MAN] override %s (via Modbus)", manualOverride ? "ENABLED" : "DISABLED");
  }
  if (isValidGearCode(reqCode) && reqCode != manualOverrideCode) {
    manualOverrideCode = reqCode;
    logf("[MAN] override code=%u (via Modbus)", manualOverrideCode);
  }
}

// ---------------- Discrete Inputs mirror (GEAR_CODE) ----------------
void updateDiscreteInputs(){
  const uint8_t used[] = {0,1,2,3,4,5,6,7,8,9,20,21};
  for (uint8_t i = 0; i < sizeof(used); ++i) { mb.Ists(used[i], false); }
  if (gearCode == 0) { mb.Ists(0, true); return; }
  if (gearCode >= 1 && gearCode <= 9) { mb.Ists((uint8_t)gearCode, true); return; }
  if (gearCode == 101) { mb.Ists(20, true); return; }
  if (gearCode == 102) { mb.Ists(21, true); return; }
}

// ================= GRID CLASSIFIER (RANGE + HYSTERESIS) =====================
enum GridSlot : uint8_t {
  GS_N=0,        // Neutral (center reference)
  GS_SEL_I,      // selector I
  GS_SEL_II,     // selector II
  // Top row (y=+1)
  GS_Xn1_Yp1,    // (-1,+1) -> 4/7
  GS_X0_Yp1,     // ( 0,+1) -> 9 (II only)
  GS_Xp1_Yp1,    // (+1,+1) -> 1/2
  // Middle row (y=0) — neutral across columns
  GS_Xn1_Y0,     // (-1,0) -> N
  GS_Xp1_Y0,     // (+1,0) -> N
  // Bottom row (y=-1)
  GS_Xn1_Yn1,    // (-1,-1) -> 5/8
  GS_X0_Yn1,     // ( 0,-1) -> 3/6
  GS_Xp1_Yn1,    // (+1,-1) -> R1/R2
  GS_COUNT
};

// Correct grid mapping for mirrored + 90° CW layout
uint16_t gearFromGrid(GridSlot gs, RangeSel r){
  switch(gs){
    case GS_N:         return 0; // neutral reference
    case GS_SEL_I:     return 0; // selection action, gear not engaged yet
    case GS_SEL_II:    return 0;
    // Top row y=+1
    case GS_Xn1_Yp1:   return (r==R_I) ? 4 : (r==R_II ? 7 : 65535);
    case GS_X0_Yp1:    return (r==R_II) ? 9 : 65535;   // II only
    case GS_Xp1_Yp1:   return (r==R_I) ? 1 : (r==R_II ? 2 : 65535);
    // Middle row
    case GS_Xn1_Y0:    return 0;  // neutral
    case GS_Xp1_Y0:    return 0;  // neutral
    // Bottom row y=-1
    case GS_Xn1_Yn1:   return (r==R_I) ? 5 : (r==R_II ? 8 : 65535);
    case GS_X0_Yn1:    return (r==R_I) ? 3 : (r==R_II ? 6 : 65535);
    case GS_Xp1_Yn1:   return (r==R_I) ? 101 : (r==R_II ? 102 : 65535); // R1/R2
    default:           return 65535;
  }
}

static const char* GRID_NAMES[GS_COUNT] = {
  "N", "SEL_I", "SEL_II",
  "X-1_Y+1", "X0_Y+1", "X+1_Y+1",
  "X-1_Y0",  "X+1_Y0",
  "X-1_Y-1", "X0_Y-1", "X+1_Y-1"
};

// ---- Polygon-based plane model ----
// Each grid slot is represented as a polygon in (X,Y) space.
// Multiple calibration calls can add rectangles (4 vertices each) to the polygon.

constexpr uint8_t MAX_POLY_VERTS = 24;  // per node, tune as needed

struct PolyNode {
  uint8_t  n = 0;                       // number of vertices
  uint16_t x[MAX_POLY_VERTS];
  uint16_t y[MAX_POLY_VERTS];
  bool     valid = false;
};

struct PlanePack {
  uint32_t magic   = 0xA11D5E77;  // 'all set'
  uint16_t version = 0x0200;      // bumped version for polygon format
  PolyNode node[GS_COUNT];
};

PlanePack planeCal;
bool      gridMode      = true;   // polygon classifier enabled
// rangeLatched and gridNode declared earlier / nearby
GridSlot  gridNode      = GS_N;

// Hysteresis thresholds kept for compatibility / UI (no longer used in classifyGrid)
uint32_t GRID_ENTER_TH = 7000;
uint32_t GRID_EXIT_TH  = 12000;
uint16_t GRID_POLY_PAD = 0;      // padding (in raw units) to expand polygons during classification

// --- Simple point-in-polygon test (even/odd rule) ---
static inline uint32_t dist2PointSeg(int32_t px, int32_t py, int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
  int32_t vx = x2 - x1;
  int32_t vy = y2 - y1;
  int32_t wx = px - x1;
  int32_t wy = py - y1;

  int32_t denom = vx * vx + vy * vy;
  if (denom == 0) {
    // Degenerate edge: treat as a point
    int32_t dx = px - x1;
    int32_t dy = py - y1;
    return (uint32_t)(dx * dx + dy * dy);
  }

  // Project (w) onto (v), clamp to segment [0,1]
  float t = (float)(wx * vx + wy * vy) / (float)denom;
  if (t < 0.0f) t = 0.0f;
  else if (t > 1.0f) t = 1.0f;

  float cx = (float)x1 + t * (float)vx;
  float cy = (float)y1 + t * (float)vy;

  float dx = (float)px - cx;
  float dy = (float)py - cy;
  return (uint32_t)(dx * dx + dy * dy);
}

static bool pointInPoly(const PolyNode &p, uint16_t px, uint16_t py, uint16_t pad = 0) {
  if (!p.valid || p.n < 3) return false;
  bool inside = false;

  for (uint8_t i = 0, j = p.n - 1; i < p.n; j = i++) {
    uint16_t xi = p.x[i], yi = p.y[i];
    uint16_t xj = p.x[j], yj = p.y[j];

    // Check if edge (xi,yi)-(xj,yj) crosses the horizontal ray at y=py
    bool intersect = ((yi > py) != (yj > py)) &&
                     ((int32_t)px <
                     (int32_t)(xj - xi) * (int32_t)(py - yi) / (int32_t)(yj - yi) +
                     (int32_t)xi);
    if (intersect) inside = !inside;
  }
  if (inside || pad == 0) return inside;

  // If outside, allow a padded halo around the polygon edges.
  uint32_t pad2 = (uint32_t)pad * (uint32_t)pad;
  uint32_t best = 0xFFFFFFFFu;
  for (uint8_t i = 0, j = p.n - 1; i < p.n; j = i++) {
    uint32_t d2 = dist2PointSeg(px, py, p.x[j], p.y[j], p.x[i], p.y[i]);
    if (d2 < best) best = d2;
  }
  return best <= pad2;
}

// For debug/heatmap: 0 if inside, squared distance to closest vertex otherwise.
static uint32_t polyScore(uint8_t idx, uint16_t px, uint16_t py) {
  const auto &p = planeCal.node[idx];
  if (!p.valid || p.n == 0) return 0xFFFFFFFFu;
  if (pointInPoly(p, px, py, GRID_POLY_PAD)) return 0;

  uint32_t best = 0xFFFFFFFFu;
  for (uint8_t i = 0; i < p.n; ++i) {
    int32_t dx = (int32_t)px - (int32_t)p.x[i];
    int32_t dy = (int32_t)py - (int32_t)p.y[i];
    uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
    if (d2 < best) best = d2;
  }
  return best;
}

// NVS I/O re-using the same "grid" key, but now storing polygons.
bool loadGrid(){
  PlanePack tmp;
  size_t sz = prefs.getBytes("grid", &tmp, sizeof(tmp));
  if (sz == sizeof(tmp) && tmp.magic == 0xA11D5E77) {
    planeCal = tmp;
    return true;
  }
  return false;
}

void saveGrid(){ prefs.putBytes("grid", &planeCal, sizeof(planeCal)); }

void clearGrid(){
  PlanePack fresh;
  planeCal = fresh;
  saveGrid();
}

// Sampling: build an axis-aligned rectangle around the lever movement
// and append it as 4 vertices into the polygon for this node.
// Multiple calls -> union of several rectangles -> polygon-ish region.
void sampleGrid(GridSlot s, uint16_t N=80){
  uint32_t sx=0, sy=0;
  uint16_t minX=65535, maxX=0, minY=65535, maxY=0;
  uint16_t a;

  for(uint16_t i=0;i<N;i++){
    if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
    if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
    sx += rawX; sy += rawY;
    if (rawX<minX) minX=rawX; if (rawX>maxX) maxX=rawX;
    if (rawY<minY) minY=rawY; if (rawY>maxY) maxY=rawY;
    delay(5);
  }

  // If lever barely moved, create a tiny box to avoid degenerate polygons
  if (maxX <= minX) { maxX = minX + 5; }
  if (maxY <= minY) { maxY = minY + 5; }

  PolyNode &p = planeCal.node[s];

  // If there is no room for 4 more vertices, start over for this node.
  if (p.n + 4 > MAX_POLY_VERTS) {
    p.n     = 0;
    p.valid = false;
  }

  uint8_t base = p.n;
  p.x[base+0] = minX; p.y[base+0] = minY;
  p.x[base+1] = maxX; p.y[base+1] = minY;
  p.x[base+2] = maxX; p.y[base+2] = maxY;
  p.x[base+3] = minX; p.y[base+3] = maxY;
  p.n += 4;
  p.valid = (p.n >= 3);
}

// Main classifier: choose first polygon that contains (x,y).
// If none contains it, keep previous node.
GridSlot classifyGrid(uint16_t x, uint16_t y){
  GridSlot best = gridNode;
  bool found = false;

  // Priority is the enum order: N, selectors, then gear slots.
  for (int i=0;i<GS_COUNT;i++){
    const auto &p = planeCal.node[i];
    if (!p.valid) continue;
    if (pointInPoly(p, x, y, GRID_POLY_PAD)) {
      best  = (GridSlot)i;
      found = true;
      break;          // first matching polygon wins
    }
  }

  if (!found) return gridNode;    // nothing matched -> remain where we are

  gridNode = best;
  return gridNode;
}

// ---------------- Wi-Fi helpers ----------------
void logWifiStatus();

void logWifiStatus(){
  if (WiFi.getMode()==WIFI_MODE_STA && WiFi.status()==WL_CONNECTED){
    IPAddress ip = WiFi.localIP();
    logf("[WiFi] STA connected ssid=%s ip=%u.%u.%u.%u", wifiSsid.c_str(), ip[0],ip[1],ip[2],ip[3]);
  } else if (WiFi.getMode()==WIFI_MODE_AP){
    IPAddress ip = WiFi.softAPIP();
    logf("[WiFi] AP mode ssid=%s ip=%u.%u.%u.%u", WiFi.softAPSSID().c_str(), ip[0],ip[1],ip[2],ip[3]);
  } else {
    logln("[WiFi] OFF");
  }
}

void wifiJoinOrAP(){
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  logf("[WiFi] joining ssid=%s ...", wifiSsid.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis()-t0) < 8000){
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED){
    wifiUp = true;
    logWifiStatus();
  } else {
    logln("[WiFi] join failed, starting AP only");
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(wifiAPSsid.c_str(), wifiAPPass.c_str());
    wifiUp = true;
    logWifiStatus();
  }
}

void maintainWifi(){
  if (WiFi.getMode()==WIFI_MODE_STA && WiFi.status()!=WL_CONNECTED){
    static uint32_t lastTry=0;
    uint32_t now = millis();
    if (now-lastTry>10000){
      lastTry = now;
      logln("[WiFi] STA lost, retrying join...");
      WiFi.disconnect();
      WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    }
  }
}

// ---------------- Logging ring buffer ----------------
struct LogEntry { uint32_t t; uint16_t code; };
constexpr size_t LOG_SIZE=256; LogEntry logBuf[LOG_SIZE]; size_t logHead=0;

void logGear(uint16_t g){
  logBuf[logHead].t = millis();
  logBuf[logHead].code = g;
  logHead = (logHead+1) % LOG_SIZE;
}

// heuristics
inline bool magOK(uint16_t mag) { return (mag >= 1000 && mag <= 3500); }

// ---------------- HTTP pages ----------------
String htmlHeader(const char* title){
  String s;
  s += "<!DOCTYPE html><html><head><meta charset='utf-8'><title>";
  s += title;
  s += "</title><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<style>body{font-family:sans-serif;margin:12px;}button{margin:2px 0;padding:4px 8px;}input{margin:2px 0;}pre{font-size:12px;}</style>";
  s += "</head><body>";
  return s;
}

void pageIndex(){
  String s = htmlHeader("MTZ Shift Detector");
  s += "<h3>MTZ Shift Detector</h3>";
  s += "<p><a href='/status'>Status</a> | <a href='/manual'>Manual override</a> | <a href='/grid'>Polygon calibration &amp; plane</a> | <a href='/grid/tune'>Tunables</a></p>";
  s += "</body></html>";
  http.send(200,"text/html",s);
}

void pageStatus(){
  String s = htmlHeader("Status");
  s += "<h3>Status</h3>";
  s += "<pre>";
  s += "rawX=" + String(rawX) + " rawY=" + String(rawY) + "\\n";
  s += "filtX=" + String(filtX) + " filtY=" + String(filtY) + "\\n";
  s += "gearCode=" + String(gearCode) + " range=" + String(rangeName(rangeLatched)) + "\\n";
  s += "gridBest=" + String(gridBest) + " cand=" + String(gridCandidate) + " debounced=" + String(gridDebounced) + "\\n";
  s += "magX=" + String(magX) + " magY=" + String(magY) + "\\n";
  s += "wifiUp=" + String((unsigned)wifiUp) + "\\n";
  s += "</pre>";
  s += "</body></html>";
  http.send(200,"text/html",s);
}

void pageLog(){
  String s = htmlHeader("Log");
  s += "<h3>Gear log</h3><pre>";
  size_t idx = logHead;
  for (size_t i=0;i<LOG_SIZE;i++){
    const LogEntry& e = logBuf[idx];
    if (e.t != 0){
      s += String(e.t) + " ms : gear=" + String(e.code) + "\\n";
    }
    idx = (idx+1) % LOG_SIZE;
  }
  s += "</pre></body></html>";
  http.send(200,"text/html",s);
}

void pageManual(){
  String s = htmlHeader("Manual override");
  s += "<h3>Manual override</h3>";
  s += "<form method='POST' action='/api/manual/set'>";
  s += "<label><input type='checkbox' name='en' value='1'";
  if (manualOverride) s += " checked";
  s += "> Enable override</label><br>";
  s += "<label>Gear code <input type='number' name='code' min='0' max='65535' value='" + String(manualOverrideCode) + "'></label><br>";
  s += "<button type='submit'>Apply</button>";
  s += "</form>";
  s += "<p>Current gearCode=" + String(gearCode) + " (range=" + String(rangeName(rangeLatched)) + ")</p>";
  s += "</body></html>";
  http.send(200,"text/html",s);
}

// Manual override API handlers
void apiManualEnable(){
  bool en = http.hasArg("en") && (http.arg("en")=="1");
  if (en != manualOverride){
    manualOverride = en;
    logf("[MAN] override %s (via HTTP)", manualOverride ? "ENABLED" : "DISABLED");
  }
  http.send(200,"text/plain","OK\n");
}

void apiManualSet(){
  bool en = http.hasArg("en") && (http.arg("en")=="1");
  uint16_t code = (uint16_t)http.arg("code").toInt();
  if (en != manualOverride){
    manualOverride = en;
    logf("[MAN] override %s (via HTTP)", manualOverride ? "ENABLED" : "DISABLED");
  }
  if (isValidGearCode(code) && code != manualOverrideCode){
    manualOverrideCode = code;
    logf("[MAN] override code=%u (via HTTP)", manualOverrideCode);
  }
  http.send(200,"text/plain","OK\n");
}

// ---------- Polygon calibration page ----------
void pageGrid(){
  String s = htmlHeader("Polygon calibration");
  s += F(
    "<h3>Polygon calibration</h3>"
    "<table><tr><th>"
  );

  // Helper: print a button and show vertex count
  auto btn = [&](const char* key, const char* label){
    s += "<button onclick=\"rec('";
    s += key;
    s += "')\">Record ";
    s += label;
    s += "</button>";

    int idx = -1;
    for (int i = 0; i < GS_COUNT; ++i) {
      if (String(key) == GRID_NAMES[i]) { idx = i; break; }
    }
    if (idx >= 0 && planeCal.node[idx].valid){
      s += " <small>(";
      s += String(planeCal.node[idx].n);
      s += " vertices)</small>";
    }
    s += "<br>";
  };

  s += "<h4>Range selectors</h4>";
  btn("SEL_I",  "Selector I lane");
  btn("SEL_II", "Selector II lane");

  s += "<h4>Gear nodes</h4>";
  btn("N",        "Neutral");
  btn("X-1_Y+1",  "4 / 7");
  btn("X0_Y+1",   "9 (range II only)");
  btn("X+1_Y+1",  "1 / 2");
  btn("X-1_Y0",   "Neutral left");
  btn("X+1_Y0",   "Neutral right");
  btn("X-1_Y-1",  "5 / 8");
  btn("X0_Y-1",   "3 / 6");
  btn("X+1_Y-1",  "R1 / R2");

  s += F(
    "<p><button onclick=\"act('save')\">Save</button> "
    "<button onclick=\"act('load')\">Load</button> "
    "<button onclick=\"act('clear')\">Clear</button></p>"
  );
  s += F("<div id='out' style='padding:8px;background:#f4f4f4;border-radius:8px'></div></th>");

  s += R"HTML(
<th>
<canvas id="plane" width="480" height="480"
        style="border:1px solid #ccc;border-radius:8px;max-width:100%;"></canvas>
</th></tr></table>
<pre id="meta" style="background:#f7f7f7;margin-top:8px;padding:8px;border-radius:8px;white-space:pre-wrap"></pre>

<script>
function rec(name){
  fetch('/api/grid/record?slot='+encodeURIComponent(name))
    .then(r => r.text())
    .then(t => { document.getElementById('out').textContent = t; })
    .catch(e => { document.getElementById('out').textContent = 'ERR: ' + e; });
}
function act(a){
  fetch('/api/grid/' + a)
    .then(r => r.text())
    .then(t => { document.getElementById('out').textContent = t; })
    .catch(e => { document.getElementById('out').textContent = 'ERR: ' + e; });
}

(function(){
  const canvas = document.getElementById('plane');
  const ctx    = canvas.getContext('2d');

  const X_MIN   = 3500;
  const X_MAX   = 3900;
  const Y_RANGE    = 4096;
  const Y_CROP_TOP = -300;  // shows 300 units above zero
  const Y_CROP_BOT = 3700;  // shows 3700 units below zero

  function clamp(v, lo, hi){ return Math.min(hi, Math.max(lo, v)); }
  function wrapY(y){
    // Wrap so that 0 sits at the vertical center of the canvas and crop to
    // the requested vertical window (300 above zero, 3700 below zero).
    const wrapped = (y > Y_CROP_BOT) ? y - Y_RANGE : y;
    return clamp(wrapped, Y_CROP_TOP, Y_CROP_BOT);
  }

  function displayToRawY(y){
    // Convert display-domain Y (which can be negative around the wrap) back
    // into the raw 0..4095 range expected by the backend readings.
    return (y < 0) ? y + Y_RANGE : y;
  }

  // Map raw readings to canvas with X cropped to [3500,3900] and Y cropped to
  // [-300, 3700], keeping 0 at the vertical center of the canvas.
  function sx(x){
    const clamped = clamp(x, X_MIN, X_MAX);
    return ((clamped - X_MIN) / (X_MAX - X_MIN)) * canvas.width;
  }
  function sy(y){
    const wy = wrapY(y);
    const center = canvas.height / 2;

    if (wy <= 0){
      const t = wy / Y_CROP_TOP; // -300 => 1 (top), 0 => 0 (center)
      return center - t * center;
    }

    const t = wy / Y_CROP_BOT; // 0 => 0, 3700 => 1 (bottom)
    return center + t * (canvas.height - center);
  }

  function drawSnapshot(j){
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw grid background
    ctx.save();
    ctx.lineWidth = 0.5;
    ctx.globalAlpha = 0.25;
    for (let t=0; t<=4; t++){
      const xVal = X_MIN + (t/4) * (X_MAX - X_MIN);
      const yVal = Y_CROP_TOP + (t/4) * (Y_CROP_BOT - Y_CROP_TOP);

      const x = sx(xVal);
      const y = sy(displayToRawY(yVal));
      ctx.beginPath();
      ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
    }
    ctx.restore();

    // Draw polygons
    j.nodes.forEach(function(n){
      if (!n.valid || !n.verts || n.verts.length < 3) return;
      ctx.beginPath();
      n.verts.forEach(function(v, idx){
        const X = sx(v[0]);
        const Y = sy(v[1]);
        if (idx === 0) ctx.moveTo(X, Y);
        else           ctx.lineTo(X, Y);
      });
      ctx.closePath();

      // fill + stroke, slightly transparent
      ctx.save();
      ctx.globalAlpha = 0.12;
      ctx.fill();
      ctx.globalAlpha = 0.9;
      ctx.lineWidth = 1.2;
      ctx.stroke();
      ctx.restore();
    });

    // Draw current filtered point
    const px = sx(j.filtX);
    const py = sy(j.filtY);
    ctx.beginPath();
    ctx.arc(px, py, 4, 0, Math.PI*2, false);
    ctx.fill();

    ctx.font = "11px sans-serif";
    ctx.fillText("(" + j.filtX + "," + j.filtY + ")", px + 6, py - 6);
  }

  function tick(){
    fetch('/api/grid/scores')
      .then(r => r.json())
      .then(j => {
        drawSnapshot(j);
        document.getElementById('meta').textContent =
          "filt=(" + j.filtX + "," + j.filtY + ")  range=" + j.range +
          "\ninstant=" + j.gridBest + "  candidate=" + j.gridCand +
          "  debounced=" + j.gridDebounced +
          "\npolygons: " + j.nodes.map(n => n.name + ":" + n.n).join(", ");
      })
      .catch(e => {
        document.getElementById('meta').textContent = "ERR: " + e;
      });
  }

  setInterval(tick, 250);
})();
</script>
)HTML";

  s += F("</body></html>");
  http.send(200,"text/html",s);
}

// ---------- 2D plane / polygon scores ----------
static String gridScoresJSON(){
  String out; out.reserve(2200);
  out += '{';
  out += "\"filtX\":" + String(filtX) + ",";
  out += "\"filtY\":" + String(filtY) + ",";
  out += "\"enter_th\":" + String(GRID_ENTER_TH) + ",";
  out += "\"exit_th\":"  + String(GRID_EXIT_TH)  + ",";
  out += "\"range\":"    + String((uint16_t)rangeLatched) + ",";
  out += "\"gridBest\":" + String(gridBest) + ",";
  out += "\"gridCand\":" + String(gridCandidate) + ",";
  out += "\"gridDebounced\":" + String(gridDebounced) + ",";

  // list nodes + find best score for debug
  uint32_t bestSc = 0xFFFFFFFFu;
  int      bestIdx = -1;

  out += "\"nodes\":[";
  for (int i=0;i<GS_COUNT;i++){
    const auto &p = planeCal.node[i];
    uint32_t sc = polyScore(i, filtX, filtY);
    if (p.valid && sc < bestSc) { bestSc = sc; bestIdx = i; }

    if (i) out += ',';
    out += '{';
    out += "\"id\":"    + String(i) + ",";
    out += "\"name\":\"" + String(GRID_NAMES[i]) + "\",";
    out += "\"valid\":" + String(p.valid ? 1 : 0) + ",";
    out += "\"score\":" + String(sc) + ",";
    out += "\"n\":"     + String(p.n) + ",";
    out += "\"verts\":[";
    for (uint8_t k=0;k<p.n;k++){
      if (k) out += ',';
      out += "[" + String(p.x[k]) + "," + String(p.y[k]) + "]";
    }
    out += "]";
    out += '}';
  }
  out += "],";

  out += "\"best\":{";
  out += "\"id\":"    + String(bestIdx) + ",";
  out += "\"score\":" + String(bestSc);
  out += "}";
  out += '}';
  return out;
}

// ---------- 2D plane / polygon test page ----------
void pageGridTest(){
  // Legacy path keeps compatibility but now serves the combined calibration page
  pageGrid();
}

// ---- Tunables page (/grid/tune) and Modbus polling ----
void pageGridTune(){
  String s = htmlHeader("Grid tuning");
  s += F("<h3>Runtime tuning</h3><p>Adjust debounce and thresholds. Values apply immediately.</p>");
  s += "<form method='POST' action='/grid/tune'>";
  s += "<label>GRID_STABLE_MS <input name='stable' type='number' min='0' max='65535' value='" + String(GRID_STABLE_MS) + "'></label><br>";
  s += "<label>GEAR_DEBOUNCE_MS <input name='gdb' type='number' min='0' max='65535' value='" + String(GEAR_DEBOUNCE_MS) + "'></label><br>";
  s += "<label>RANGE_DEBOUNCE_MS <input name='rdb' type='number' min='0' max='65535' value='" + String(RANGE_DEBOUNCE_MS) + "'></label><br>";
  s += "<label>GRID_ENTER_TH <input name='enter' type='number' min='0' max='65535' value='" + String(GRID_ENTER_TH) + "'></label><br>";
  s += "<label>GRID_EXIT_TH <input name='exit' type='number' min='0' max='65535' value='" + String(GRID_EXIT_TH) + "'></label><br>";
  s += "<label>GRID_POLY_PAD <input name='pad' type='number' min='0' max='1023' value='" + String(GRID_POLY_PAD) + "'></label><br>";
  s += "<button type='submit'>Save</button>";
  s += "</form></body></html>";
  http.send(200,"text/html",s);
}

void apiGridTunePost(){
  if (http.hasArg("stable")) GRID_STABLE_MS = http.arg("stable").toInt();
  if (http.hasArg("gdb"))    GEAR_DEBOUNCE_MS = http.arg("gdb").toInt();
  if (http.hasArg("rdb"))    RANGE_DEBOUNCE_MS = http.arg("rdb").toInt();
  if (http.hasArg("enter"))  GRID_ENTER_TH = http.arg("enter").toInt();
  if (http.hasArg("exit"))   GRID_EXIT_TH  = http.arg("exit").toInt();
  if (http.hasArg("pad"))    GRID_POLY_PAD = (uint16_t)http.arg("pad").toInt();
  // reflect to Modbus
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);
  mb.Hreg(23, (uint16_t)GRID_POLY_PAD);
  http.send(200, "text/html", "Saved. <a href='/grid/tune'>Back</a>");
}

void pollGridTunablesFromModbus(){
  GRID_STABLE_MS    = mb.Hreg(18);
  GEAR_DEBOUNCE_MS  = mb.Hreg(19);
  RANGE_DEBOUNCE_MS = mb.Hreg(20);
  GRID_ENTER_TH     = mb.Hreg(21);
  GRID_EXIT_TH      = mb.Hreg(22);
  GRID_POLY_PAD     = mb.Hreg(23);
}

// ---- Grid scores API ----
void apiGridScores(){
  String j = gridScoresJSON();
  http.send(200, "application/json", j);
}

GridSlot parseGrid(const String& n){
  for (int i=0;i<GS_COUNT;i++){
    if (n.equalsIgnoreCase(GRID_NAMES[i])) return (GridSlot)i;
  }
  return GS_N;
}

void apiGridRecord(){
  if (!http.hasArg("slot")) { http.send(400,"text/plain","slot?="); return; }
  GridSlot gs = parseGrid(http.arg("slot"));
  sampleGrid(gs);
  PolyNode &p = planeCal.node[gs];
  char buf[160];
  snprintf(buf,sizeof(buf),
           "OK %s: vertices=%u (max=%u)\n",
           GRID_NAMES[gs], (unsigned)p.n, (unsigned)MAX_POLY_VERTS);
  http.send(200,"text/plain",buf);
}
void apiGridSave(){ saveGrid(); http.send(200,"text/plain","Saved\n"); }
void apiGridLoad(){ bool ok=loadGrid(); http.send(200,"text/plain", ok?"Loaded\n":"No data\n"); }
void apiGridClear(){ clearGrid(); http.send(200,"text/plain","Cleared\n"); }

// ---------------- Modbus writeout ----------------
void applyModbus(){
  mb.Hreg(0, rawX);
  mb.Hreg(1, rawY);
  mb.Hreg(2, filtX);
  mb.Hreg(3, filtY);
  mb.Hreg(4, gearCode);

  uint16_t st=0;
  // calibrated: require selectors + center + two gear nodes to have polygons
  {
    int v = 0;
    if (planeCal.node[GS_SEL_I].valid)   v++;
    if (planeCal.node[GS_SEL_II].valid)  v++;
    if (planeCal.node[GS_N].valid)       v++;
    if (planeCal.node[GS_X0_Yp1].valid)  v++;
    if (planeCal.node[GS_X0_Yn1].valid)  v++;
    if (v >= 5) st |= 1<<0;
  }
  if (magOK(magX)) st |= 1<<1; else st |= 1<<5; // fault_X
  if (magOK(magY)) st |= 1<<2; else st |= 1<<6; // fault_Y
  if (inSelectorLane) st |= 1<<3;
  if (wifiUp) st |= 1<<4;
  if (manualOverride) st |= 1<<7;        // manual override flag
  if (gridMode) st |= 1<<8;               // using grid classifier
  statusBits = st;
  mb.Hreg(5, statusBits);

  // write tunables as holding regs for external monitoring
  mb.Hreg(16, (uint16_t)rangeLatched);    // 40017
  mb.Hreg(17, (uint16_t)gridNode);         // 40018
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);
  mb.Hreg(23, (uint16_t)GRID_POLY_PAD);

  // Mirror to Discrete Inputs (1xxxx)
  updateDiscreteInputs();
}

// ---------------- Setup/loop ----------------
float    alpha = 0.2f;           // can be made configurable
bool     fInit=false; uint16_t fx=0, fy=0;

void setup(){
  delay(3000);
  LOG_BEGIN();
  logln("[BOOT] MTZ Shift Detector v5");

  // I2C
  WireX.begin(SDA_X, SCL_X, 400000);
  WireY.begin(SDA_Y, SCL_Y, 400000);

  prefs.begin("mtz_shift", false);
  loadGrid();

  // WiFi + HTTP
  logln("[BOOT] Wait for AP to start");
  delay(27000);
  wifiJoinOrAP();

  http.on("/", pageIndex);
  http.on("/status", pageStatus);
  http.on("/log", pageLog);
  http.on("/manual", pageManual);
  http.on("/api/manual/enable", apiManualEnable);
  http.on("/api/manual/set",    apiManualSet);

  http.on("/grid",        pageGrid);
  http.on("/grid/test",   pageGridTest);
  http.on("/grid/tune",   HTTP_GET,  pageGridTune);
  http.on("/grid/tune",   HTTP_POST, apiGridTunePost);

  http.on("/api/grid/record", apiGridRecord);
  http.on("/api/grid/save",    apiGridSave);
  http.on("/api/grid/load",    apiGridLoad);
  http.on("/api/grid/clear",   apiGridClear);
  http.on("/api/grid/scores",  apiGridScores);

  http.begin();

  // Modbus
  mb.server();
  mb.addHreg(0,0,32);    // rawX,rawY,filtX,filtY,gearCode,statusBits,rangeLatched,gridNode,GRID_* etc.
  mb.addIsts(0, false);  // discrete inputs 1xxxx
  mb.addIsts(1, false);
  mb.addIsts(2, false);
  mb.addIsts(3, false);
  mb.addIsts(4, false);
  mb.addIsts(5, false);
  mb.addIsts(6, false);
  mb.addIsts(7, false);
  mb.addIsts(8, false);
  mb.addIsts(9, false);
  mb.addIsts(20,false);
  mb.addIsts(21,false);

  // Seed tunables to Modbus
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);
  mb.Hreg(23, (uint16_t)GRID_POLY_PAD);
}

void loop(){
  http.handleClient();
  maintainWifi();

  // Modbus override + tunables
  pollOverrideFromModbus();
  pollGridTunablesFromModbus();

  // Read sensors
  uint16_t a;
  if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
  if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
  i2cRead16(WireX, REG_MAG_H, REG_MAG_L, magX);
  i2cRead16(WireY, REG_MAG_H, REG_MAG_L, magY);
  i2cRead8(WireX, REG_AGC, (uint8_t&)agcX);
  i2cRead8(WireY, REG_AGC, (uint8_t&)agcY);

  // Simple LPF
  if (!fInit){ fx=rawX; fy=rawY; fInit=true; }
  filtX = (uint16_t)((1.0f-alpha)*fx + alpha*rawX); fx=filtX;
  filtY = (uint16_t)((1.0f-alpha)*fy + alpha*rawY); fy=filtY;

  // ----- Debounced grid classification & range/gear updates -----
  uint32_t nowMs = millis();
  GridSlot gsNow = classifyGrid(filtX, filtY);
  inSelectorLane = (gsNow == GS_SEL_I || gsNow == GS_SEL_II);
  gridBest = (int)gsNow;
  if ((int)gsNow != gridCandidate){ gridCandidate = (int)gsNow; gridCandSince = nowMs; }
  if (((nowMs - gridCandSince) >= GRID_STABLE_MS) && ((int)gsNow != gridDebounced)){
    gridDebounced = (int)gsNow;
    // Range latch with debounce
    if (gsNow == GS_SEL_I && (nowMs - lastRangeChangeMs) >= RANGE_DEBOUNCE_MS){ rangeLatched = R_I;  lastRangeChangeMs = nowMs; }
    if (gsNow == GS_SEL_II && (nowMs - lastRangeChangeMs) >= RANGE_DEBOUNCE_MS){ rangeLatched = R_II; lastRangeChangeMs = nowMs; }
    // Gear from debounced node
    uint16_t cand = gearFromGrid(gsNow, rangeLatched);
    if (cand != gearCode && (nowMs - lastGearChangeMs) >= GEAR_DEBOUNCE_MS){
      gearCode = cand; lastGearChangeMs = nowMs;
      logGear(gearCode); logf("[GEAR] code=%u (debounced)", gearCode);
    }
  }

  // status → Modbus
  applyModbus();

  // diagnostics edge logs
  bool mX = magOK(magX), mY = magOK(magY);
  if (mX != prevMagOkX || mY != prevMagOkY){ logf("[MAG] X:%u(%s) Y:%u(%s)", magX, mX?"OK":"BAD", magY, mY?"OK":"BAD"); prevMagOkX=mX; prevMagOkY=mY; }
  if (errorCode != prevErrorLogged){ logf("[ERR] code=%u", errorCode); prevErrorLogged=errorCode; }

  // periodic status @1Hz
  if (millis()-lastPrintMs >= 1000){
    int rssi = (WiFi.getMode()==WIFI_MODE_STA && WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : 0;
    logf("[STAT] rawX=%u rawY=%u filtX=%u filtY=%u gear=%u range=%u grid=%u AGC=%u/%u MAG=%u/%u wifiUp=%u RSSI=%d manual=%u mCode=%u",
          rawX,rawY,filtX,filtY,gearCode,(unsigned)rangeLatched,(unsigned)gridNode,agcX,agcY,magX,magY,(unsigned)wifiUp,rssi,
          (unsigned)manualOverride, manualOverrideCode);
    lastPrintMs = millis();
  }

  mb.task();
  delay(20); // ~50 Hz internal loop (external Modbus 1..20 Hz OK)
}
