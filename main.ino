/*
  MTZ Shift Detector — ESP32‑S2 (LOLIN S2 mini) v1.1

  Two AS5600 angle sensors (one per axis) -> classify tractor gear position
  Adds grid-based shifter model with range latch (I/II) and hysteresis.
  Manual override page allows forcing gear code for remote testing.

  Exposes Modbus TCP holding registers and a minimal Wi‑Fi HTTP UI for
  calibration, status, manual override, grid calibration, and network settings.

  ───────────────────────────────────────────────────────────────────────────
  Hardware notes (quick schematic snippet)
  ───────────────────────────────────────────────────────────────────────────
  • MCU: LOLIN S2 mini (ESP32‑S2). Power via USB‑C 5 V; on‑board 3V3 LDO.
  • Sensors: 2 × AS5600 (address 0x36 each) on SEPARATE I²C controllers.
      I²C‑X: SDA=GPIO33, SCL=GPIO35
      I²C‑Y: SDA=GPIO9,  SCL=GPIO11
      (Remappable if layout requires. Avoid GPIO19/20 — USB D−/D+.)
  • Pull‑ups: 2.2–4.7 kΩ from SDA/SCL to 3V3 on EACH bus near the MCU.
  • Decoupling: each AS5600 gets 100 nF + 1 µF close to VDD.
  • Magnets: 6–8 mm diametric, 2–3 mm thick; air‑gap 1.0–2.5 mm; align axes.
  • ESD/EMI: optional ESD diodes to 3V3/GND on SDA/SCL; twist SDA/SCL pairs.
  • Power: AS5600 VDD = 3V3. All logic is 3.3 V only.

  ───────────────────────────────────────────────────────────────────────────
  Modbus TCP register map (all Holding, big‑endian 16‑bit; index=Hreg-40001)
  ───────────────────────────────────────────────────────────────────────────
  40001 RAW_X        u16  (0..4095)           [index 0]
  40002 RAW_Y        u16                       1
  40003 FILT_X       u16                       2
  40004 FILT_Y       u16                       3
  40005 GEAR_CODE    u16  0=N, 1..9, 101=R1, 102=R2, 65535=unknown   4
  40006 STATUS       u16  b0:calib b1:magOK_X b2:magOK_Y b3:selector
                           b4:wifiUp b5:fault_X b6:fault_Y b7:manual
                           b8:gridMode
  40007 AGC_X        u16                       6
  40008 MAG_X        u16                       7
  40009 AGC_Y        u16                       8
  40010 MAG_Y        u16                       9
  40011 UPTIME_LO    u16                      10
  40012 UPTIME_HI    u16                      11
  40013 FW_VER       u16  0x0100 = v1.0.0     12
  40014 ERROR        u16  last error code     13
  40015 OVERRIDE_CTRL u16 bit0: enable        14
  40016 OVERRIDE_CODE u16 code (0..,101,102,65535) 15
  40017 RANGE        u16  0=unknown,1=I,2=II  16
  40018 GRID_NODE    u16  enum id of grid     17

  Discrete Inputs (1xxxx) — one-hot mirror of GEAR_CODE for compatibility
  ───────────────────────────────────────────────────────────────────────────
  DI0   (10001) = Neutral
  DI1   (10002) = Gear 1
  DI2   (10003) = Gear 2
  DI3   (10004) = Gear 3
  DI4   (10005) = Gear 4
  DI5   (10006) = Gear 5
  DI6   (10007) = Gear 6
  DI7   (10008) = Gear 7
  DI8   (10009) = Gear 8
  DI9   (10010) = Gear 9
  DI20  (10021) = Reverse 1 (R1)
  DI21  (10022) = Reverse 2 (R2)

  HTTP UI
  ───────────────────────────────────────────────────────────────────────────
  • /           : index links
  • /status     : JSON live data
  • /log        : text ring‑buffer log of gear changes
  • /grid       : GRID calibration (selectors, N, six grid nodes)
      \-> /api/grid/record?slot=SEL_I|SEL_II|X±1_Y±1|X±1_Y0|N
      \-> /api/grid/save | /api/grid/load | /api/grid/clear       : GRID calibration (selectors, N, six grid nodes)
      \-> /api/grid/record?slot=SEL_I|SEL_II|X±1_Y±1|X±1_Y0|N
      \-> /api/grid/save | /api/grid/load | /api/grid/clear
  • /manual     : manual override enable/disable and force gear
      \-> /api/manual/enable?on=0|1
      \-> /api/manual/set?code=0..9|101|102|65535
  • /net        : set Wi‑Fi password for SSID "Tracktor" and reboot

  Build: Arduino IDE (ESP32 board core), select LOLIN S2 mini (ESP32‑S2).
  Libraries: built‑in WiFi, WebServer; install "Modbus-ESP8266".
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
constexpr uint8_t REG_STATUS  = 0x0B;   // MD ML MH bits
constexpr uint8_t REG_AGC     = 0x1A;
constexpr uint8_t REG_MAG_H   = 0x1B;   // MAGNITUDE (12 bit)
constexpr uint8_t REG_MAG_L   = 0x1C;
constexpr uint8_t REG_ANG_H   = 0x0E;   // ANGLE (12 bit)
constexpr uint8_t REG_ANG_L   = 0x0F;

// ---------------- Wi‑Fi configuration ----------------
String wifiSsid = "Tracktor";          // fixed SSID per requirements
String wifiPass = "ujooCeeph9@$";      // clean literal
static constexpr char WIFI_AP_SSID_PREFIX[] = "TractorShift";
static constexpr char WIFI_AP_PASS[]        = "shift1234";
const unsigned long DHCP_TIMEOUT_MS = 10000; // milliseconds to wait for DHCP

// ---------------- Logging helpers ----------------
static inline void LOG_BEGIN(){ Serial.begin(115200); delay(200); }
static inline void logln(const char* s){ Serial.println(s); }
static inline void logf(const char* fmt, ...){ char buf[256]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap); Serial.println(buf); }

// ---------------- Global objects ----------------
TwoWire WireX = TwoWire(0);
TwoWire WireY = TwoWire(1);
bool   wifiUp   = false;

// ---------------- HTTP server ----------------
WebServer http(80);

// ---------------- Modbus TCP ----------------
ModbusIP mb;

// ---------------- NVS ----------------
Preferences prefs; // namespace "shift"

// ---------------- Types ----------------

// live measurements
uint16_t rawX=0, rawY=0, filtX=0, filtY=0;
uint16_t agcX=0, agcY=0, magX=0, magY=0;
uint16_t gearCode = 0;           // 0=N, 1..9, 101=R1, 102=R2, 65535=unknown
uint16_t statusBits = 0;         // see map
uint16_t errorCode = 0;          // last error
bool     inSelectorLane=false;   // runtime flag

// ---- Grid / Range debounce additions ----
enum RangeSel : uint8_t { R_UNKNOWN=0, R_I=1, R_II=2 };

// Debounce parameters (tune as needed)
uint32_t GRID_STABLE_MS = 120;  // candidate must be stable this long to accept
uint32_t GEAR_DEBOUNCE_MS = 200;  // min time between gear updates
uint32_t RANGE_DEBOUNCE_MS = 200;  // min time between range latches

// Debounce state (GridSlot is declared later; use int to avoid forward-decl issues here)
int        gridBest      = 0;   // instantaneous best (from classifyGrid)
int        gridCandidate = 0;   // currently considered node
int        gridDebounced = 0;   // accepted node after debounce
uint32_t   gridCandSince = 0;
uint32_t   lastGearChangeMs  = 0;
uint32_t   lastRangeChangeMs = 0;

// debug / telemetry helpers
uint32_t lastPrintMs = 0;
bool prevMagOkX = true, prevMagOkY = true;
uint16_t prevGearLogged = 65535;
uint16_t prevErrorLogged = 0xFFFF;

// filter
float    alpha = 0.2f;           // can be made configurable
bool     fInit=false; uint16_t fx=0, fy=0;

// logging (ring)
struct LogEntry { uint32_t t; uint16_t code; };
constexpr size_t LOG_SIZE=256; LogEntry logBuf[LOG_SIZE]; size_t logHead=0;

// heuristics
inline bool magOK(uint16_t mag) { return (mag >= 1000 && mag <= 3500); }

// ---------------- I²C helpers ----------------
bool i2cRead8(TwoWire& W, uint8_t reg, uint8_t& out){
  W.beginTransmission(AS5600_ADDR); W.write(reg);
  if (W.endTransmission(false) != 0) return false;
  if (W.requestFrom(AS5600_ADDR, 1) != 1) return false;
  out = W.read(); return true;
}

bool i2cRead16_12b(TwoWire& W, uint8_t regH, uint8_t regL, uint16_t& out){
  W.beginTransmission(AS5600_ADDR); W.write(regH);
  if (W.endTransmission(false) != 0) return false;
  if (W.requestFrom(AS5600_ADDR, 1) != 1) return false;
  uint8_t hi = W.read();
  W.beginTransmission(AS5600_ADDR); W.write(regL);
  if (W.endTransmission(false) != 0) return false;
  if (W.requestFrom(AS5600_ADDR, 1) != 1) return false;
  uint8_t lo = W.read();
  out = ((uint16_t)(hi & 0x0F) << 8) | lo; // 12‑bit value right‑aligned
  return true;
}

// ---------------- Filtering & logging ----------------
static inline uint16_t lpf(uint16_t prev, uint16_t now){
  return (uint16_t)((1.0f - alpha) * prev + alpha * now);
}

void logGear(uint16_t code){ logBuf[logHead % LOG_SIZE] = { (uint32_t)(millis()/1000), code }; logHead++; }

// (legacy per-gear calibration removed)

// (legacy clearCal removed)

// (legacy sampleSlot removed)

// (legacy per-gear classification removed)

// (legacy bestSlot removed)

// (legacy classify removed)

// ---------------- Discrete Inputs mirror (GEAR_CODE) ----------------
void updateDiscreteInputs(){
  const uint8_t used[] = {0,1,2,3,4,5,6,7,8,9,20,21};
  for (uint8_t i = 0; i < sizeof(used); ++i) { mb.Ists(used[i], false); }
  if (gearCode == 0) { mb.Ists(0, true); return; }
  if (gearCode >= 1 && gearCode <= 9) { mb.Ists((uint8_t)gearCode, true); return; }
  if (gearCode == 101) { mb.Ists(20, true); return; }
  if (gearCode == 102) { mb.Ists(21, true); return; }
}

// ---------------- Manual override ----------------
bool     manualOverride = false;       // when true, sensors are ignored
uint16_t manualOverrideCode = 65535;   // default "unknown"

inline bool isValidGearCode(uint16_t c){
  return (c == 0) || (c >= 1 && c <= 9) || (c == 101) || (c == 102) || (c == 65535);
}

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
    // Middle row y=0 -> Neutral across columns
    case GS_Xn1_Y0:    return 0;
    case GS_Xp1_Y0:    return 0;
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

struct GridCal { uint16_t x=2048, y=2048, rx=220, ry=220; bool valid=false; };
struct GridPack {
  uint32_t magic = 0xA11D5E77; // 'all set'
  uint16_t version = 0x0100;
  GridCal node[GS_COUNT];
};

GridPack gridCal;
bool     gridMode = true;         // prefer grid classifier
RangeSel rangeLatched = R_UNKNOWN;
GridSlot gridNode    = GS_N;

// Hysteresis thresholds (score is scaled ~256 * (dx^2/rx^2 + dy^2/ry^2))
uint32_t GRID_ENTER_TH = 7000;    // stricter enter
uint32_t GRID_EXIT_TH  = 12000;   // looser exit to reduce chatter

static inline uint32_t gridScore(uint8_t idx, uint16_t x, uint16_t y){
  const auto &c = gridCal.node[idx];
  int32_t dx = (int32_t)x - (int32_t)c.x;
  int32_t dy = (int32_t)y - (int32_t)c.y;
  uint32_t rx2 = (uint32_t)((c.rx>20?c.rx:20)); rx2*=rx2;
  uint32_t ry2 = (uint32_t)((c.ry>20?c.ry:20)); ry2*=ry2;
  return (uint32_t)((dx*dx)*256UL/rx2 + (dy*dy)*256UL/ry2);
}

bool loadGrid(){ GridPack tmp; size_t sz = prefs.getBytes("grid", &tmp, sizeof(tmp)); if (sz==sizeof(tmp) && tmp.magic==0xA11D5E77){ gridCal=tmp; return true; } return false; }
void saveGrid(){ prefs.putBytes("grid", &gridCal, sizeof(gridCal)); }
void clearGrid(){ GridPack fresh; gridCal=fresh; saveGrid(); }

void sampleGrid(GridSlot s, uint16_t N=80){
  uint32_t sx=0, sy=0; uint16_t minX=65535, maxX=0, minY=65535, maxY=0;
  uint16_t a;
  for(uint16_t i=0;i<N;i++){
    if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
    if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
    sx += rawX; sy += rawY;
    if (rawX<minX) minX=rawX; if (rawX>maxX) maxX=rawX;
    if (rawY<minY) minY=rawY; if (rawY>maxY) maxY=rawY;
    delay(5);
  }
  uint16_t mx = (uint16_t)(sx/N), my=(uint16_t)(sy/N);
  uint16_t rx = (uint16_t)((maxX>minX ? (maxX-minX)/2 : 20)); if (rx<20) rx=20;
  uint16_t ry = (uint16_t)((maxY>minY ? (maxY-minY)/2 : 20)); if (ry<20) ry=20;
  gridCal.node[s].x=mx; gridCal.node[s].y=my; gridCal.node[s].rx=rx; gridCal.node[s].ry=ry; gridCal.node[s].valid=true;
}



GridSlot classifyGrid(uint16_t x, uint16_t y){
  uint32_t bestSc = 0xFFFFFFFFu; GridSlot bestGs=GS_N; bool found=false;
  for (int i=0;i<GS_COUNT;i++){
    if (!gridCal.node[i].valid) continue;
    uint32_t sc = gridScore((uint8_t)i, x, y);
    if (sc < bestSc){ bestSc=sc; bestGs=(GridSlot)i; found=true; }
  }
  if (!found) return gridNode; // keep last
  uint32_t th = (bestGs==gridNode) ? GRID_EXIT_TH : GRID_ENTER_TH;
  if (bestSc > th) return gridNode; // not confident enough to switch
  gridNode = bestGs;
  if (gridNode == GS_SEL_I)  rangeLatched = R_I;
  if (gridNode == GS_SEL_II) rangeLatched = R_II;
  return gridNode;
}

// ---------------- Wi‑Fi helpers ----------------
void logWifiStatus();

void wifiLoadCreds() {
  String saved = prefs.getString("pass", "");
  if (saved.length() > 0) wifiPass = saved;
}

void logWifiStatus(){
  if (WiFi.getMode()==WIFI_MODE_STA && WiFi.status()==WL_CONNECTED){
    IPAddress ip = WiFi.localIP();
    logf("[WiFi] STA connected ssid=%s ip=%u.%u.%u.%u", wifiSsid.c_str(), ip[0],ip[1],ip[2],ip[3]);
  } else if (WiFi.getMode()==WIFI_MODE_AP){
    IPAddress ip = WiFi.softAPIP();
    logf("[WiFi] AP mode ssid=%s ip=%u.%u.%u.%u", WiFi.softAPSSID().c_str(), ip[0],ip[1],ip[2],ip[3]);
  } else {
    logln("[WiFi] disconnected");
  }
}

void wifiJoinOrAP(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("mtz-shift");
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < DHCP_TIMEOUT_MS) { delay(50); }
  wifiUp = (WiFi.status() == WL_CONNECTED);
  if (!wifiUp){
    WiFi.mode(WIFI_AP);
    char ap[32]; uint64_t mac = ESP.getEfuseMac();
    snprintf(ap, sizeof(ap), "%s-%02X%02X", WIFI_AP_SSID_PREFIX,
             (unsigned)((mac>>8)&0xFF), (unsigned)(mac&0xFF));
    WiFi.softAP(ap, WIFI_AP_PASS);
    wifiUp=false;
  }
  logWifiStatus();
}

void maintainWifi(){
  bool prev = wifiUp;
  if (WiFi.getMode() == WIFI_MODE_STA){
    wifiUp = (WiFi.status() == WL_CONNECTED);
    if (!wifiUp){ wifiJoinOrAP(); return; }
  } else { wifiUp = false; }
  if (wifiUp != prev) logWifiStatus();
}

// ---------------- HTTP pages ----------------
String htmlHeader(const char* title){
  String s = F("<!doctype html><html><head><meta charset='utf-8'>");
  s += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  s += F("<style>body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:20px}button{padding:8px 12px;margin:4px}code{background:#eee;padding:2px 4px;border-radius:4px}</style>");
  s += "<title>"; s += title; s += "</title></head><body>";
  return s;
}

void pageIndex(){
  String s = htmlHeader("MTZ Shift — Index");
  s += F("<h2>MTZ Shift Detector</h2><p><a href='/status'>/status</a> · <a href='/grid/tune'>/grid/tune</a> · <a href='/grid'>/grid</a> · <a href='/grid/test'>/grid/test</a> · <a href='/net'>/net</a> · <a href='/log'>/log</a> · <a href='/manual'>/manual</a></p>");
  s += F("<p>Modbus TCP on port <code>502</code>. Poll holding registers 40001…</p>");
  s += F("</body></html>"); http.send(200, "text/html", s);
}

void pageStatus(){
  char buf[768];
  snprintf(buf, sizeof(buf),
    "{\n"
    "\"rawX\":%u,\n\"rawY\":%u,\n\"filtX\":%u,\n\"filtY\":%u,\n"
    "\"gear\":%u,\n\"selector\":%s,\n\"wifiUp\":%s,\n"
    "\"agcX\":%u,\n\"agcY\":%u,\n\"magX\":%u,\n\"magY\":%u,\n"
    "\"statusBits\":%u,\n"
    "\"manual\":%s,\n\"manualCode\":%u,\n"
    "\"range\":%u,\n\"gridNode\":%u\n"
    "}\n",
    rawX,rawY,filtX,filtY,gearCode,
    inSelectorLane?"true":"false",wifiUp?"true":"false",
    agcX,agcY,magX,magY,statusBits,
    manualOverride?"true":"false",manualOverrideCode,
    (unsigned)rangeLatched,(unsigned)gridNode);
  http.send(200, "application/json", buf);
}

void pageLog(){
  String s; s.reserve(2048);
  s += F("time_s,gear\n");
  size_t n = min((size_t)LOG_SIZE, logHead);
  for (size_t i=0;i<n;i++){
    auto &e = logBuf[(logHead - n + i) % LOG_SIZE];
    s += String(e.t); s += ","; s += String(e.code); s += "\n";
  }
  http.send(200, "text/plain", s);
}

void pageNet(){
  String s = htmlHeader("Network");
  s += F("<h3>Wi‑Fi</h3><p>Target SSID: <code>Tracktor</code></p>");
  s += F("<form method='POST' action='/net'>Password: <input name='pass' type='password' placeholder='(leave empty for open)'><button type='submit'>Save & Reboot</button></form>");
  s += F("</body></html>");
  http.send(200, "text/html", s);
}

void pageNetPost(){
  if (http.hasArg("pass")) {
    wifiPass = http.arg("pass");
    prefs.putString("pass", wifiPass);
  }
  http.send(200, "text/html", F("Saved. Rebooting…<script>setTimeout(()=>location.href='/',1500)</script>"));
  delay(500);
  ESP.restart();
}

// ---------- Manual override page ----------
void pageManual(){
  String s = htmlHeader("Manual override");
  s += F("<h3>Manual override</h3>"
         "<p>When enabled, the sensor classifier is ignored and the selected gear "
         "is forced to Modbus outputs.</p>");
  s += "<p>Status: <b id='manStatus'>";
  s += manualOverride ? "ENABLED" : "disabled";
  s += "</b> · Current code: <code id='manCode'>";
  s += String(manualOverrideCode);
  s += "</code></p>";
  s += F("<p><button onclick=\"en(1)\">Enable</button> <button onclick=\"en(0)\">Disable</button></p>");
  auto btn = [&](const char* label, uint16_t code){ s += "<button onclick=\"setc("; s += String(code); s += ")\">"; s += label; s += " ("; s += String(code); s += ")</button> "; };
  s += F("<p>Force gear:</p><p>");
  btn("N", 0);
  for (int g=1; g<=9; ++g) { String L = String("G")+String(g); btn(L.c_str(), (uint16_t)g); }
  btn("R1", 101); btn("R2", 102); btn("Unknown", 65535);
  s += F("</p><div id='out' style='padding:8px;background:#f4f4f4;border-radius:8px'></div>");
  s += F(
    "<script>"
    "const out=document.getElementById('out');"
    "const st=document.getElementById('manStatus');"
    "const cd=document.getElementById('manCode');"
    "function refresh(){fetch('/status').then(r=>r.json()).then(j=>{st.textContent=j.manual?'ENABLED':'disabled'; cd.textContent=String(j.manualCode);}).catch(()=>{});}"
    "function en(on){fetch('/api/manual/enable?on='+on).then(r=>r.text()).then(t=>{out.textContent=t; refresh();});}"
    "function setc(code){fetch('/api/manual/set?code='+code).then(r=>r.text()).then(t=>{out.textContent=t; refresh();});}"
    "refresh();"
    // "setInterval(refresh,2000);" // optional live refresh
    "</script>");
  s += F("</body></html>");
  http.send(200, "text/html", s);
}

void apiManualEnable(){
  if (!http.hasArg("on")) { http.send(400, "text/plain", "on?=0|1"); return; }
  manualOverride = (http.arg("on") != "0");
  mb.Hreg(14, (uint16_t)(manualOverride ? 1 : 0));
  logf("[MAN] override %s (via HTTP)", manualOverride ? "ENABLED" : "DISABLED");
  http.send(200, "text/plain", manualOverride ? "Enabled\n" : "Disabled\n");
}

void apiManualSet(){
  if (!http.hasArg("code")) { http.send(400, "text/plain", "code?=0..9|101|102|65535"); return; }
  uint32_t v = http.arg("code").toInt();
  if (!isValidGearCode((uint16_t)v)){ http.send(400, "text/plain", "bad code\n"); return; }
  manualOverrideCode = (uint16_t)v;
  mb.Hreg(15, manualOverrideCode);
  logf("[MAN] set code=%u (via HTTP)", manualOverrideCode);
  http.send(200, "text/plain", "OK\n");
}

// ---------- GRID calibration page ----------
void pageGrid(){
  String s = htmlHeader("Grid calibration");
  s += F("<h3>Grid calibration</h3><p>Record each node with the lever parked steadily in position. After recording, the stored (x,y,rx,ry) appear next to the button.</p>");

  // Small helper: print a single button with current stored values, by GRID_NAMES key
  auto btn = [&](const char* key, const char* label){
    s += "<button onclick=\"rec('"; s += key; s += "')\">Record "; s += label; s += "</button>";
    int idx=-1; for(int i=0;i<GS_COUNT;i++){ if (String(key)==GRID_NAMES[i]) { idx=i; break; } }
    if (idx>=0 && gridCal.node[idx].valid){
      s += " <small>(x=" + String(gridCal.node[idx].x) + ", y=" + String(gridCal.node[idx].y)
         + ", rx=" + String(gridCal.node[idx].rx) + ", ry=" + String(gridCal.node[idx].ry) + ")</small>";
    }
    s += "<br>";
  };

  s += "<h4>Range selectors</h4>";
  btn("SEL_I",  "SEL_I  (x=-2,y=-1)");
  btn("SEL_II", "SEL_II (x=-2,y=+1)");

  s += "<h4>Neutral row (y=0)</h4>";
  btn("N",       "N center (x=0,y=0)");
  btn("X-1_Y0",  "(-1,0) N");
  btn("X+1_Y0",  "(+1,0) N");

  s += "<h4>Top row (y=+1)</h4>";
  btn("X-1_Y+1", "(-1,+1) 4/7");
  btn("X0_Y+1",  "(0,+1) 9 (II only)");
  btn("X+1_Y+1", "(+1,+1) 1/2");

  s += "<h4>Bottom row (y=-1)</h4>";
  btn("X-1_Y-1", "(-1,-1) 5/8");
  btn("X0_Y-1",  "(0,-1) 3/6");
  btn("X+1_Y-1", "(+1,-1) R1/R2");

  s += F("<p><button onclick=\"act('save')\">Save</button> <button onclick=\"act('load')\">Load</button> <button onclick=\"act('clear')\">Clear</button></p>");
  s += F("<div id='out' style='padding:8px;background:#f4f4f4;border-radius:8px'></div>");
  s += R"HTML(
<script>
function rec(name){
  fetch('/api/grid/record?slot='+encodeURIComponent(name))
    .then(r=>r.text())
    .then(t=>{ document.getElementById('out').textContent=t; location.reload(); })
    .catch(e=>{ document.getElementById('out').textContent='ERR: '+e; });
}
function act(a){
  fetch('/api/grid/'+a)
    .then(r=>r.text())
    .then(t=>{ document.getElementById('out').textContent=t; if(a==='load'||a==='clear'){ location.reload(); }})
    .catch(e=>{ document.getElementById('out').textContent='ERR: '+e; });
}
</script>
)HTML";
  s += F("</body></html>");
  http.send(200,"text/html",s);
}

// ---------- GRID live test & heatmap ----------
static String gridScoresJSON(){
  String out; out.reserve(1400);
  out += '{';
  out += "\"filtX\":" + String(filtX) + ",\"filtY\":" + String(filtY) + ",";
  out += "\"enter_th\":" + String(GRID_ENTER_TH) + ",\"exit_th\":" + String(GRID_EXIT_TH) + ",";
  out += "\"range\":" + String((uint16_t)rangeLatched) + ",";
  out += "\"gridBest\":" + String(gridBest) + ",";
  out += "\"gridCand\":" + String(gridCandidate) + ",";
  out += "\"gridDebounced\":" + String(gridDebounced) + ",";
  // find best and list nodes
  uint32_t bestSc = 0xFFFFFFFFu; int bestIdx=-1;
  out += "\"nodes\":[";
  for (int i=0;i<GS_COUNT;i++){
    uint32_t sc = 0xFFFFFFFFu; bool v = gridCal.node[i].valid;
    if (v) sc = gridScore(i, filtX, filtY);
    if (v && sc < bestSc){ bestSc=sc; bestIdx=i; }
    if (i) out += ',';
    out += '{';
    out += "\"id\":" + String(i) + ",\"name\":\"" + String(GRID_NAMES[i]) + "\",";
    out += "\"valid\":" + String(v?1:0) + ",\"score\":" + String(sc) + ",";
    out += "\"x\":" + String(gridCal.node[i].x) + ",\"y\":" + String(gridCal.node[i].y) + ",";
    out += "\"rx\":" + String(gridCal.node[i].rx) + ",\"ry\":" + String(gridCal.node[i].ry);
    out += '}';
  }
  out += "],";
  out += "\"best\":{";
  out += "\"id\":" + String(bestIdx) + ",\"score\":" + String(bestSc) + '}';
  out += '}';
  return out;
}

void pageGridTest(){
  String s = htmlHeader("Grid test / heatmap");
  s += R"HTML(
<h3>Grid test / heatmap</h3>
<p>Layout: mirrored + 90° CW. Rows: y=+1 (top), y=0 (Neutral row), y=-1 (bottom). Cols: x=-2, -1, 0, +1.</p>
<div id="gridwrap" style="display:grid;grid-template-columns:80px repeat(4,150px);gap:6px;align-items:stretch">
  <div></div><div style="text-align:center;font-weight:600">x=-2</div><div style="text-align:center;font-weight:600">x=-1</div><div style="text-align:center;font-weight:600">x=0</div><div style="text-align:center;font-weight:600">x=+1</div>
  <div style="display:flex;align-items:center;font-weight:600">y=+1</div>
  <div id="c_yp1_xm2"></div><div id="c_yp1_xm1"></div><div id="c_yp1_x0"></div><div id="c_yp1_xp1"></div>
  <div style="display:flex;align-items:center;font-weight:600">y=0</div>
  <div id="c_y0_xm2"></div><div id="c_y0_xm1"></div><div id="c_y0_x0"></div><div id="c_y0_xp1"></div>
  <div style="display:flex;align-items:center;font-weight:600">y=-1</div>
  <div id="c_ym1_xm2"></div><div id="c_ym1_xm1"></div><div id="c_ym1_x0"></div><div id="c_ym1_xp1"></div>
</div>
<pre id="meta" style="background:#f7f7f7;margin-top:8px;padding:8px;border-radius:8px;white-space:pre-wrap"></pre>
<script>
(function(){
  function colorFor(score){ if(score===4294967295) return '#eee'; var t=Math.min(score/15000,1.0); var h=(120*(1.0-t)); return 'hsl('+h+',70%,55%)'; }
  function cell(id, label, score){ var el=document.getElementById(id); if(!el) return; el.style.border='1px solid #ccc'; el.style.borderRadius='8px'; el.style.height='100px'; el.style.display='flex'; el.style.flexDirection='column'; el.style.justifyContent='center'; el.style.alignItems='center'; el.style.background=colorFor(score); el.innerHTML='<b>'+label+'</b><div style="font-size:12px">score:'+(score===4294967295?'—':score)+'</div>'; }
  function tick(){
    fetch('/api/grid/scores').then(function(r){return r.json();}).then(function(j){
      var map={}; for (var i=0;i<j.nodes.length;i++){ map[j.nodes[i].name]=j.nodes[i]; }
      cell('c_yp1_xm2','SEL II', (map['SEL_II']?map['SEL_II'].score:4294967295));
      cell('c_yp1_xm1','4/7',    (map['X-1_Y+1']?map['X-1_Y+1'].score:4294967295));
      cell('c_yp1_x0', '9 (II)', (map['X0_Y+1']?map['X0_Y+1'].score:4294967295));
      cell('c_yp1_xp1','1/2',    (map['X+1_Y+1']?map['X+1_Y+1'].score:4294967295));
      var sN=(map['N']?map['N'].score:4294967295), sL=(map['X-1_Y0']?map['X-1_Y0'].score:4294967295), sR=(map['X+1_Y0']?map['X+1_Y0'].score:4294967295);
      cell('c_y0_xm2','N', Math.min(sN,sL));
      cell('c_y0_xm1','N', sL);
      cell('c_y0_x0', 'N', sN);
      cell('c_y0_xp1','N', sR);
      cell('c_ym1_xm2','SEL I',  (map['SEL_I']?map['SEL_I'].score:4294967295));
      cell('c_ym1_xm1','5/8',    (map['X-1_Y-1']?map['X-1_Y-1'].score:4294967295));
      cell('c_ym1_x0', '3/6',    (map['X0_Y-1']?map['X0_Y-1'].score:4294967295));
      cell('c_ym1_xp1','R1/R2',  (map['X+1_Y-1']?map['X+1_Y-1'].score:4294967295));
      document.getElementById('meta').textContent = 'filt=(' + j.filtX + ',' + j.filtY + ')  bestIdx=' + j.best.id + ' score=' + j.best.score + '  range=' + j.range + ' gridBest=' + j.gridBest + ' gridCand=' + j.gridCand + ' gridDebounced=' + j.gridDebounced + '  enterTH=' + j.enter_th + ' exitTH=' + j.exit_th;
    }).catch(function(e){ console.log(e); });
  }
  setInterval(tick, 300);
  tick();
})();
</script>
)HTML";
  s += "</body></html>";
  http.send(200, "text/html", s);
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
  s += "<button type='submit'>Save</button></form>";
  s += F("</body></html>");
  http.send(200, "text/html", s);
}

void apiGridTunePost(){
  if (http.hasArg("stable")) GRID_STABLE_MS = http.arg("stable").toInt();
  if (http.hasArg("gdb"))    GEAR_DEBOUNCE_MS = http.arg("gdb").toInt();
  if (http.hasArg("rdb"))    RANGE_DEBOUNCE_MS = http.arg("rdb").toInt();
  if (http.hasArg("enter"))  GRID_ENTER_TH = http.arg("enter").toInt();
  if (http.hasArg("exit"))   GRID_EXIT_TH  = http.arg("exit").toInt();
  // reflect to Modbus
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);
  http.send(200, "text/html", "Saved. <a href='/grid/tune'>Back</a>");
}

void pollGridTunablesFromModbus(){
  GRID_STABLE_MS    = mb.Hreg(18);
  GEAR_DEBOUNCE_MS  = mb.Hreg(19);
  RANGE_DEBOUNCE_MS = mb.Hreg(20);
  GRID_ENTER_TH     = mb.Hreg(21);
  GRID_EXIT_TH      = mb.Hreg(22);
}

void apiGridScores(){
  String j = gridScoresJSON();
  http.send(200, "application/json", j);
}

GridSlot parseGrid(const String& n){ for (int i=0;i<GS_COUNT;i++) if (n.equalsIgnoreCase(GRID_NAMES[i])) return (GridSlot)i; return GS_N; }

void apiGridRecord(){
  if (!http.hasArg("slot")) { http.send(400,"text/plain","slot?="); return; }
  GridSlot gs = parseGrid(http.arg("slot"));
  sampleGrid(gs);
  char buf[128]; snprintf(buf,sizeof(buf),"OK %s: x=%u y=%u rx=%u ry=%u\n",
                          GRID_NAMES[gs], gridCal.node[gs].x, gridCal.node[gs].y,
                          gridCal.node[gs].rx, gridCal.node[gs].ry);
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
  // calibrated: grid-based — require selectors + center + two gear nodes
  { int v=0; if(gridCal.node[GS_SEL_I].valid) v++; if(gridCal.node[GS_SEL_II].valid) v++; if(gridCal.node[GS_N].valid) v++; if(gridCal.node[GS_X0_Yp1].valid) v++; if(gridCal.node[GS_X0_Yn1].valid) v++; if(v>=5) st |= 1<<0; }
  if (magOK(magX)) st |= 1<<1; else st |= 1<<5; // fault_X
  if (magOK(magY)) st |= 1<<2; else st |= 1<<6; // fault_Y
  if (inSelectorLane) st |= 1<<3;
  if (wifiUp) st |= 1<<4;
  if (manualOverride) st |= 1<<7;        // manual override flag
  if (gridMode) st |= 1<<8;               // using grid classifier
  statusBits = st;
  mb.Hreg(5, statusBits);

  mb.Hreg(6, agcX);
  mb.Hreg(7, magX);
  mb.Hreg(8, agcY);
  mb.Hreg(9, magY);

  uint32_t up = millis()/1000;
  mb.Hreg(10, (uint16_t)(up & 0xFFFF));
  mb.Hreg(11, (uint16_t)(up >> 16));
  mb.Hreg(12, 0x0100); // FW v1.0.0
  mb.Hreg(13, errorCode);

  // Manual override control & code
  mb.Hreg(14, (uint16_t)(manualOverride ? 1 : 0));
  mb.Hreg(15, manualOverrideCode);

  // NEW: range + grid node
  mb.Hreg(16, (uint16_t)rangeLatched);     // 40017
  mb.Hreg(17, (uint16_t)gridNode);         // 40018
  // expose tunables
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);

  // Mirror to Discrete Inputs (1xxxx)
  updateDiscreteInputs();
}



// ---------------- Setup/loop ----------------
void setup(){
  LOG_BEGIN();
  Serial.println("[MTZ] Shifter v0.6");
  Serial.println("[MTZ] waiting 30s");
  delay(30000);
  Serial.println("[MTZ] boot");
  prefs.begin("shift", false);
  wifiLoadCreds();
  /* legacy loadCal removed */
  loadGrid();

  WireX.begin(SDA_X, SCL_X, 400000);
  WireY.begin(SDA_Y, SCL_Y, 400000);

  wifiJoinOrAP();

  http.on("/", pageIndex);
  http.on("/status", pageStatus);
  http.on("/log", pageLog);
  http.on("/net", HTTP_GET, pageNet);
  http.on("/net", HTTP_POST, pageNetPost);
  http.on("/manual", pageManual);
  http.on("/api/manual/enable", apiManualEnable);
  http.on("/api/manual/set",    apiManualSet);
  http.on("/grid", pageGrid);
  http.on("/api/grid/record", apiGridRecord);
  http.on("/api/grid/save",    apiGridSave);
  http.on("/api/grid/load",    apiGridLoad);
  http.on("/api/grid/clear",   apiGridClear);
  http.on("/grid/test",       pageGridTest);
  http.on("/api/grid/scores", apiGridScores);
  http.on("/grid/tune", HTTP_GET, pageGridTune);
  http.on("/grid/tune", HTTP_POST, apiGridTunePost);
  http.begin();

  mb.server(); logln("[Modbus] TCP server started on 502");
  for (int i=0;i<32;i++) mb.addHreg(i, 0);   // 40001..40032
  for (int i=0;i<64;i++) mb.addIsts(i, false); // 10001..10064
  // Initialize tunables into Modbus regs
  mb.Hreg(18, (uint16_t)GRID_STABLE_MS);
  mb.Hreg(19, (uint16_t)GEAR_DEBOUNCE_MS);
  mb.Hreg(20, (uint16_t)RANGE_DEBOUNCE_MS);
  mb.Hreg(21, (uint16_t)GRID_ENTER_TH);
  mb.Hreg(22, (uint16_t)GRID_EXIT_TH);
}

void loop(){
  // Process Modbus writes ASAP so override/grid commands apply this cycle
  mb.task();
  pollOverrideFromModbus();
  pollGridTunablesFromModbus();

  maintainWifi();
  http.handleClient();

  // I²C reads
  uint16_t a; uint8_t u8;
  if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
  if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
  if (i2cRead8(WireX, REG_AGC, u8)) agcX=u8; if (i2cRead16_12b(WireX, REG_MAG_H, REG_MAG_L, a)) magX=a;
  if (i2cRead8(WireY, REG_AGC, u8)) agcY=u8; if (i2cRead16_12b(WireY, REG_MAG_H, REG_MAG_L, a)) magY=a;

  // filter
  if (!fInit){ fx=rawX; fy=rawY; fInit=true; }
  filtX = lpf(fx, rawX); fx=filtX;
  filtY = lpf(fy, rawY); fy=filtY;
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
