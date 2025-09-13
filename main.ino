/*
  MTZ Shift Detector — ESP32‑S2 (LOLIN S2 mini) v1

  Two AS5600 angle sensors (one per axis) -> classify tractor gear position
  Exposes Modbus TCP holding registers and a minimal Wi‑Fi HTTP UI for
  calibration, status, and network settings.

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
  Modbus TCP register map (all Holding, big‑endian 16‑bit)
  ───────────────────────────────────────────────────────────────────────────
  40001 RAW_X     u16  (0..4095)
  40002 RAW_Y     u16
  40003 FILT_X    u16
  40004 FILT_Y    u16
  40005 GEAR_CODE u16  0=N, 1..9, 101=R1, 102=R2, 65535=unknown
  40006 STATUS    u16  b0:calib b1:magOK_X b2:magOK_Y b3:selectorLane
                         b4:wifiUp b5:fault_X b6:fault_Y
  40007 AGC_X     u16  (0..255)
  40008 MAG_X     u16  (0..4095)
  40009 AGC_Y     u16
  40010 MAG_Y     u16
  40011 UPTIME_LO u16  seconds
  40012 UPTIME_HI u16
  40013 FW_VER    u16  0x0100 = v1.0.0
  40014 ERROR     u16  last error code (0=OK)

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

  ───────────────────────────────────────────────────────────────────────────
  HTTP UI
  ───────────────────────────────────────────────────────────────────────────
  • /           : index links
  • /status     : JSON live data
  • /log        : text ring‑buffer log of gear changes
  • /cal        : calibration page (record + save/load/clear)
      \-> /api/cal/record?slot=... (G1..G9,R1,R2,NMID,NSEL)
      \-> /api/cal/save | /api/cal/load | /api/cal/clear
  • /net        : set Wi‑Fi password for SSID "Tractor" and reboot

  Build: Arduino IDE (ESP32 board core), select LOLIN S2 mini (ESP32‑S2).
  Libraries: built‑in WiFi, WebServer; install "Modbus-ESP8266".
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ModbusIP_ESP8266.h>
#include <Preferences.h>
#include <Wire.h>

// --- Forward declarations to satisfy Arduino function auto-prototypes ---
#include <stdint.h>
enum Slot : uint8_t; // defined later
typedef struct { Slot s; uint32_t score; bool ok; } Best; // used in generated prototypes

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

// Wi‑Fi configuration
String wifiSsid = "Tractor";            // fixed SSID per requirements
String wifiPass = "";                   // configurable in /net
static constexpr char WIFI_AP_SSID_PREFIX[] = "TractorShift";
static constexpr char WIFI_AP_PASS[]        = "shift1234";
const IPAddress STATIC_IP(192,168,4,10);
const IPAddress STATIC_GATE(192,168,4,1);
const IPAddress STATIC_MASK(255,255,255,0);
const unsigned long DHCP_TIMEOUT_MS = 10000; // milliseconds to wait for DHCP

#include <stdarg.h>
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

// ---------------- Types & state ----------------
struct CalState { uint16_t x=2048, y=2048, rx=200, ry=200; bool valid=false; };

enum Slot : uint8_t {
  S_N_MID=0, S_N_SEL,
  S_R1, S_R2,
  S_G1, S_G2, S_G3, S_G4, S_G5, S_G6, S_G7, S_G8, S_G9,
  S_COUNT
};

static const char* SLOT_NAMES[S_COUNT] = {
  "NMID","NSEL","R1","R2",
  "G1","G2","G3","G4","G5","G6","G7","G8","G9"
};

static const uint16_t SLOT_CODES[S_COUNT] = {
  0, 0, 101, 102,
  1,2,3,4,5,6,7,8,9
};

struct CalibrationPack {
  uint32_t magic = 0xC011AB1E; // 'collable' ;)
  uint16_t version = 0x0100;
  CalState  st[S_COUNT];
};

CalibrationPack calib;  // live copy

// live measurements
uint16_t rawX=0, rawY=0, filtX=0, filtY=0;
uint16_t agcX=0, agcY=0, magX=0, magY=0;
uint16_t gearCode = 0;           // 0=N, 1..9, 101=R1, 102=R2, 65535=unknown
uint16_t statusBits = 0;         // see map
uint16_t errorCode = 0;          // last error
bool     inSelectorLane=false;   // runtime flag

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

// ---------------- Calibration store/load ----------------
void saveCal(){ prefs.putBytes("cal", &calib, sizeof(calib)); }

bool loadCal(){
  CalibrationPack tmp; size_t sz=prefs.getBytes("cal", &tmp, sizeof(tmp));
  if (sz==sizeof(tmp) && tmp.magic==0xC011AB1E) { calib=tmp; return true; }
  return false;
}

void clearCal(){
  CalibrationPack fresh; calib=fresh; saveCal();
}

// sample N readings (raw) and compute mean + robust spread
void sampleSlot(Slot s, uint16_t N=80){
  uint32_t sx=0, sy=0; uint16_t minX=65535, maxX=0, minY=65535, maxY=0;
  for(uint16_t i=0;i<N;i++){
    uint16_t a;
    if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
    if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
    sx += rawX; sy += rawY;
    if (rawX<minX) minX=rawX; if (rawX>maxX) maxX=rawX;
    if (rawY<minY) minY=rawY; if (rawY>maxY) maxY=rawY;
    delay(5); // ~200 Hz window
  }
  uint16_t mx = (uint16_t)(sx / N);
  uint16_t my = (uint16_t)(sy / N);
  uint16_t rx = (uint16_t)((maxX>minX ? (maxX-minX)/2 : 20));
  if (rx < 20) rx = 20;
  uint16_t ry = (uint16_t)((maxY>minY ? (maxY-minY)/2 : 20));
  if (ry < 20) ry = 20;
  calib.st[s].x=mx; calib.st[s].y=my; calib.st[s].rx=rx; calib.st[s].ry=ry; calib.st[s].valid=true;
}

// ---------------- Classification ----------------
// normalized distance scoring (elliptical, integer math)
static inline uint32_t scoreToSlot(uint8_t idx, uint16_t x, uint16_t y) {
  const auto &cs = calib.st[idx];
  int32_t dx = (int32_t)x - (int32_t)cs.x;
  int32_t dy = (int32_t)y - (int32_t)cs.y;
  uint32_t rx2 = (uint32_t)((cs.rx > 20 ? cs.rx : 20)); rx2 *= rx2;
  uint32_t ry2 = (uint32_t)((cs.ry > 20 ? cs.ry : 20)); ry2 *= ry2;
  return (uint32_t)((dx*dx) * 256UL / rx2 + (dy*dy) * 256UL / ry2);
}

Best bestSlot(uint16_t x, uint16_t y){
  Best b{S_N_MID, 0xFFFFFFFFu, false};
  for (int i=0;i<S_COUNT;i++){
    if (!calib.st[i].valid) continue;
    uint32_t sc = scoreToSlot((uint8_t)i, x, y);
    if (sc < b.score) { b = { (Slot)i, sc, true }; }
  }
  if (b.ok && b.score <= 9000) return b; // inside ellipse threshold
  b.ok=false; return b;
}

uint16_t classify(uint16_t x, uint16_t y){
  Best b = bestSlot(x,y);
  if (!b.ok) return 65535; // unknown
  inSelectorLane = (b.s == S_N_SEL);
  return SLOT_CODES[b.s];
}

// ---------------- Discrete Inputs mirror (GEAR_CODE) ---------------- (GEAR_CODE) ----------------
void updateDiscreteInputs(){
  // Clear used indices first
  const uint8_t used[] = {0,1,2,3,4,5,6,7,8,9,20,21};
  for (uint8_t i = 0; i < sizeof(used); ++i) {
    mb.Ists(used[i], false);
  }
  // Set according to current gearCode (one‑hot mirror)
  if (gearCode == 0) { mb.Ists(0, true); return; }
  if (gearCode >= 1 && gearCode <= 9) { mb.Ists((uint8_t)gearCode, true); return; }
  if (gearCode == 101) { mb.Ists(20, true); return; }
  if (gearCode == 102) { mb.Ists(21, true); return; }
  // Unknown → no DI asserted
}

// ---------------- Modbus writeout ----------------
void applyModbus(){
  mb.Hreg(0, rawX);
  mb.Hreg(1, rawY);
  mb.Hreg(2, filtX);
  mb.Hreg(3, filtY);
  mb.Hreg(4, gearCode);
  uint16_t st=0;
  if (calib.st[S_G1].valid || calib.st[S_R1].valid) st |= 1<<0; // calibrated
  if (magOK(magX)) st |= 1<<1; else st |= 1<<5; // fault_X
  if (magOK(magY)) st |= 1<<2; else st |= 1<<6; // fault_Y
  if (inSelectorLane) st |= 1<<3;
  if (wifiUp) st |= 1<<4;
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

  // Mirror to Discrete Inputs (1xxxx)
  updateDiscreteInputs();
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
  s += F("<h2>MTZ Shift Detector</h2><p><a href='/status'>/status</a> · <a href='/cal'>/cal</a> · <a href='/net'>/net</a> · <a href='/log'>/log</a></p>");
  s += F("<p>Modbus TCP on port <code>502</code>. Poll holding registers 40001…</p>");
  s += F("</body></html>"); http.send(200, "text/html", s);
}

void pageStatus(){
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\n\"rawX\":%u,\n\"rawY\":%u,\n\"filtX\":%u,\n\"filtY\":%u,\n\"gear\":%u,\n\"selector\":%s,\n\"wifiUp\":%s,\n\"agcX\":%u,\n\"agcY\":%u,\n\"magX\":%u,\n\"magY\":%u,\n\"statusBits\":%u\n}\n",
    rawX,rawY,filtX,filtY,gearCode,inSelectorLane?"true":"false",wifiUp?"true":"false",agcX,agcY,magX,magY,statusBits);
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
  s += F("<h3>Wi‑Fi</h3><p>Target SSID: <code>Tractor</code></p>");
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

String slotBtns(){
  String s;
  for (int i=0;i<S_COUNT;i++){
    s += "<button onclick=rec('"; s += SLOT_NAMES[i]; s += "')>Record "; s += SLOT_NAMES[i]; s += "</button>";
    if (calib.st[i].valid){ s += " <small>(x=" + String(calib.st[i].x) + ", y=" + String(calib.st[i].y) + ")</small>"; }
    s += "<br>";
  }
  return s;
}

void pageCal(){
  String s = htmlHeader("Calibration");
  s += F("<h3>Calibration</h3><p>Hold the lever in the requested position fully engaged, then press the corresponding <b>Record</b> button.</p>");
  s += F("<p>Neutrals:<br>• <b>NMID</b> — center neutral<br>• <b>NSEL</b> — neutral inside the left selector lane</p>");
  s += F("<div id='out' style='padding:8px;background:#f4f4f4;border-radius:8px'></div>");
  s += slotBtns();
  s += F("<p><button onclick=act('save')>Save</button> <button onclick=act('load')>Load</button> <button onclick=act('clear')>Clear</button></p>");
  s += F("<script>\nfunction rec(name){fetch('/api/cal/record?slot='+name).then(r=>r.text()).then(t=>{document.getElementById('out').innerText=t; location.reload();});}\nfunction act(a){fetch('/api/cal/'+a).then(r=>r.text()).then(t=>document.getElementById('out').innerText=t);}\n</script>");
  s += F("</body></html>");
  http.send(200, "text/html", s);
}

Slot parseSlot(const String& n){
  for (int i=0;i<S_COUNT;i++) if (n.equalsIgnoreCase(SLOT_NAMES[i])) return (Slot)i;
  return S_N_MID;
}

void apiCalRecord(){
  if (!http.hasArg("slot")) { http.send(400, "text/plain", "slot?="); return; }
  Slot s = parseSlot(http.arg("slot"));
  sampleSlot(s);
  char buf[128]; snprintf(buf,sizeof(buf),"OK %s: x=%u y=%u rx=%u ry=%u\n", SLOT_NAMES[s], calib.st[s].x, calib.st[s].y, calib.st[s].rx, calib.st[s].ry);
  http.send(200, "text/plain", buf);
}

void apiCalSave(){ saveCal(); http.send(200, "text/plain", "Saved\n"); }
void apiCalLoad(){
  bool ok = loadCal();
  logf("[CAL] %s", ok?"loaded":"empty");
  http.send(200, "text/plain", ok ? "Loaded\n" : "No data\n");
}

void apiCalClear(){ clearCal(); http.send(200, "text/plain", "Cleared\n"); }

// ---------------- Wi‑Fi join / fallback AP ----------------
void wifiJoinOrAP(){
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Attempt DHCP
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  unsigned long t0=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<DHCP_TIMEOUT_MS) delay(50);
  wifiUp = (WiFi.status()==WL_CONNECTED);

  // If DHCP failed, try static IP configuration
  if (!wifiUp){
    WiFi.config(STATIC_IP, STATIC_GATE, STATIC_MASK);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    t0=millis();
    while (WiFi.status()!=WL_CONNECTED && millis()-t0<DHCP_TIMEOUT_MS) delay(50);
    wifiUp = (WiFi.status()==WL_CONNECTED);
  }

  // If still not connected, fall back to Access Point mode
  if (!wifiUp){
    WiFi.mode(WIFI_AP);
    char ap[32]; uint64_t mac = ESP.getEfuseMac();
    snprintf(ap,sizeof(ap),"%s-%02X%02X", WIFI_AP_SSID_PREFIX,
             (unsigned)((mac>>8)&0xFF), (unsigned)(mac&0xFF));
    WiFi.softAP(ap, WIFI_AP_PASS);
    wifiUp=false;
  }
  logWifiStatus();
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

// Ensure Wi‑Fi connectivity after setup. If the station disconnects, try to
// reconnect or fall back to access point mode again. Updates wifiUp flag so
// status bits and diagnostics reflect the current state accurately.
void maintainWifi(){
  bool prev = wifiUp;
  if (WiFi.getMode() == WIFI_MODE_STA){
    wifiUp = (WiFi.status() == WL_CONNECTED);
    if (!wifiUp){
      wifiJoinOrAP();
      return; // wifiJoinOrAP logs status
    }
  } else {
    wifiUp = false;
  }
  if (wifiUp != prev) logWifiStatus();
}

// ---------------- Setup/loop ----------------
void setup(){
  LOG_BEGIN();
  Serial.println("[MTZ] boot");
  prefs.begin("shift", false);
  wifiPass = prefs.getString("pass", "");
  loadCal();

  WireX.begin(SDA_X, SCL_X, 400000);
  WireY.begin(SDA_Y, SCL_Y, 400000);

  wifiJoinOrAP();
  

  http.on("/", pageIndex);
  http.on("/status", pageStatus);
  http.on("/log", pageLog);
  http.on("/net", HTTP_GET, pageNet);
  http.on("/net", HTTP_POST, pageNetPost);
  http.on("/cal", pageCal);
  http.on("/api/cal/record", apiCalRecord);
  http.on("/api/cal/save", apiCalSave);
  http.on("/api/cal/load", apiCalLoad);
  http.on("/api/cal/clear", apiCalClear);
  http.begin();

  mb.server(); logln("[Modbus] TCP server started on 502");
  // Create Holding Registers 0..31 (maps to 40001..40032)
  for (int i=0;i<32;i++) mb.addHreg(i, 0);
  // Create Discrete Inputs 0..63 (maps to 10001..10064)
  for (int i=0;i<64;i++) mb.addIsts(i, false);
}

void loop(){
  maintainWifi();
  http.handleClient();

  // I²C reads
  uint16_t a;
  if (i2cRead16_12b(WireX, REG_ANG_H, REG_ANG_L, a)) rawX=a; else errorCode=1;
  if (i2cRead16_12b(WireY, REG_ANG_H, REG_ANG_L, a)) rawY=a; else errorCode=1;
  uint8_t u8;
  if (i2cRead8(WireX, REG_AGC, u8)) agcX=u8; if (i2cRead16_12b(WireX, REG_MAG_H, REG_MAG_L, a)) magX=a;
  if (i2cRead8(WireY, REG_AGC, u8)) agcY=u8; if (i2cRead16_12b(WireY, REG_MAG_H, REG_MAG_L, a)) magY=a;

  // filter
  if (!fInit){ fx=rawX; fy=rawY; fInit=true; }
  filtX = lpf(fx, rawX); fx=filtX;
  filtY = lpf(fy, rawY); fy=filtY;

  // classify
  uint16_t newCode = classify(filtX, filtY);
  if (newCode != gearCode){ gearCode=newCode; logGear(gearCode); logf("[GEAR] code=%u", gearCode); }

  // status → Modbus
  applyModbus();

  // diagnostics edge logs
  bool mX = magOK(magX), mY = magOK(magY);
  if (mX != prevMagOkX || mY != prevMagOkY){ logf("[MAG] X:%u(%s) Y:%u(%s)", magX, mX?"OK":"BAD", magY, mY?"OK":"BAD"); prevMagOkX=mX; prevMagOkY=mY; }
  if (errorCode != prevErrorLogged){ logf("[ERR] code=%u", errorCode); prevErrorLogged=errorCode; }

  // periodic status @1Hz
  if (millis()-lastPrintMs >= 1000){
    int rssi = (WiFi.getMode()==WIFI_MODE_STA && WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : 0;
    logf("[STAT] rawX=%u rawY=%u filtX=%u filtY=%u gear=%u selector=%u AGC=%u/%u MAG=%u/%u wifiUp=%u RSSI=%d",
          rawX,rawY,filtX,filtY,gearCode,(unsigned)inSelectorLane,agcX,agcY,magX,magY,(unsigned)wifiUp,rssi);
    lastPrintMs = millis();
  }

  // Modbus TCP task
  mb.task();

  delay(20); // ~50 Hz internal loop (external Modbus 1..20 Hz OK)
}
