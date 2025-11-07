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
