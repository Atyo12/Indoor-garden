/*
  Controllino MAXI + Nextion + DS3231 + EEPROM Indoor Garden
  V2_6_AUTO_FIX_260526

  Alap:
  - kert_V2_2_240526_manual verzióra építve
  - MANUAL mód megtartva
  - AUTO MVP hozzáadva

  AUTO MVP működés:
  - Z1/Z2 engedély alapján automata ciklus indul
  - Feltöltés: kiválasztott IBC + zóna + feltöltő pumpa
  - Feltöltést a Z1/Z2 FULL bemenet állítja le
  - Áztatás idő alapján
  - Leeresztés idő alapján, mert jelenleg még nincs külön "üres" érzékelő
  - Pihenő idő után új ciklus
  - Egyszerre csak egy zóna végezhet közös műveletet: FELTÖLTÉS vagy LEERESZTÉS
  - Tiltott időszakban AUTO nem öntöz, minden automata kimenet KI
  - Külső DS18B20 hőmérséklet később kerül be

  Nextion Touch Release Event parancsok:
  SETTINGS oldal:
    bApply:    printh A5 01 FF FF FF
    bSave:     printh A5 02 FF FF FF
    bDefault:  printh A5 03 FF FF FF
    bSaveSkip: printh A5 04 FF FF FF
    bSetRtc:   printh A5 05 FF FF FF

  RTC oldal események:
    oldal megnyitás:  printh A5 14 FF FF FF
    oldal elhagyás:  printh A5 15 FF FF FF

  ÜZEMMÓD / GLOBAL:
    bManual:  printh A5 10 FF FF FF
    bAuto:    printh A5 11 FF FF FF
    bAllOff:  printh A5 12 FF FF FF
    btR:      printh A5 13 FF FF FF

  MANUAL FELTÖLTÉS gombok:
    btFillIbc1Sel:  printh A5 20 FF FF FF
    btFillIbc1Open: printh A5 21 FF FF FF
    btFillIbc2Sel:  printh A5 22 FF FF FF
    btFillIbc2Open: printh A5 23 FF FF FF
    btFillIbc3Sel:  printh A5 24 FF FF FF
    btFillIbc3Open: printh A5 25 FF FF FF
    btFillZ1Sel:    printh A5 26 FF FF FF
    btFillZ2Sel:    printh A5 27 FF FF FF

  MANUAL LEERESZTÉS gombok:
    btDrainZ1Open:  printh A5 28 FF FF FF
    btDrainZ2Open:  printh A5 29 FF FF FF
    btDI1:          printh A5 2A FF FF FF
    btDI2:          printh A5 2B FF FF FF
    btDI3:          printh A5 2C FF FF FF
    btDP:           printh A5 2D FF FF FF
    btDS:           printh A5 2E FF FF FF
*/

#include <Controllino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RTClib.h"

#define nex Serial2

#define EEPROM_ADDR 0
#define SETTINGS_MAGIC 0x4752544EUL
#define SETTINGS_VERSION 5

RTC_DS3231 rtc;
bool g_rtcOk = false;
bool g_rtcEditFieldsLoaded = false;

// ============================================================
// IO KIOSZTÁS
// ============================================================

const uint8_t MANUAL_OUT_COUNT = 15;

const uint8_t PIN_FILL_IBC1_SEL  = CONTROLLINO_R0; // 01 - FELTÖLTÉS: IBC1 választ
const uint8_t PIN_FILL_IBC1_OPEN = CONTROLLINO_R1; // 02 - FELTÖLTÉS: IBC1 nyit
const uint8_t PIN_FILL_IBC2_SEL  = CONTROLLINO_R2; // 03 - FELTÖLTÉS: IBC2 választ
const uint8_t PIN_FILL_IBC2_OPEN = CONTROLLINO_R3; // 04 - FELTÖLTÉS: IBC2 nyit
const uint8_t PIN_FILL_IBC3_SEL  = CONTROLLINO_R4; // 05 - FELTÖLTÉS: IBC3 választ / tiszta víz
const uint8_t PIN_FILL_IBC3_OPEN = CONTROLLINO_R5; // 06 - FELTÖLTÉS: IBC3 nyit / tiszta víz
const uint8_t PIN_FILL_Z1_SEL    = CONTROLLINO_R6; // 07 - FELTÖLTÉS: Z1 választ
const uint8_t PIN_FILL_Z2_SEL    = CONTROLLINO_R7; // 08 - FELTÖLTÉS: Z2 választ

const uint8_t PIN_DRAIN_Z1_OPEN  = CONTROLLINO_R8; // 09 - LEERESZTÉS: Z1 nyit
const uint8_t PIN_DRAIN_Z2_OPEN  = CONTROLLINO_R9; // 10 - LEERESZTÉS: Z2 nyit
const uint8_t PIN_DRAIN_IBC1_OPEN = CONTROLLINO_D0; // 11 - LEERESZTÉS: IBC1 nyit
const uint8_t PIN_DRAIN_IBC2_OPEN = CONTROLLINO_D1; // 12 - LEERESZTÉS: IBC2 nyit
const uint8_t PIN_DRAIN_IBC3_OPEN = CONTROLLINO_D2; // 13 - LEERESZTÉS: IBC3 nyit
const uint8_t PIN_FILL_PUMP      = CONTROLLINO_D3; // 14 - Feltöltő pumpa
const uint8_t PIN_DRAIN_PUMP     = CONTROLLINO_D4; // 15 - Leeresztő szivattyú

const uint8_t MANUAL_OUT_PINS[MANUAL_OUT_COUNT] = {
  PIN_FILL_IBC1_SEL,
  PIN_FILL_IBC1_OPEN,
  PIN_FILL_IBC2_SEL,
  PIN_FILL_IBC2_OPEN,
  PIN_FILL_IBC3_SEL,
  PIN_FILL_IBC3_OPEN,
  PIN_FILL_Z1_SEL,
  PIN_FILL_Z2_SEL,
  PIN_DRAIN_Z1_OPEN,
  PIN_DRAIN_Z2_OPEN,
  PIN_DRAIN_IBC1_OPEN,
  PIN_DRAIN_IBC2_OPEN,
  PIN_DRAIN_IBC3_OPEN,
  PIN_FILL_PUMP,
  PIN_DRAIN_PUMP
};

const char* MANUAL_OUT_NAMES[MANUAL_OUT_COUNT] = {
  "FILL_IBC1_SEL",
  "FILL_IBC1_OPEN",
  "FILL_IBC2_SEL",
  "FILL_IBC2_OPEN",
  "FILL_IBC3_SEL",
  "FILL_IBC3_OPEN",
  "FILL_Z1_SEL",
  "FILL_Z2_SEL",
  "DRAIN_Z1_OPEN",
  "DRAIN_Z2_OPEN",
  "DRAIN_IBC1_OPEN",
  "DRAIN_IBC2_OPEN",
  "DRAIN_IBC3_OPEN",
  "FILL_PUMP",
  "DRAIN_PUMP"
};

const char* MANUAL_HMI_BTNS[MANUAL_OUT_COUNT] = {
  "btFillIbc1Sel",
  "btFillIbc1Open",
  "btFillIbc2Sel",
  "btFillIbc2Open",
  "btFillIbc3Sel",
  "btFillIbc3Open",
  "btFillZ1Sel",
  "btFillZ2Sel",
  "btDrainZ1Open",
  "btDrainZ2Open",
  "btDI1",
  "btDI2",
  "btDI3",
  "btDP",
  "btDS"
};

// Jelenlegi bemenetek
const uint8_t PIN_DI_Z1_FULL = CONTROLLINO_DI0;
const uint8_t PIN_DI_Z2_FULL = CONTROLLINO_DI1;

// Ha a relémodulod aktív LOW bemenetű, írd true-ra.
const bool RELAY_ACTIVE_LOW = false;

// Ha a szintérzékelő aktív LOW, írd false-ra.
const bool LEVEL_FULL_ACTIVE_HIGH = true;

// ============================================================
// ADATSZERKEZETEK
// ============================================================

struct ZoneSettings {
  bool enabled;
  uint8_t tank; // 1 vagy 2 tápoldatos IBC
  uint16_t fillTimeoutMin;
  uint16_t soakMin;
  uint16_t drainTimeoutMin;
  uint16_t fullLostMin;
  uint16_t pauseMin;
};

struct Settings {
  uint32_t magic;
  uint16_t version;

  ZoneSettings z1;
  ZoneSettings z2;

  uint8_t cleanTank;   // jelenleg 3
  int16_t hotTempC;    // külső hőmérséklet küszöb későbbre

  bool skipEnabled;
  uint8_t skipStartH;
  uint8_t skipStartM;
  uint8_t skipEndH;
  uint8_t skipEndM;

  uint16_t crc;
};

Settings g_settings;

// ============================================================
// ÜZEMMÓD / MANUAL / AUTO ÁLLAPOTOK
// ============================================================

enum WorkMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO = 1
};

WorkMode g_mode = MODE_MANUAL;

struct ManualCmd {
  bool out[MANUAL_OUT_COUNT];
};

ManualCmd g_manual;

enum AutoStep : uint8_t {
  AUTO_IDLE = 0,
  AUTO_FILL,
  AUTO_SOAK,
  AUTO_DRAIN,
  AUTO_PAUSE
};

struct AutoZone {
  AutoStep step;
  unsigned long stepStartMs;
  bool active;
};

AutoZone g_autoZ1 = { AUTO_IDLE, 0, false };
AutoZone g_autoZ2 = { AUTO_IDLE, 0, false };

bool g_autoOut[MANUAL_OUT_COUNT];

bool g_fault = false;
char g_faultText[32] = "NINCS HIBA";

unsigned long g_lastHmiUpdateMs = 0;

// ============================================================
// ELŐDEKLARÁCIÓK
// ============================================================

void manualAllOff();
void autoAllOff();
void applyOutputs();
void writeManualButtonsToHMI();
void writeMainStatusToHMI();
void resetAutoZone(AutoZone &az);
void runAutoMode();
void setFault(const char* txt);
void clearFault();

// ============================================================
// Nextion alapfüggvények
// ============================================================

void nexEnd() {
  nex.write(0xFF);
  nex.write(0xFF);
  nex.write(0xFF);
}

void nexCmd(const char* cmd) {
  nex.print(cmd);
  nexEnd();
}

void nexSetText(const char* obj, const char* txt) {
  nex.print(obj);
  nex.print(".txt=\"");
  nex.print(txt);
  nex.print("\"");
  nexEnd();
}

void nexSetValue(const char* obj, int value) {
  nex.print(obj);
  nex.print(".val=");
  nex.print(value);
  nexEnd();
}

bool nexReadNumber(const char* obj, uint32_t &value) {
  while (nex.available()) nex.read();

  nex.print("get ");
  nex.print(obj);
  nex.print(".val");
  nexEnd();

  unsigned long start = millis();
  uint8_t buf[8];
  uint8_t idx = 0;

  while (millis() - start < 300) {
    if (nex.available()) {
      uint8_t b = nex.read();
      if (idx < sizeof(buf)) buf[idx++] = b;
      if (idx >= 8) break;
    }
  }

  if (idx >= 8 && buf[0] == 0x71 && buf[5] == 0xFF && buf[6] == 0xFF && buf[7] == 0xFF) {
    value = (uint32_t)buf[1]
          | ((uint32_t)buf[2] << 8)
          | ((uint32_t)buf[3] << 16)
          | ((uint32_t)buf[4] << 24);
    return true;
  }

  return false;
}

bool readU16(const char* obj, uint16_t &dst) {
  uint32_t v;
  if (!nexReadNumber(obj, v)) return false;
  dst = (uint16_t)v;
  return true;
}

bool readU8(const char* obj, uint8_t &dst) {
  uint32_t v;
  if (!nexReadNumber(obj, v)) return false;
  dst = (uint8_t)v;
  return true;
}

bool readBool(const char* obj, bool &dst) {
  uint32_t v;
  if (!nexReadNumber(obj, v)) return false;
  dst = (v != 0);
  return true;
}

// ============================================================
// CRC / EEPROM
// ============================================================

uint16_t calcCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }

  return crc;
}

uint16_t settingsCrc(const Settings &s) {
  Settings tmp = s;
  tmp.crc = 0;
  return calcCrc16((const uint8_t*)&tmp, sizeof(Settings));
}

void loadDefaultSettings() {
  g_settings.magic = SETTINGS_MAGIC;
  g_settings.version = SETTINGS_VERSION;

  g_settings.z1.enabled = true;
  g_settings.z1.tank = 1;
  g_settings.z1.fillTimeoutMin = 15;
  g_settings.z1.soakMin = 20;
  g_settings.z1.drainTimeoutMin = 10;
  g_settings.z1.fullLostMin = 5;
  g_settings.z1.pauseMin = 60;

  g_settings.z2.enabled = true;
  g_settings.z2.tank = 2;
  g_settings.z2.fillTimeoutMin = 12;
  g_settings.z2.soakMin = 25;
  g_settings.z2.drainTimeoutMin = 9;
  g_settings.z2.fullLostMin = 4;
  g_settings.z2.pauseMin = 45;

  g_settings.cleanTank = 3;
  g_settings.hotTempC = 28;

  // Első indításkor is legyen értelmes tiltott idő.
  // A régi EEPROM-adatok a SETTINGS_VERSION miatt automatikusan eldobódnak.
  g_settings.skipEnabled = true;
  g_settings.skipStartH = 22;
  g_settings.skipStartM = 0;
  g_settings.skipEndH = 6;
  g_settings.skipEndM = 0;

  g_settings.crc = settingsCrc(g_settings);
}

void clampSettings() {
  if (g_settings.z1.tank < 1) g_settings.z1.tank = 1;
  if (g_settings.z1.tank > 2) g_settings.z1.tank = 2;

  if (g_settings.z2.tank < 1) g_settings.z2.tank = 1;
  if (g_settings.z2.tank > 2) g_settings.z2.tank = 2;

  if (g_settings.cleanTank < 1) g_settings.cleanTank = 1;
  if (g_settings.cleanTank > 3) g_settings.cleanTank = 3;

  if (g_settings.hotTempC < 10) g_settings.hotTempC = 10;
  if (g_settings.hotTempC > 50) g_settings.hotTempC = 50;

  if (g_settings.z1.fillTimeoutMin < 1) g_settings.z1.fillTimeoutMin = 1;
  if (g_settings.z1.soakMin < 1) g_settings.z1.soakMin = 1;
  if (g_settings.z1.drainTimeoutMin < 1) g_settings.z1.drainTimeoutMin = 1;
  if (g_settings.z1.fullLostMin < 1) g_settings.z1.fullLostMin = 1;
  if (g_settings.z1.pauseMin < 1) g_settings.z1.pauseMin = 1;

  if (g_settings.z1.fillTimeoutMin > 1440) g_settings.z1.fillTimeoutMin = 1440;
  if (g_settings.z1.soakMin > 1440) g_settings.z1.soakMin = 1440;
  if (g_settings.z1.drainTimeoutMin > 1440) g_settings.z1.drainTimeoutMin = 1440;
  if (g_settings.z1.fullLostMin > 1440) g_settings.z1.fullLostMin = 1440;
  if (g_settings.z1.pauseMin > 1440) g_settings.z1.pauseMin = 1440;

  if (g_settings.z2.fillTimeoutMin < 1) g_settings.z2.fillTimeoutMin = 1;
  if (g_settings.z2.soakMin < 1) g_settings.z2.soakMin = 1;
  if (g_settings.z2.drainTimeoutMin < 1) g_settings.z2.drainTimeoutMin = 1;
  if (g_settings.z2.fullLostMin < 1) g_settings.z2.fullLostMin = 1;
  if (g_settings.z2.pauseMin < 1) g_settings.z2.pauseMin = 1;

  if (g_settings.z2.fillTimeoutMin > 1440) g_settings.z2.fillTimeoutMin = 1440;
  if (g_settings.z2.soakMin > 1440) g_settings.z2.soakMin = 1440;
  if (g_settings.z2.drainTimeoutMin > 1440) g_settings.z2.drainTimeoutMin = 1440;
  if (g_settings.z2.fullLostMin > 1440) g_settings.z2.fullLostMin = 1440;
  if (g_settings.z2.pauseMin > 1440) g_settings.z2.pauseMin = 1440;

  if (g_settings.skipStartH > 23) g_settings.skipStartH = 23;
  if (g_settings.skipEndH > 23) g_settings.skipEndH = 23;
  if (g_settings.skipStartM > 59) g_settings.skipStartM = 59;
  if (g_settings.skipEndM > 59) g_settings.skipEndM = 59;
}

bool settingsLooksValid() {
  if (g_settings.magic != SETTINGS_MAGIC) return false;
  if (g_settings.version != SETTINGS_VERSION) return false;

  if (g_settings.z1.tank < 1 || g_settings.z1.tank > 2) return false;
  if (g_settings.z2.tank < 1 || g_settings.z2.tank > 2) return false;
  if (g_settings.cleanTank < 1 || g_settings.cleanTank > 3) return false;

  if (g_settings.hotTempC < 10 || g_settings.hotTempC > 50) return false;

  if (g_settings.z1.fillTimeoutMin < 1 || g_settings.z1.fillTimeoutMin > 1440) return false;
  if (g_settings.z1.soakMin < 1 || g_settings.z1.soakMin > 1440) return false;
  if (g_settings.z1.drainTimeoutMin < 1 || g_settings.z1.drainTimeoutMin > 1440) return false;
  if (g_settings.z1.fullLostMin < 1 || g_settings.z1.fullLostMin > 1440) return false;
  if (g_settings.z1.pauseMin < 1 || g_settings.z1.pauseMin > 1440) return false;

  if (g_settings.z2.fillTimeoutMin < 1 || g_settings.z2.fillTimeoutMin > 1440) return false;
  if (g_settings.z2.soakMin < 1 || g_settings.z2.soakMin > 1440) return false;
  if (g_settings.z2.drainTimeoutMin < 1 || g_settings.z2.drainTimeoutMin > 1440) return false;
  if (g_settings.z2.fullLostMin < 1 || g_settings.z2.fullLostMin > 1440) return false;
  if (g_settings.z2.pauseMin < 1 || g_settings.z2.pauseMin > 1440) return false;

  if (g_settings.skipStartH > 23 || g_settings.skipEndH > 23) return false;
  if (g_settings.skipStartM > 59 || g_settings.skipEndM > 59) return false;

  return true;
}

void saveSettingsToEEPROM() {
  g_settings.magic = SETTINGS_MAGIC;
  g_settings.version = SETTINGS_VERSION;
  clampSettings();
  g_settings.crc = settingsCrc(g_settings);
  EEPROM.put(EEPROM_ADDR, g_settings);
  Serial.println("EEPROM mentes kesz.");
}

bool loadSettingsFromEEPROM() {
  Settings tmp;
  EEPROM.get(EEPROM_ADDR, tmp);

  if (tmp.magic != SETTINGS_MAGIC) {
    Serial.println("EEPROM magic hibas.");
    return false;
  }

  if (tmp.version != SETTINGS_VERSION) {
    Serial.println("EEPROM verzio hibas.");
    return false;
  }

  uint16_t crcStored = tmp.crc;
  tmp.crc = 0;
  uint16_t crcCalc = calcCrc16((const uint8_t*)&tmp, sizeof(Settings));

  if (crcStored != crcCalc) {
    Serial.println("EEPROM CRC hibas.");
    return false;
  }

  tmp.crc = crcStored;
  g_settings = tmp;

  if (!settingsLooksValid()) {
    Serial.println("EEPROM tartalom ervenytelen / ertelmetlen adat.");
    return false;
  }

  clampSettings();

  Serial.println("EEPROM betoltes OK.");
  return true;
}

// ============================================================
// HMI settings oldal
// ============================================================

bool readSettingsFromHMI() {
  bool ok = true;

  ok &= readBool("btZ1Enable", g_settings.z1.enabled);
  ok &= readU8("nZ1Tank", g_settings.z1.tank);
  ok &= readU16("nZ1FillTO", g_settings.z1.fillTimeoutMin);
  ok &= readU16("nZ1SoakMin", g_settings.z1.soakMin);
  ok &= readU16("nZ1DrainMin", g_settings.z1.drainTimeoutMin);
  ok &= readU16("nZ1FullLostMin", g_settings.z1.fullLostMin);
  ok &= readU16("nZ1PauseMin", g_settings.z1.pauseMin);

  ok &= readBool("btZ2Enable", g_settings.z2.enabled);
  ok &= readU8("nZ2Tank", g_settings.z2.tank);
  ok &= readU16("nZ2FillTO", g_settings.z2.fillTimeoutMin);
  ok &= readU16("nZ2SoakMin", g_settings.z2.soakMin);
  ok &= readU16("nZ2DrainMin", g_settings.z2.drainTimeoutMin);
  ok &= readU16("nZ2FullLostMin", g_settings.z2.fullLostMin);
  ok &= readU16("nZ2PauseMin", g_settings.z2.pauseMin);

  ok &= readU8("nCleanTank", g_settings.cleanTank);

  uint32_t hot;
  if (nexReadNumber("nHotTemp", hot)) {
    g_settings.hotTempC = (int16_t)hot;
  } else {
    ok = false;
  }

  if (ok) {
    g_settings.magic = SETTINGS_MAGIC;
    g_settings.version = SETTINGS_VERSION;
    clampSettings();
    g_settings.crc = settingsCrc(g_settings);
  }

  return ok;
}

void writeSettingsToHMI() {
  nexCmd("page settings");
  delay(300);

  nexSetValue("btZ1Enable", g_settings.z1.enabled ? 1 : 0);
  nexSetText("btZ1Enable", g_settings.z1.enabled ? "IGEN" : "NEM");
  nexSetValue("nZ1Tank", g_settings.z1.tank);
  nexSetValue("nZ1FillTO", g_settings.z1.fillTimeoutMin);
  nexSetValue("nZ1SoakMin", g_settings.z1.soakMin);
  nexSetValue("nZ1DrainMin", g_settings.z1.drainTimeoutMin);
  nexSetValue("nZ1FullLostMin", g_settings.z1.fullLostMin);
  nexSetValue("nZ1PauseMin", g_settings.z1.pauseMin);

  nexSetValue("btZ2Enable", g_settings.z2.enabled ? 1 : 0);
  nexSetText("btZ2Enable", g_settings.z2.enabled ? "IGEN" : "NEM");
  nexSetValue("nZ2Tank", g_settings.z2.tank);
  nexSetValue("nZ2FillTO", g_settings.z2.fillTimeoutMin);
  nexSetValue("nZ2SoakMin", g_settings.z2.soakMin);
  nexSetValue("nZ2DrainMin", g_settings.z2.drainTimeoutMin);
  nexSetValue("nZ2FullLostMin", g_settings.z2.fullLostMin);
  nexSetValue("nZ2PauseMin", g_settings.z2.pauseMin);

  nexSetValue("nCleanTank", g_settings.cleanTank);
  nexSetValue("nHotTemp", g_settings.hotTempC);
}

// ============================================================
// HMI tiltott idő oldal
// ============================================================

bool readSkipSettingsFromHMI() {
  bool ok = true;

  ok &= readBool("btSkipEnable", g_settings.skipEnabled);
  ok &= readU8("nSkipStartH", g_settings.skipStartH);
  ok &= readU8("nSkipStartM", g_settings.skipStartM);
  ok &= readU8("nSkipEndH", g_settings.skipEndH);
  ok &= readU8("nSkipEndM", g_settings.skipEndM);

  if (ok) {
    g_settings.magic = SETTINGS_MAGIC;
    g_settings.version = SETTINGS_VERSION;
    clampSettings();
    g_settings.crc = settingsCrc(g_settings);
  }

  return ok;
}

void writeSkipSettingsToHMI() {
  nexCmd("page rtc");
  delay(300);

  nexSetValue("btSkipEnable", g_settings.skipEnabled ? 1 : 0);
  nexSetValue("nSkipStartH", g_settings.skipStartH);
  nexSetValue("nSkipStartM", g_settings.skipStartM);
  nexSetValue("nSkipEndH", g_settings.skipEndH);
  nexSetValue("nSkipEndM", g_settings.skipEndM);
}

// ============================================================
// RTC
// ============================================================

void writeRtcToHMI() {
  if (!g_rtcOk) return;

  DateTime now = rtc.now();

  char timeBuf[12];
  char dateBuf[16];

  sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  sprintf(dateBuf, "%04d.%02d.%02d", now.year(), now.month(), now.day());

  nexSetText("tRtcTime", timeBuf);
  nexSetText("tRtcDate", dateBuf);
}

bool readRtcFromHMIAndSet() {
  if (!g_rtcOk) return false;

  uint16_t year;
  uint8_t month, day, hour, minute, second;

  bool ok = true;
  ok &= readU16("nYear", year);
  ok &= readU8("nMonth", month);
  ok &= readU8("nDay", day);
  ok &= readU8("nHour", hour);
  ok &= readU8("nMinute", minute);
  ok &= readU8("nSecond", second);

  if (!ok) return false;

  if (year < 2024 || year > 2099) return false;
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour > 23) return false;
  if (minute > 59) return false;
  if (second > 59) return false;

  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  return true;
}

bool isInSkipTime() {
  if (!g_rtcOk) return false;
  if (!g_settings.skipEnabled) return false;

  DateTime now = rtc.now();

  uint16_t nowMin = now.hour() * 60 + now.minute();
  uint16_t startMin = g_settings.skipStartH * 60 + g_settings.skipStartM;
  uint16_t endMin = g_settings.skipEndH * 60 + g_settings.skipEndM;

  if (startMin == endMin) return false;

  if (startMin < endMin) {
    return nowMin >= startMin && nowMin < endMin;
  }

  return nowMin >= startMin || nowMin < endMin;
}

void writeRtcEditFieldsToHMI() {
  if (!g_rtcOk) return;

  DateTime now = rtc.now();

  nexSetValue("nYear", now.year());
  nexSetValue("nMonth", now.month());
  nexSetValue("nDay", now.day());
  nexSetValue("nHour", now.hour());
  nexSetValue("nMinute", now.minute());
  nexSetValue("nSecond", now.second());

  nexSetValue("btSkipEnable", g_settings.skipEnabled ? 1 : 0);
  nexSetValue("nSkipStartH", g_settings.skipStartH);
  nexSetValue("nSkipStartM", g_settings.skipStartM);
  nexSetValue("nSkipEndH", g_settings.skipEndH);
  nexSetValue("nSkipEndM", g_settings.skipEndM);
}

// ============================================================
// MANUAL LOGIKA
// ============================================================

void manualAllOff() {
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    g_manual.out[i] = false;
  }
}

void setFault(const char* txt) {
  g_fault = true;

  strncpy(g_faultText, txt, sizeof(g_faultText) - 1);
  g_faultText[sizeof(g_faultText) - 1] = '\0';

  manualAllOff();
  autoAllOff();
}

void clearFault() {
  g_fault = false;

  strncpy(g_faultText, "NINCS HIBA", sizeof(g_faultText) - 1);
  g_faultText[sizeof(g_faultText) - 1] = '\0';
}

void toggleManualOutput(uint8_t idx) {
  if (idx >= MANUAL_OUT_COUNT) return;
  if (g_mode != MODE_MANUAL) return;
  if (g_fault) return;

  g_manual.out[idx] = !g_manual.out[idx];
}

void enforceManualSafety() {
  if (g_mode != MODE_MANUAL) return;

  // MANUAL szerviz mód:
  // Itt szándékosan nem kapcsolgatunk le automatikusan más kimeneteket,
  // mert szervizeléskor te döntöd el, milyen útvonalat akarsz próbálni.
}

// ============================================================
// AUTO LOGIKA
// ============================================================

unsigned long minToMs(uint16_t minVal) {
  return (unsigned long)minVal * 60UL * 1000UL;
}

void autoAllOff() {
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    g_autoOut[i] = false;
  }
}

bool zoneFull(uint8_t zone) {
  bool raw = false;

  if (zone == 1) raw = (digitalRead(PIN_DI_Z1_FULL) == HIGH);
  else if (zone == 2) raw = (digitalRead(PIN_DI_Z2_FULL) == HIGH);
  else return false;

  return LEVEL_FULL_ACTIVE_HIGH ? raw : !raw;
}

ZoneSettings* getZoneSettings(uint8_t zone) {
  if (zone == 1) return &g_settings.z1;
  if (zone == 2) return &g_settings.z2;
  return NULL;
}

AutoZone* getAutoZone(uint8_t zone) {
  if (zone == 1) return &g_autoZ1;
  if (zone == 2) return &g_autoZ2;
  return NULL;
}

const char* autoStepName(AutoStep step) {
  switch (step) {
    case AUTO_IDLE:  return "IDLE";
    case AUTO_FILL:  return "FELTOLTES";
    case AUTO_SOAK:  return "AZTATAS";
    case AUTO_DRAIN: return "LEERESZTES";
    case AUTO_PAUSE: return "PIHENO";
    default:         return "?";
  }
}

bool isCommonStep(AutoStep step) {
  return step == AUTO_FILL || step == AUTO_DRAIN;
}

bool otherZoneInCommonOperation(uint8_t zone) {
  if (zone == 1) return isCommonStep(g_autoZ2.step);
  if (zone == 2) return isCommonStep(g_autoZ1.step);
  return false;
}

bool canZoneUseCommonSystem(uint8_t zone) {
  AutoZone* own = getAutoZone(zone);
  AutoZone* other = getAutoZone(zone == 1 ? 2 : 1);

  if (!own) return false;
  if (!isCommonStep(own->step)) return true;
  if (!other || !isCommonStep(other->step)) return true;

  // Aki korábban kérte a közös rendszert, az folytathatja.
  // Ha teljesen egyszerre indulnak, Z1 kap elsőbbséget.
  if (own->stepStartMs < other->stepStartMs) return true;
  if (own->stepStartMs > other->stepStartMs) return false;
  return zone == 1;
}

void resetAutoZone(AutoZone &az) {
  az.step = AUTO_IDLE;
  az.stepStartMs = millis();
  az.active = false;
}

void startAutoCycle(uint8_t zone) {
  AutoZone* az = getAutoZone(zone);
  ZoneSettings* zs = getZoneSettings(zone);

  if (!az || !zs) return;
  if (!zs->enabled) return;

  az->active = true;
  az->step = AUTO_FILL;
  az->stepStartMs = millis();

  Serial.print("AUTO Z");
  Serial.print(zone);
  Serial.println(" ciklus indul: FELTOLTES");
}

void setFillPath(uint8_t zone, uint8_t tank) {
  if (tank == 1) {
    g_autoOut[0] = true; // IBC1 választ
    g_autoOut[1] = true; // IBC1 nyit
  } else if (tank == 2) {
    g_autoOut[2] = true; // IBC2 választ
    g_autoOut[3] = true; // IBC2 nyit
  } else {
    g_autoOut[4] = true; // IBC3 / tiszta víz választ
    g_autoOut[5] = true; // IBC3 / tiszta víz nyit
  }

  if (zone == 1) g_autoOut[6] = true; // Z1 feltöltés választ
  if (zone == 2) g_autoOut[7] = true; // Z2 feltöltés választ

  g_autoOut[13] = true; // Feltöltő pumpa
}

void setDrainPath(uint8_t zone, uint8_t tank) {
  if (zone == 1) g_autoOut[8] = true; // Z1 leeresztés nyit
  if (zone == 2) g_autoOut[9] = true; // Z2 leeresztés nyit

  if (tank == 1) {
    g_autoOut[10] = true; // IBC1 vissza
  } else if (tank == 2) {
    g_autoOut[11] = true; // IBC2 vissza
  } else {
    g_autoOut[12] = true; // IBC3 / tiszta víz vissza
  }

  g_autoOut[14] = true; // Leeresztő szivattyú
}

void runAutoZone(uint8_t zone) {
  AutoZone* az = getAutoZone(zone);
  ZoneSettings* zs = getZoneSettings(zone);

  if (!az || !zs) return;

  unsigned long now = millis();

  if (!zs->enabled) {
    resetAutoZone(*az);
    return;
  }

  if (az->step == AUTO_IDLE) {
    startAutoCycle(zone);
    return;
  }

  uint8_t tank = zs->tank;

  // KÉSŐBB:
  // DS18B20 külső hőmérő beépítése után:
  // ha külső hőmérséklet > g_settings.hotTempC,
  // akkor tank = g_settings.cleanTank;

  if (az->step == AUTO_FILL) {
    if (!canZoneUseCommonSystem(zone)) {
      // Másik zóna birtokolja a közös rendszert, ezért ez vár.
      // Várakozás közben nem számoljuk a feltöltési timeoutot.
      az->stepStartMs = now;
      return;
    }

    setFillPath(zone, tank);

    if (zoneFull(zone)) {
      az->step = AUTO_SOAK;
      az->stepStartMs = now;

      Serial.print("AUTO Z");
      Serial.print(zone);
      Serial.println(": FELTOLTES kesz -> AZTATAS");
      return;
    }

    if (now - az->stepStartMs > minToMs(zs->fillTimeoutMin)) {
      setFault(zone == 1 ? "Z1 FELTOLTES TIMEOUT" : "Z2 FELTOLTES TIMEOUT");
      resetAutoZone(*az);
      return;
    }
  }

  else if (az->step == AUTO_SOAK) {
    if (now - az->stepStartMs >= minToMs(zs->soakMin)) {
      az->step = AUTO_DRAIN;
      az->stepStartMs = now;

      Serial.print("AUTO Z");
      Serial.print(zone);
      Serial.println(": AZTATAS kesz -> LEERESZTES");
      return;
    }
  }

  else if (az->step == AUTO_DRAIN) {
    if (!canZoneUseCommonSystem(zone)) {
      // Másik zóna birtokolja a közös rendszert, ezért ez vár.
      // Várakozás közben nem számoljuk a leeresztési időt.
      az->stepStartMs = now;
      return;
    }

    setDrainPath(zone, tank);

    // Jelenleg nincs külön "üres" szintérzékelő.
    // Ezért az első AUTO MVP-ben a leeresztést idő alapján zárjuk le.
    if (now - az->stepStartMs >= minToMs(zs->drainTimeoutMin)) {
      az->step = AUTO_PAUSE;
      az->stepStartMs = now;

      Serial.print("AUTO Z");
      Serial.print(zone);
      Serial.println(": LEERESZTES kesz -> PIHENO");
      return;
    }
  }

  else if (az->step == AUTO_PAUSE) {
    if (now - az->stepStartMs >= minToMs(zs->pauseMin)) {
      az->step = AUTO_FILL;
      az->stepStartMs = now;

      Serial.print("AUTO Z");
      Serial.print(zone);
      Serial.println(": PIHENO kesz -> FELTOLTES");
      return;
    }
  }
}

void runAutoMode() {
  autoAllOff();

  if (g_fault) return;

  if (isInSkipTime()) {
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    return;
  }

  runAutoZone(1);
  runAutoZone(2);
}

// ============================================================
// KIMENETEK
// ============================================================

void safeWrite(uint8_t pin, bool cmd) {
  bool out = RELAY_ACTIVE_LOW ? !cmd : cmd;
  digitalWrite(pin, out ? HIGH : LOW);
}

void writeAllManualOutputsOff() {
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    safeWrite(MANUAL_OUT_PINS[i], false);
  }
}

void applyOutputs() {
  if (g_fault) {
    writeAllManualOutputsOff();
    return;
  }

  if (g_mode == MODE_MANUAL) {
    enforceManualSafety();

    for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
      safeWrite(MANUAL_OUT_PINS[i], g_manual.out[i]);
    }
    return;
  }

  if (g_mode == MODE_AUTO) {
    for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
      safeWrite(MANUAL_OUT_PINS[i], g_autoOut[i]);
    }
    return;
  }

  writeAllManualOutputsOff();
}

// ============================================================
// HMI STÁTUSZ
// ============================================================

void writeManualButtonsToHMI() {
  // MANUAL oldal dual-state gombok visszaírása.
  // Ha valamelyik objektum nincs a Nextionon, a parancs nem gond, csak nem látszik.
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    nexSetValue(MANUAL_HMI_BTNS[i], g_manual.out[i] ? 1 : 0);
  }
}


bool zoneHasCommonOutput(uint8_t zone) {
  if (zone == 1) {
    return g_autoOut[6] || g_autoOut[8];   // Z1 fill select vagy Z1 drain open
  }
  if (zone == 2) {
    return g_autoOut[7] || g_autoOut[9];   // Z2 fill select vagy Z2 drain open
  }
  return false;
}

const char* autoDisplayState(uint8_t zone) {
  AutoZone* az = getAutoZone(zone);
  ZoneSettings* zs = getZoneSettings(zone);

  if (!az || !zs) return "-";
  if (!zs->enabled) return "KI";

  if (g_mode == MODE_MANUAL) return "MANUAL";
  if (isInSkipTime()) return "TILTOTT IDO";

  if (isCommonStep(az->step) && !canZoneUseCommonSystem(zone)) {
    return "VARAKOZAS";
  }

  return autoStepName(az->step);
}

uint8_t activeTankForZone(uint8_t zone) {
  ZoneSettings* zs = getZoneSettings(zone);
  if (!zs) return 0;

  // Később ide jön a DS18B20 logika:
  // ha külső hőmérséklet > g_settings.hotTempC, akkor return g_settings.cleanTank;

  return zs->tank;
}

unsigned long autoStepDurationMs(uint8_t zone) {
  AutoZone* az = getAutoZone(zone);
  ZoneSettings* zs = getZoneSettings(zone);

  if (!az || !zs) return 0;

  switch (az->step) {
    case AUTO_FILL:  return minToMs(zs->fillTimeoutMin);
    case AUTO_SOAK:  return minToMs(zs->soakMin);
    case AUTO_DRAIN: return minToMs(zs->drainTimeoutMin);
    case AUTO_PAUSE: return minToMs(zs->pauseMin);
    default:         return 0;
  }
}

void formatRemain(uint8_t zone, char* buf, size_t len) {
  AutoZone* az = getAutoZone(zone);
  ZoneSettings* zs = getZoneSettings(zone);

  if (!buf || len == 0) return;

  if (!az || !zs || !zs->enabled || g_mode == MODE_MANUAL) {
    snprintf(buf, len, "-");
    return;
  }

  if (az->step == AUTO_IDLE) {
    snprintf(buf, len, "-");
    return;
  }

  if (isCommonStep(az->step) && !canZoneUseCommonSystem(zone)) {
    snprintf(buf, len, "VAR");
    return;
  }

  unsigned long dur = autoStepDurationMs(zone);
  if (dur == 0) {
    snprintf(buf, len, "-");
    return;
  }

  unsigned long elapsed = millis() - az->stepStartMs;
  unsigned long remain = (elapsed >= dur) ? 0 : (dur - elapsed);

  unsigned long sec = remain / 1000UL;
  unsigned int minPart = sec / 60UL;
  unsigned int secPart = sec % 60UL;

  snprintf(buf, len, "%02u:%02u", minPart, secPart);
}

uint8_t commonActiveZone() {
  if (g_autoOut[13]) { // Feltöltő pumpa
    if (g_autoOut[6]) return 1;
    if (g_autoOut[7]) return 2;
  }

  if (g_autoOut[14]) { // Leeresztő szivattyú
    if (g_autoOut[8]) return 1;
    if (g_autoOut[9]) return 2;
  }

  return 0;
}

const char* commonOperationText() {
  if (g_autoOut[13]) return "FELTOLTES";
  if (g_autoOut[14]) return "LEERESZTES";
  return "VAR";
}

uint8_t commonIbc() {
  if (g_autoOut[0] || g_autoOut[1] || g_autoOut[10]) return 1;
  if (g_autoOut[2] || g_autoOut[3] || g_autoOut[11]) return 2;
  if (g_autoOut[4] || g_autoOut[5] || g_autoOut[12]) return 3;
  return 0;
}

const char* commonPumpText() {
  if (g_autoOut[13]) return "FELTOLTO";
  if (g_autoOut[14]) return "LEERESZTO";
  return "KI";
}

const char* commonOxygenText() {
  // Jelenleg nincs külön oxigén kimenet/logika a kódban.
  return "KI";
}

void writeZoneStatusToHMI(
  uint8_t zone,
  const char* activeObj,
  const char* stateObj,
  const char* ibcObj,
  const char* remainObj,
  const char* fullObj
) {
  ZoneSettings* zs = getZoneSettings(zone);

  char buf[16];

  nexSetText(activeObj, (zs && zs->enabled) ? "AKTIV" : "KI");
  nexSetText(stateObj, autoDisplayState(zone));

  uint8_t tank = activeTankForZone(zone);
  if (tank == 0) snprintf(buf, sizeof(buf), "-");
  else snprintf(buf, sizeof(buf), "IBC%u", tank);
  nexSetText(ibcObj, buf);

  formatRemain(zone, buf, sizeof(buf));
  nexSetText(remainObj, buf);

  nexSetText(fullObj, zoneFull(zone) ? "TELE" : "NEM TELE");
}


void writeMainStatusToHMI() {
  if (g_rtcOk) {
    DateTime now = rtc.now();

    char timeBuf[12];
    sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

    // A régi és az új mezőnevet is írjuk, így nem baj, ha valamelyik nincs a HMI-n.
    nexSetText("tTime", timeBuf);
    nexSetText("tMainTime", timeBuf);
  }

  // DS18B20 későbbi bővítéshez.
  nexSetText("tTemp", "--.- C");

  if (g_mode == MODE_MANUAL) {
    nexSetText("tModeState", "MANUAL");
  } else {
    nexSetText("tModeState", "AUTO");
  }

  writeZoneStatusToHMI(1, "tZ1Active", "tZ1State", "tZ1Ibc", "tZ1Remain", "tZ1Full");
  writeZoneStatusToHMI(2, "tZ2Active", "tZ2State", "tZ2Ibc", "tZ2Remain", "tZ2Full");

  char commonIbcBuf[8];
  uint8_t cibc = commonIbc();

  if (cibc == 0) snprintf(commonIbcBuf, sizeof(commonIbcBuf), "-");
  else snprintf(commonIbcBuf, sizeof(commonIbcBuf), "IBC%u", cibc);

  // Közös rendszer mezők. Több lehetséges objektumnevet is írunk,
  // így akkor is megjelenik, ha a HMI-n rövidebb nevet adtál a mezőnek.
char ibc1[8];
char ibc2[8];

snprintf(ibc1, sizeof(ibc1), "IBC%u", activeTankForZone(1));
snprintf(ibc2, sizeof(ibc2), "IBC%u", activeTankForZone(2));


nexSetText("tZ1Source", ibc1);
nexSetText("tZ2Source", ibc2);

nexSetText("tPump", commonPumpText());
nexSetText("tOxigen", "KI");
nexSetText("tCurrentAction", commonOperationText());
nexSetText("tCurrentSource", commonIbcBuf);

  nexSetText("tError", g_faultText);
}

void updateHmiPeriodic() {
  if (millis() - g_lastHmiUpdateMs < 1000) return;

  g_lastHmiUpdateMs = millis();

  writeMainStatusToHMI();

  // Ha az RTC oldalon vagyunk, az óra kijelzés frissülhet.
  writeRtcToHMI();
}

// ============================================================
// DEBUG
// ============================================================

void printSettings() {
  Serial.println("===== SETTINGS =====");

  Serial.print("Z1 enable: "); Serial.println(g_settings.z1.enabled);
  Serial.print("Z1 tank: "); Serial.println(g_settings.z1.tank);
  Serial.print("Z1 fill TO: "); Serial.println(g_settings.z1.fillTimeoutMin);
  Serial.print("Z1 soak: "); Serial.println(g_settings.z1.soakMin);
  Serial.print("Z1 drain: "); Serial.println(g_settings.z1.drainTimeoutMin);
  Serial.print("Z1 full lost: "); Serial.println(g_settings.z1.fullLostMin);
  Serial.print("Z1 pause: "); Serial.println(g_settings.z1.pauseMin);

  Serial.print("Z2 enable: "); Serial.println(g_settings.z2.enabled);
  Serial.print("Z2 tank: "); Serial.println(g_settings.z2.tank);
  Serial.print("Z2 fill TO: "); Serial.println(g_settings.z2.fillTimeoutMin);
  Serial.print("Z2 soak: "); Serial.println(g_settings.z2.soakMin);
  Serial.print("Z2 drain: "); Serial.println(g_settings.z2.drainTimeoutMin);
  Serial.print("Z2 full lost: "); Serial.println(g_settings.z2.fullLostMin);
  Serial.print("Z2 pause: "); Serial.println(g_settings.z2.pauseMin);

  Serial.print("Clean tank: "); Serial.println(g_settings.cleanTank);
  Serial.print("Hot temp C: "); Serial.println(g_settings.hotTempC);

  Serial.print("Skip enabled: "); Serial.println(g_settings.skipEnabled);
  Serial.print("Skip start: ");
  Serial.print(g_settings.skipStartH);
  Serial.print(":");
  if (g_settings.skipStartM < 10) Serial.print("0");
  Serial.println(g_settings.skipStartM);

  Serial.print("Skip end: ");
  Serial.print(g_settings.skipEndH);
  Serial.print(":");
  if (g_settings.skipEndM < 10) Serial.print("0");
  Serial.println(g_settings.skipEndM);

  Serial.print("In skip time now: ");
  Serial.println(isInSkipTime() ? "YES" : "NO");

  Serial.print("CRC: "); Serial.println(g_settings.crc);
  Serial.println("====================");
}

void printManual() {
  Serial.println("===== MANUAL =====");

  Serial.print("MODE: ");
  Serial.println(g_mode == MODE_MANUAL ? "MANUAL" : "AUTO");

  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    Serial.print(i + 1);
    Serial.print(" - ");
    Serial.print(MANUAL_OUT_NAMES[i]);
    Serial.print(": ");
    Serial.println(g_manual.out[i] ? "ON" : "OFF");
  }

  Serial.print("FAULT: ");
  Serial.println(g_faultText);

  Serial.println("==================");
}

void printAuto() {
  Serial.println("===== AUTO =====");

  Serial.print("Z1 step: ");
  Serial.println(autoStepName(g_autoZ1.step));

  Serial.print("Z2 step: ");
  Serial.println(autoStepName(g_autoZ2.step));

  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    Serial.print(i + 1);
    Serial.print(" - ");
    Serial.print(MANUAL_OUT_NAMES[i]);
    Serial.print(": ");
    Serial.println(g_autoOut[i] ? "ON" : "OFF");
  }

  Serial.println("================");
}

// ============================================================
// Nextion események
// ============================================================

void handleNextionCommand(uint8_t cmd) {
  if (cmd == 0x01) {
    Serial.println("bApply: settings oldal -> RAM");

    if (readSettingsFromHMI()) {
      Serial.println("Apply OK.");
      printSettings();
      writeSettingsToHMI();
    } else {
      Serial.println("Apply HIBA.");
    }
  }

  else if (cmd == 0x02) {
    Serial.println("bSave: settings oldal -> RAM -> EEPROM");

    if (readSettingsFromHMI()) {
      saveSettingsToEEPROM();
      printSettings();
      writeSettingsToHMI();
    } else {
      Serial.println("Save HIBA: settings oldal olvasasa sikertelen.");
    }
  }

  else if (cmd == 0x03) {
    Serial.println("bDefault: gyari technologiai beallitasok");
    loadDefaultSettings();
    saveSettingsToEEPROM();
    writeSettingsToHMI();
    printSettings();
  }

  else if (cmd == 0x04) {
    Serial.println("bSaveSkip: tiltott idoszak -> RAM -> EEPROM");

    if (readSkipSettingsFromHMI()) {
      saveSettingsToEEPROM();
      printSettings();
      writeSkipSettingsToHMI();
    } else {
      Serial.println("SaveSkip HIBA: tiltott idoszak olvasasa sikertelen.");
    }
  }

  else if (cmd == 0x05) {
    Serial.println("bSetRtc: RTC beallitas HMI-rol");

    if (readRtcFromHMIAndSet()) {
      Serial.println("RTC beallitas OK.");
      writeRtcToHMI();
    } else {
      Serial.println("RTC beallitas HIBA.");
    }
  }

  else if (cmd == 0x14) {
    if (!g_rtcEditFieldsLoaded) {
      Serial.println("RTC oldal megnyitva - szerkeszto mezok egyszeri feltoltese");
      delay(200);
      writeRtcEditFieldsToHMI();
      g_rtcEditFieldsLoaded = true;
    }
  }

  else if (cmd == 0x15) {
    Serial.println("RTC oldal elhagyva - kovetkezo belepesnel ujratoltheto");
    g_rtcEditFieldsLoaded = false;
  }

  // Üzemmód / global
  else if (cmd == 0x10) {
    Serial.println("MANUAL mod");

    g_mode = MODE_MANUAL;

    manualAllOff();
    autoAllOff();
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    clearFault();

    writeManualButtonsToHMI();
  }

  else if (cmd == 0x11) {
    Serial.println("AUTO mod indul");

    g_mode = MODE_AUTO;

    manualAllOff();
    autoAllOff();
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    clearFault();

    writeManualButtonsToHMI();
  }

  else if (cmd == 0x12) {
    Serial.println("ALL OFF");

    manualAllOff();
    autoAllOff();
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    clearFault();

    writeManualButtonsToHMI();
  }

  else if (cmd == 0x13) {
    Serial.println("btR / RESET - minden kimenet KI");

    manualAllOff();
    autoAllOff();
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    clearFault();

    writeManualButtonsToHMI();
  }

  else if (cmd == 0x16) {
    if (g_mode == MODE_MANUAL) {
      Serial.println("btMode: AUTO mod");

      g_mode = MODE_AUTO;
    } else {
      Serial.println("btMode: MANUAL mod");

      g_mode = MODE_MANUAL;
    }

    // Módváltáskor minden kimenet KI, és az automata ciklus várakozó állapotba kerül.
    manualAllOff();
    autoAllOff();
    resetAutoZone(g_autoZ1);
    resetAutoZone(g_autoZ2);
    clearFault();

    writeManualButtonsToHMI();
  }

  // MANUAL technológiai kimenetek: 0x20..0x2E = 15 db feltöltés/leeresztés kimenet
  else if (cmd >= 0x20 && cmd <= 0x2E) {
    uint8_t idx = cmd - 0x20;
    toggleManualOutput(idx);
  }

  else {
    Serial.print("Ismeretlen Nextion parancs: 0x");
    Serial.println(cmd, HEX);
  }

  if (g_mode == MODE_AUTO) {
    runAutoMode();
  }

  enforceManualSafety();
  applyOutputs();
  writeMainStatusToHMI();
  writeManualButtonsToHMI();

  if (g_mode == MODE_MANUAL) printManual();
  else printAuto();
}

void processNextionEvents() {
  static uint8_t state = 0;
  static uint8_t cmd = 0;
  static uint8_t ffCount = 0;

  while (nex.available()) {
    uint8_t b = nex.read();

    switch (state) {
      case 0:
        if (b == 0xA5) {
          state = 1;
        }
        break;

      case 1:
        cmd = b;
        ffCount = 0;
        state = 2;
        break;

      case 2:
        if (b == 0xFF) {
          ffCount++;
          if (ffCount >= 3) {
            handleNextionCommand(cmd);
            state = 0;
          }
        } else {
          state = 0;
        }
        break;
    }
  }
}

// ============================================================
// Setup / loop
// ============================================================

void setupPins() {
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    pinMode(MANUAL_OUT_PINS[i], OUTPUT);
  }

  pinMode(PIN_DI_Z1_FULL, INPUT);
  pinMode(PIN_DI_Z2_FULL, INPUT);

  manualAllOff();
  autoAllOff();
  applyOutputs();
}

void setup() {
  Serial.begin(115200);
  nex.begin(9600);

  setupPins();

  delay(500);

  nexCmd("bkcmd=0");
  delay(1500);

  Serial.println("Indoor Garden - V2_6_AUTO_FIX_260526 indul");

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("HIBA: DS3231 nem talalhato!");
    g_rtcOk = false;
  } else {
    g_rtcOk = true;
    Serial.println("DS3231 RTC OK.");

    if (rtc.lostPower()) {
      Serial.println("RTC elvesztette az idot, forditasi idore allitom.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  if (!loadSettingsFromEEPROM()) {
    Serial.println("Gyari beallitasok betoltese.");
    loadDefaultSettings();
    saveSettingsToEEPROM();
  }

  g_mode = MODE_MANUAL;

  manualAllOff();
  autoAllOff();
  resetAutoZone(g_autoZ1);
  resetAutoZone(g_autoZ2);
  clearFault();

  applyOutputs();
  printSettings();

  nexCmd("page main");
  delay(300);
  writeMainStatusToHMI();

  Serial.println("Kesz.");
  Serial.println("bApply = settings RAM");
  Serial.println("bSave = settings EEPROM");
  Serial.println("bDefault = gyari technologiai beallitasok");
  Serial.println("bSaveSkip = tiltott ido EEPROM");
  Serial.println("bSetRtc = RTC beallitas");
  Serial.println("bManual = MANUAL mod");
  Serial.println("bAuto = AUTO mod indul");
  Serial.println("bAllOff = minden parancs KI");
  Serial.println("btR = reset, minden kimenet KI");
  Serial.println("btFill... / btDrain... = 15 db manual technologiai kimenet");
}

void loop() {
  processNextionEvents();

  if (g_mode == MODE_AUTO) {
    runAutoMode();
  }

  applyOutputs();
  updateHmiPeriodic();
}
