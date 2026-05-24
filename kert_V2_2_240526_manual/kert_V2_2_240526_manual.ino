/*
  Controllino MAXI + Nextion + DS3231 + EEPROM
  Indoor Garden - ALAP + MANUAL FELTOLTES/LEERESZTES 15 KIMENET

  Ez a verzio a GitHub-on levo kert_V2.0_240526.ino alapra epul:
  - Nextion Serial2 kommunikacio
  - settings oldal olvasas/iras
  - EEPROM mentes CRC-vel
  - tiltott idoszak kulon mentese
  - DS3231 RTC olvasas / beallitas
  - MAIN statusz frissites
  - MANUAL direkt kimenet vezerles HMI gombokkal

  FONTOS:
  A MANUAL gombok a te technologiai kiosztasod szerint mukodnek:
  - 8 db FELTOLTES kezi kimenet
  - 7 db LEERESZTES kezi kimenet
  Minden gomb egy konkret szelepet/pumpat/szivattyut kapcsol KI/BE.

  Nextion Touch Release Event parancsok:

  SETTINGS oldal:
    bApply:     printh A5 01 FF FF FF
    bSave:      printh A5 02 FF FF FF
    bDefault:   printh A5 03 FF FF FF
    bSaveSkip:  printh A5 04 FF FF FF
    bSetRtc:    printh A5 05 FF FF FF

  UZEMMOD / GLOBAL:
    bManual:    printh A5 10 FF FF FF
    bAuto:      printh A5 11 FF FF FF
    bAllOff:    printh A5 12 FF FF FF
    btR:        printh A5 13 FF FF FF   // MANUAL reset, minden kimenet KI

  MANUAL FELTOLTES gombok:
    btFillIbc1Sel:   printh A5 20 FF FF FF   // IBC1 valaszt
    btFillIbc1Open:  printh A5 21 FF FF FF   // IBC1 nyit
    btFillIbc2Sel:   printh A5 22 FF FF FF   // IBC2 valaszt
    btFillIbc2Open:  printh A5 23 FF FF FF   // IBC2 nyit
    btFillIbc3Sel:   printh A5 24 FF FF FF   // IBC3 valaszt
    btFillIbc3Open:  printh A5 25 FF FF FF   // IBC3 nyit
    btFillZ1Sel:     printh A5 26 FF FF FF   // Z1 valaszt
    btFillZ2Sel:     printh A5 27 FF FF FF   // Z2 valaszt

  MANUAL LEERESZTES gombok:
    btDrainZ1Open:   printh A5 28 FF FF FF   // Z1 nyit
    btDrainZ2Open:   printh A5 29 FF FF FF   // Z2 nyit
    btDI1:      printh A5 2A FF FF FF   // IBC1 nyit
    btDI2:      printh A5 2B FF FF FF   // IBC2 nyit
    btDI3:      printh A5 2C FF FF FF   // IBC3 nyit
    btDP:      printh A5 2D FF FF FF   // Pumpa
    btDS:     printh A5 2E FF FF FF   // Szivattyu

  Javasolt Nextion objektumnevek MANUAL oldalon:
    btFillIbc1Sel, btFillIbc1Open, btFillIbc2Sel, btFillIbc2Open,
    btFillIbc3Sel, btFillIbc3Open, btFillZ1Sel, btFillZ2Sel,
    btDrainZ1Open, btDrainZ2Open, btDI1, btDI2,
    btDI3, btDS, btDP, btR

  Javasolt MAIN objektumok:
    tMainTime, tTemp, tModeState, tError

  Javasolt RTC objektumok:
    tRtcTime, tRtcDate, nYear, nMonth, nDay, nHour, nMinute, nSecond

  Megjegyzes:
  Ha nalad mas az objektum neve, akkor csak a lenti nexSetValue/nexSetText hivasokban kell atirni.
*/

#include <Controllino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"

#define nex Serial2

#define EEPROM_ADDR 0
#define SETTINGS_MAGIC 0x4752544EUL
#define SETTINGS_VERSION 3

RTC_DS3231 rtc;
bool g_rtcOk = false;
bool g_rtcEditFieldsLoaded = false;

// ============================================================
// IO KIOSZTAS - EZT ELLENORIZD A SAJAT BEKOTESED SZERINT
// ============================================================
// 10 relekimenet a Controllino MAXI-n.
// Minden kimenet csak relemodult kapcsol, nem kozvetlen fogyasztot.

// 15 kezi technologiai kimenet a te leirasod szerint.
// Alap fizikai kiosztas:
//   01..10 = Controllino relékimenetek R0..R9
//   11..15 = Controllino digitalis kimenetek D0..D4
// Ha a bekotesed mas, ITT kell atirni a fizikai pineket.
const uint8_t MANUAL_OUT_COUNT = 15;

const uint8_t PIN_FILL_IBC1_SEL   = CONTROLLINO_R0;  // 01 - FELTOLTES: IBC1 valaszt
const uint8_t PIN_FILL_IBC1_OPEN  = CONTROLLINO_R1;  // 02 - FELTOLTES: IBC1 nyit
const uint8_t PIN_FILL_IBC2_SEL   = CONTROLLINO_R2;  // 03 - FELTOLTES: IBC2 valaszt
const uint8_t PIN_FILL_IBC2_OPEN  = CONTROLLINO_R3;  // 04 - FELTOLTES: IBC2 nyit
const uint8_t PIN_FILL_IBC3_SEL   = CONTROLLINO_R4;  // 05 - FELTOLTES: IBC3 valaszt
const uint8_t PIN_FILL_IBC3_OPEN  = CONTROLLINO_R5;  // 06 - FELTOLTES: IBC3 nyit
const uint8_t PIN_FILL_Z1_SEL     = CONTROLLINO_R6;  // 07 - FELTOLTES: Z1 valaszt
const uint8_t PIN_FILL_Z2_SEL     = CONTROLLINO_R7;  // 08 - FELTOLTES: Z2 valaszt

const uint8_t PIN_DRAIN_Z1_OPEN   = CONTROLLINO_R8;  // 09 - LEERESZTES: Z1 nyit
const uint8_t PIN_DRAIN_Z2_OPEN   = CONTROLLINO_R9;  // 10 - LEERESZTES: Z2 nyit
const uint8_t PIN_DRAIN_IBC1_OPEN = CONTROLLINO_D0;  // 11 - LEERESZTES: IBC1 nyit
const uint8_t PIN_DRAIN_IBC2_OPEN = CONTROLLINO_D1;  // 12 - LEERESZTES: IBC2 nyit
const uint8_t PIN_DRAIN_IBC3_OPEN = CONTROLLINO_D2;  // 13 - LEERESZTES: IBC3 nyit
const uint8_t PIN_FILL_PUMP       = CONTROLLINO_D3;  // 14 - Pumpa
const uint8_t PIN_DRAIN_PUMP      = CONTROLLINO_D4;  // 15 - Szivattyu

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

// Bemenetek - csak azok legyenek itt, amiket tenyleg hasznalsz.
// A Controllino konyvtar verzio miatt NEM hasznalok CONTROLLINO_IN2...IN7 neveket.
// Ha mar kijavitottad a sajat jelolesedre, itt is atirhatod.
const uint8_t PIN_DI_Z1_FULL = CONTROLLINO_DI0;
const uint8_t PIN_DI_Z2_FULL = CONTROLLINO_DI1;

// Logikai polaritas.
// Ha a relemodulod aktiv LOW bemenetu, ird true-ra.
const bool RELAY_ACTIVE_LOW = false;

// ============================================================
// ADATSZERKEZETEK
// ============================================================
struct ZoneSettings {
  bool enabled;
  uint8_t tank;              // 1 vagy 2 tapoldatos IBC
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

  uint8_t cleanTank;         // jelenleg 3
  int16_t hotTempC;          // kulso homerseklet kuszob kesobbre

  bool skipEnabled;
  uint8_t skipStartH;
  uint8_t skipStartM;
  uint8_t skipEndH;
  uint8_t skipEndM;

  uint16_t crc;
};

Settings g_settings;

// ============================================================
// UZEMMOD / MANUAL PARANCSOK
// ============================================================
enum WorkMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

WorkMode g_mode = MODE_MANUAL;

struct ManualCmd {
  bool out[MANUAL_OUT_COUNT];
};

ManualCmd g_manual;

bool g_fault = false;
char g_faultText[32] = "NINCS HIBA";

unsigned long g_lastHmiUpdateMs = 0;

// ============================================================
// Nextion alapfuggvenyek
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
    value = (uint32_t)buf[1] |
            ((uint32_t)buf[2] << 8) |
            ((uint32_t)buf[3] << 16) |
            ((uint32_t)buf[4] << 24);
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
  bool oldSkipEnabled = g_settings.skipEnabled;
  uint8_t oldSkipStartH = g_settings.skipStartH;
  uint8_t oldSkipStartM = g_settings.skipStartM;
  uint8_t oldSkipEndH = g_settings.skipEndH;
  uint8_t oldSkipEndM = g_settings.skipEndM;

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

  // Tiltott ido megtartasa default technologiai beallitasnal.
  g_settings.skipEnabled = oldSkipEnabled;
  g_settings.skipStartH = oldSkipStartH;
  g_settings.skipStartM = oldSkipStartM;
  g_settings.skipEndH = oldSkipEndH;
  g_settings.skipEndM = oldSkipEndM;

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

  if (g_settings.z2.fillTimeoutMin < 1) g_settings.z2.fillTimeoutMin = 1;
  if (g_settings.z2.soakMin < 1) g_settings.z2.soakMin = 1;
  if (g_settings.z2.drainTimeoutMin < 1) g_settings.z2.drainTimeoutMin = 1;
  if (g_settings.z2.fullLostMin < 1) g_settings.z2.fullLostMin = 1;
  if (g_settings.z2.pauseMin < 1) g_settings.z2.pauseMin = 1;

  if (g_settings.skipStartH > 23) g_settings.skipStartH = 23;
  if (g_settings.skipEndH > 23) g_settings.skipEndH = 23;
  if (g_settings.skipStartM > 59) g_settings.skipStartM = 59;
  if (g_settings.skipEndM > 59) g_settings.skipEndM = 59;
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
// HMI tiltott ido oldal
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
void writeRtcToHMI()
{
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

  // MANUAL szerviz mod: minden gomb direkt kapcsol.
  // Itt szandekosan NEM kapcsolgatunk le automatikusan mas kimeneteket,
  // mert szervizeleskor te dontod el, milyen utvonalat akarsz probalni.
  // Ha kesobb szeretned, ide be tudjuk tenni peldaul:
  // - egyszerre csak egy IBC valasztas
  // - pumpa es szivattyu egyuttes tiltasa
  // - feltoltes/leeresztes oldal egymas elleni tiltasa
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
  if (g_fault || g_mode != MODE_MANUAL) {
    writeAllManualOutputsOff();
    return;
  }

  enforceManualSafety();

  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    safeWrite(MANUAL_OUT_PINS[i], g_manual.out[i]);
  }
}

// ============================================================
// HMI STATUSZ
// ============================================================
void writeManualButtonsToHMI() {
  // MANUAL oldal dual-state gombok visszairasa.
  // Ha valamelyik objektum nincs a Nextionon, a parancs nem gond, csak nem latszik.
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    nexSetValue(MANUAL_HMI_BTNS[i], g_manual.out[i] ? 1 : 0);
  }
}

void writeMainStatusToHMI() {
  if (g_rtcOk) {
    DateTime now = rtc.now();
    char timeBuf[12];
    sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    nexSetText("tMainTime", timeBuf);
  }

  // DS18B20 kesobbi boviteshez.
  nexSetText("tTemp", "--.- C");

  if (g_mode == MODE_MANUAL) {
    nexSetText("tModeState", "MANUAL");
  } else {
    if (isInSkipTime()) nexSetText("tModeState", "AUTO - TILTOTT IDO");
    else nexSetText("tModeState", "AUTO - VARAKOZIK");
  }

  nexSetText("tError", g_faultText);
}

void writeRtcEditFieldsToHMI()
{
  if (!g_rtcOk) return;

  DateTime now = rtc.now();

  nexSetValue("nYear",   now.year());
  nexSetValue("nMonth",  now.month());
  nexSetValue("nDay",    now.day());
  nexSetValue("nHour",   now.hour());
  nexSetValue("nMinute", now.minute());
  nexSetValue("nSecond", now.second());

  nexSetValue("btSkipEnable", g_settings.skipEnabled ? 1 : 0);
  nexSetValue("nSkipStartH",  g_settings.skipStartH);
  nexSetValue("nSkipStartM",  g_settings.skipStartM);
  nexSetValue("nSkipEndH",    g_settings.skipEndH);
  nexSetValue("nSkipEndM",    g_settings.skipEndM);
}

void updateHmiPeriodic() {
  if (millis() - g_lastHmiUpdateMs < 1000) return;
  g_lastHmiUpdateMs = millis();

  writeMainStatusToHMI();
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

  Serial.print("In skip time now: "); Serial.println(isInSkipTime() ? "YES" : "NO");
  Serial.print("CRC: "); Serial.println(g_settings.crc);
  Serial.println("====================");
}

void printManual() {
  Serial.println("===== MANUAL =====");
  Serial.print("MODE: "); Serial.println(g_mode == MODE_MANUAL ? "MANUAL" : "AUTO");
  for (uint8_t i = 0; i < MANUAL_OUT_COUNT; i++) {
    Serial.print(i + 1);
    Serial.print(" - ");
    Serial.print(MANUAL_OUT_NAMES[i]);
    Serial.print(": ");
    Serial.println(g_manual.out[i] ? "ON" : "OFF");
  }
  Serial.print("FAULT: "); Serial.println(g_faultText);
  Serial.println("==================");
}

// ============================================================
// Nextion esemenyek
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

  // Uzemmod / global
  else if (cmd == 0x10) {
    Serial.println("MANUAL mod");
    g_mode = MODE_MANUAL;
    manualAllOff();
    clearFault();
    writeManualButtonsToHMI();
  }
  else if (cmd == 0x11) {
    Serial.println("AUTO mod - jelenleg biztonsagi alap, kimenetek KI");
    g_mode = MODE_AUTO;
    manualAllOff();
    clearFault();
    writeManualButtonsToHMI();
  }
  else if (cmd == 0x12) {
    Serial.println("ALL OFF");
    manualAllOff();
    clearFault();
    writeManualButtonsToHMI();
  }
  else if (cmd == 0x13) {
    Serial.println("btR / RESET - minden manual kimenet KI");
    manualAllOff();
    clearFault();
    writeManualButtonsToHMI();
  }

  // MANUAL technologiai kimenetek: 0x20..0x2E = 15 db feltoltes/leeresztes kimenet
  else if (cmd >= 0x20 && cmd <= 0x2E) {
    uint8_t idx = cmd - 0x20;
    toggleManualOutput(idx);
  }
  else {
    Serial.print("Ismeretlen Nextion parancs: 0x");
    Serial.println(cmd, HEX);
  }

  enforceManualSafety();
  applyOutputs();
  writeMainStatusToHMI();
  writeManualButtonsToHMI();
  printManual();
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
  applyOutputs();
}

void setup() {
  Serial.begin(115200);
  nex.begin(9600);

  setupPins();

  delay(500);
  nexCmd("bkcmd=0");
  delay(1500);

  Serial.println("Indoor Garden - ALAP + MANUAL FELTOLTES/LEERESZTES 15 KIMENET indul");

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
  clearFault();
  applyOutputs();

  printSettings();

  nexCmd("page main");
  delay(300);
  writeMainStatusToHMI();

  Serial.println("Kesz.");
  Serial.println("bApply     = settings RAM");
  Serial.println("bSave      = settings EEPROM");
  Serial.println("bDefault   = gyari technologiai beallitasok");
  Serial.println("bSaveSkip  = tiltott ido EEPROM");
  Serial.println("bSetRtc    = RTC beallitas");
  Serial.println("bManual    = MANUAL mod");
  Serial.println("bAuto      = AUTO mod / jelenleg kimenetek KI");
  Serial.println("bAllOff    = minden manual parancs KI");
  Serial.println("btR        = manual oldali reset, minden kimenet KI");
  Serial.println("btFill... / btDrain... = 15 db manual technologiai kimenet");
}

void loop() {
  processNextionEvents();
  applyOutputs();
  updateHmiPeriodic();
}
