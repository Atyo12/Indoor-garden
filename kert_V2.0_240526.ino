/*
  Controllino MAXI + Nextion + DS3231 + EEPROM teszt
  --------------------------------------------------
  Tartalmazza:
  - Nextion Serial2 kommunikáció
  - settings oldal olvasás/írás
  - EEPROM mentés CRC-vel
  - Default technológiai értékek
  - tiltott időszak külön mentése
  - DS3231 RTC olvasás
  - RTC dátum/idő kiírás Nextionra
  - RTC beállítás HMI-ről

  Nextion Touch Release Event parancsok:
  bApply:
    printh A5 01 FF FF FF

  bSave:
    printh A5 02 FF FF FF

  bDefault:
    printh A5 03 FF FF FF

  bSaveSkip:
    printh A5 04 FF FF FF

  bSetRtc:
    printh A5 05 FF FF FF
*/

#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"

#define nex Serial2

#define EEPROM_ADDR 0
#define SETTINGS_MAGIC 0x4752544EUL
#define SETTINGS_VERSION 2

RTC_DS3231 rtc;
bool g_rtcOk = false;

struct ZoneSettings {
  bool enabled;
  uint8_t tank;
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

  uint8_t cleanTank;
  int16_t hotTempC;

  bool skipEnabled;
  uint8_t skipStartH;
  uint8_t skipStartM;
  uint8_t skipEndH;
  uint8_t skipEndM;

  uint16_t crc;
};

Settings g_settings;

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

  if (idx >= 8 &&
      buf[0] == 0x71 &&
      buf[5] == 0xFF &&
      buf[6] == 0xFF &&
      buf[7] == 0xFF) {

    value =
      (uint32_t)buf[1] |
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

  // Tiltott idő megtartása
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
  // Ha nálad az oldal neve nem "rtc", írd át itt.
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

void writeMainStatusToHMI() {
  if (g_rtcOk) {
    DateTime now = rtc.now();

    char timeBuf[12];
    sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

    nexSetText("tMainTime", timeBuf);
  }

  // Később ide jön a DS18B20 külső hőmérő értéke
  nexSetText("tTemp", "--.- C");

  nexSetText("tModeState", "MANUAL");
  nexSetText("tError", "NINCS HIBA");
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

// Tiltott idő ellenőrzés. Később az AUTO állapotgép ezt fogja használni.
bool isInSkipTime() {
  if (!g_rtcOk) return false;
  if (!g_settings.skipEnabled) return false;

  DateTime now = rtc.now();

  uint16_t nowMin = now.hour() * 60 + now.minute();
  uint16_t startMin = g_settings.skipStartH * 60 + g_settings.skipStartM;
  uint16_t endMin = g_settings.skipEndH * 60 + g_settings.skipEndM;

  if (startMin == endMin) {
    return false;
  }

  if (startMin < endMin) {
    return nowMin >= startMin && nowMin < endMin;
  }

  // Éjfélen átnyúló tiltás, pl. 22:00 - 06:00
  return nowMin >= startMin || nowMin < endMin;
}

// ============================================================
// Debug
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

// ============================================================
// Nextion események
// ============================================================

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

void setup() {
  Serial.begin(115200);
  nex.begin(9600);

  delay(500);
  nexCmd("bkcmd=0");

  delay(1500);

  Serial.println("Nextion + EEPROM + DS3231 RTC teszt indul");

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

  printSettings();

  // Induláskor a settings oldalt tölti fel.
 nexCmd("page main");
 delay(300);
 writeMainStatusToHMI();

  Serial.println("Kesz.");
  Serial.println("bApply = settings RAM");
  Serial.println("bSave = settings EEPROM");
  Serial.println("bDefault = gyari technologiai beallitasok");
  Serial.println("bSaveSkip = tiltott ido EEPROM");
  Serial.println("bSetRtc = RTC beallitas");
}

void loop() {
  processNextionEvents();

  static unsigned long lastMainUpdate = 0;

  if (millis() - lastMainUpdate >= 1000) {
    lastMainUpdate = millis();

    writeMainStatusToHMI();
  }
}
