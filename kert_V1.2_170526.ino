#include <EEPROM.h>

/*
  Nextion HMI - Settings + EEPROM test
  Controllino MAXI
  Nextion: Serial2
  Baud: 115200

  SETTINGS PAGE:
  pageId = 1

  bSend:
  compId = 31
*/

#define NEXTION_BAUD 9600

#define SETTINGS_PAGE_ID 1
#define BSEND_COMP_ID   31

#define EEPROM_ADDR_SETTINGS 0
#define SETTINGS_MAGIC 0xA77A2026UL
#define SETTINGS_VERSION 1

struct HmiSettings
{
  uint32_t magic;
  uint16_t version;

  uint32_t z1Tank;
  uint32_t z2Tank;
  uint32_t cleanTank;
  uint32_t hotTemp;

  // Ezek most percek!
  uint32_t soakMin;
  uint32_t fillSec;    // valojaban perc
  uint32_t drainSec;   // valojaban perc
  uint32_t pauseSec;   // valojaban perc

  uint32_t skipStartH;
  uint32_t skipStartM;
  uint32_t skipEndH;
  uint32_t skipEndM;

  uint16_t checksum;
};

HmiSettings settings;

// =====================================================
// Nextion alap fuggvenyek
// =====================================================

void nexEnd()
{
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

void nexCmd(const char* cmd)
{
  Serial2.print(cmd);
  nexEnd();
}

uint32_t readNextionNumber(uint16_t timeoutMs = 1000)
{
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < timeoutMs)
  {
    if (Serial2.available())
    {
      byte header = Serial2.read();

      // Number return:
      // 0x71 + 4 byte little endian + FF FF FF
      if (header == 0x71)
      {
        while (Serial2.available() < 7)
        {
          if ((uint32_t)(millis() - start) > timeoutMs)
          {
            return 0xFFFFFFFF;
          }
        }

        uint32_t value = 0;

        value  = (uint32_t)Serial2.read();
        value |= (uint32_t)Serial2.read() << 8;
        value |= (uint32_t)Serial2.read() << 16;
        value |= (uint32_t)Serial2.read() << 24;

        // FF FF FF
        Serial2.read();
        Serial2.read();
        Serial2.read();

        return value;
      }
    }
  }

  return 0xFFFFFFFF;
}

uint32_t getNumber(const char* objectName)
{
  char cmd[40];

  snprintf(cmd, sizeof(cmd), "get %s.val", objectName);

  nexCmd(cmd);

  uint32_t value = readNextionNumber();

  if (value == 0xFFFFFFFF)
  {
    Serial.print("HIBA: nincs valasz: ");
    Serial.println(objectName);
  }

  delay(30);

  return value;
}

void setHmiNumber(const char* objectName, uint32_t value)
{
  char cmd[50];

  snprintf(cmd, sizeof(cmd), "%s.val=%lu", objectName, value);

  nexCmd(cmd);

  delay(30);
}

// =====================================================
// Clamp / checksum / EEPROM
// =====================================================

uint32_t clampValue(uint32_t v, uint32_t minV, uint32_t maxV)
{
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

void clampSettings()
{
  // Zona indexek: 1-2
  settings.z1Tank = clampValue(settings.z1Tank, 1, 2);
  settings.z2Tank = clampValue(settings.z2Tank, 1, 2);

  // Tartaly index: 1-3
  settings.cleanTank = clampValue(settings.cleanTank, 1, 3);

  // Homerseklet
  settings.hotTemp = clampValue(settings.hotTemp, 0, 90);

  // Idok - PERCBEN, max 24 ora
  settings.soakMin  = clampValue(settings.soakMin,  0, 1440);
  settings.fillSec  = clampValue(settings.fillSec,  0, 1440);
  settings.drainSec = clampValue(settings.drainSec, 0, 1440);
  settings.pauseSec = clampValue(settings.pauseSec, 0, 1440);

  // Skip ido
  settings.skipStartH = clampValue(settings.skipStartH, 0, 23);
  settings.skipStartM = clampValue(settings.skipStartM, 0, 59);

  settings.skipEndH = clampValue(settings.skipEndH, 0, 23);
  settings.skipEndM = clampValue(settings.skipEndM, 0, 59);
}

uint16_t calcChecksum(const HmiSettings &s)
{
  const uint8_t *p = (const uint8_t*)&s;
  uint16_t sum = 0;

  // checksum mezot nem szamoljuk bele
  for (size_t i = 0; i < sizeof(HmiSettings) - sizeof(uint16_t); i++)
  {
    sum += p[i];
  }

  return sum;
}

void setDefaultSettings()
{
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  settings.z1Tank = 1;
  settings.z2Tank = 2;
  settings.cleanTank = 3;

  settings.hotTemp = 25;

  settings.soakMin = 10;
  settings.fillSec = 5;
  settings.drainSec = 5;
  settings.pauseSec = 2;

  settings.skipStartH = 22;
  settings.skipStartM = 0;
  settings.skipEndH = 6;
  settings.skipEndM = 0;

  clampSettings();

  settings.checksum = calcChecksum(settings);
}

bool loadSettingsFromEeprom()
{
  EEPROM.get(EEPROM_ADDR_SETTINGS, settings);

  if (settings.magic != SETTINGS_MAGIC)
  {
    return false;
  }

  if (settings.version != SETTINGS_VERSION)
  {
    return false;
  }

  uint16_t storedChecksum = settings.checksum;

  settings.checksum = 0;
  uint16_t actualChecksum = calcChecksum(settings);
  settings.checksum = storedChecksum;

  if (storedChecksum != actualChecksum)
  {
    return false;
  }

  clampSettings();

  return true;
}

void saveSettingsToEeprom()
{
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  clampSettings();

  settings.checksum = 0;
  settings.checksum = calcChecksum(settings);

  EEPROM.put(EEPROM_ADDR_SETTINGS, settings);

  Serial.println("EEPROM: settings saved");
}

// =====================================================
// HMI settings olvasas / iras
// =====================================================

void readSettingsFromHmi()
{
  settings.z1Tank     = getNumber("nZ1Tank");
  settings.z2Tank     = getNumber("nZ2Tank");

  settings.cleanTank  = getNumber("nCleanTank");

  settings.hotTemp    = getNumber("nHotTemp");

  settings.soakMin    = getNumber("nSoakMin");

  settings.fillSec    = getNumber("nFillSec");
  settings.drainSec   = getNumber("nDrainSec");
  settings.pauseSec   = getNumber("nPauseSec");

  settings.skipStartH = getNumber("nSkipStartH");
  settings.skipStartM = getNumber("nSkipStartM");

  settings.skipEndH   = getNumber("nSkipEndH");
  settings.skipEndM   = getNumber("nSkipEndM");

  clampSettings();
}

void writeSettingsToHmi()
{
  setHmiNumber("nZ1Tank", settings.z1Tank);
  setHmiNumber("nZ2Tank", settings.z2Tank);

  setHmiNumber("nCleanTank", settings.cleanTank);

  setHmiNumber("nHotTemp", settings.hotTemp);

  setHmiNumber("nSoakMin", settings.soakMin);

  setHmiNumber("nFillSec", settings.fillSec);
  setHmiNumber("nDrainSec", settings.drainSec);
  setHmiNumber("nPauseSec", settings.pauseSec);

  setHmiNumber("nSkipStartH", settings.skipStartH);
  setHmiNumber("nSkipStartM", settings.skipStartM);

  setHmiNumber("nSkipEndH", settings.skipEndH);
  setHmiNumber("nSkipEndM", settings.skipEndM);
}

void printSettings()
{
  Serial.println();
  Serial.println("=========== SETTINGS ===========");

  Serial.print("Z1 Tank: ");
  Serial.println(settings.z1Tank);

  Serial.print("Z2 Tank: ");
  Serial.println(settings.z2Tank);

  Serial.print("Clean Tank: ");
  Serial.println(settings.cleanTank);

  Serial.print("Hot Temp: ");
  Serial.println(settings.hotTemp);

  Serial.print("Soak Min: ");
  Serial.println(settings.soakMin);

  Serial.print("Fill Min: ");
  Serial.println(settings.fillSec);

  Serial.print("Drain Min: ");
  Serial.println(settings.drainSec);

  Serial.print("Pause Min: ");
  Serial.println(settings.pauseSec);

  Serial.print("Skip Start: ");
  Serial.print(settings.skipStartH);
  Serial.print(":");
  if (settings.skipStartM < 10) Serial.print("0");
  Serial.println(settings.skipStartM);

  Serial.print("Skip End: ");
  Serial.print(settings.skipEndH);
  Serial.print(":");
  if (settings.skipEndM < 10) Serial.print("0");
  Serial.println(settings.skipEndM);

  Serial.println("================================");
  Serial.println();
}

// =====================================================
// Setup / loop
// =====================================================

void setup()
{
  Serial.begin(115200);
  Serial2.begin(NEXTION_BAUD);

  delay(2000);

  Serial.println();
  Serial.println("Nextion settings + EEPROM test indul...");

  if (!loadSettingsFromEeprom())
  {
    Serial.println("EEPROM: nincs ervenyes adat, default beallitasok");
    setDefaultSettings();
    saveSettingsToEeprom();
  }
  else
  {
    Serial.println("EEPROM: settings loaded");
  }

  nexCmd("page settings");
  delay(500);

  writeSettingsToHmi();

  printSettings();

  Serial.println("Nyomd meg a SEND gombot.");
}

void loop()
{
  static byte buffer[10];
  static byte idx = 0;

  while (Serial2.available())
  {
    byte b = Serial2.read();

    buffer[idx++] = b;

    if (idx >= sizeof(buffer))
    {
      idx = 0;
    }

    // Touch event:
    // 0x65 page comp event FF FF FF

    if (idx >= 7 &&
        buffer[idx - 7] == 0x65 &&
        buffer[idx - 3] == 0xFF &&
        buffer[idx - 2] == 0xFF &&
        buffer[idx - 1] == 0xFF)
    {
      byte pageId = buffer[idx - 6];
      byte compId = buffer[idx - 5];
      byte eventType = buffer[idx - 4];

      Serial.print("Touch event - page=");
      Serial.print(pageId);

      Serial.print(" comp=");
      Serial.print(compId);

      Serial.print(" event=");
      Serial.println(eventType);

      // bSend pressed
      if (pageId == SETTINGS_PAGE_ID &&
          compId == BSEND_COMP_ID &&
          eventType == 1)
      {
        Serial.println();
        Serial.println("SEND pressed -> settings olvasasa");

        readSettingsFromHmi();

        saveSettingsToEeprom();

        writeSettingsToHmi();

        printSettings();

        Serial.println("Settings elmentve EEPROM-ba es visszairva HMI-re.");
      }

      idx = 0;
    }
  }
}