#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <ArduinoJson.h>
#include <NdefMessage.h>
#include <NdefRecord.h>


#define RST_PIN 22

MFRC522DriverPinSimple ss_pin1(5);  // Reader 1 SS
MFRC522DriverPinSimple ss_pin2(4);  // Reader 2 SS

MFRC522DriverSPI driver1{ss_pin1};
MFRC522DriverSPI driver2{ss_pin2};

MFRC522 mfrc1{driver1};
MFRC522 mfrc2{driver2};

MFRC522::MIFARE_Key keyA;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);  // Shared RST held HIGH

  byte keyVal[6] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};
  memcpy(keyA.keyByte, keyVal, 6);

  SPI.begin(18, 19, 23);  // ESP32 default: SCK=18, MISO=19, MOSI=23

  mfrc1.PCD_Init();
  mfrc2.PCD_Init();

  Serial.println("Reader 1:");
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc1, Serial);
  Serial.println("Reader 2:");
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc2, Serial);
  Serial.println(F("System ready. Present tag to one reader."));
}

void loop() {
  // Only one reader active per loop
  if (mfrc1.PICC_IsNewCardPresent() && mfrc1.PICC_ReadCardSerial()) {
    handleReader(mfrc1, 1);
    return;
  }

  if (mfrc2.PICC_IsNewCardPresent() && mfrc2.PICC_ReadCardSerial()) {
    handleReader(mfrc2, 2);
    return;
  }
if (Serial.available()) {
  char cmd = Serial.read();
  if (cmd == '1') {
    Serial.println("[WRITE] Write triggered by serial command");
    writeNewUsageLog();
  }
}

  delay(100);
}

void handleReader(MFRC522& reader, int readerNum) {
  const int MAX_BLOCKS = 64;
  String raw = "";
  bool blockValid[MAX_BLOCKS] = {false};
  byte blockData[MAX_BLOCKS][16];  // Final merged data

  reader.PCD_Reset();
  reader.PCD_Init();

  if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) {
    Serial.printf("[Reader %d] Tag lost before reading\n", readerNum);
    return;
  }

  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3) continue;  // Skip trailer blocks

    byte reads[3][16];
    bool success[3] = {false, false, false};

    byte sector = block / 4;
    byte trailer = sector * 4 + 3;

    for (int r = 0; r < 3; r++) {
      auto auth = reader.PCD_Authenticate(
        MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A,
        trailer,
        &keyA,
        &(reader.uid)
      );
      if (auth != MFRC522Constants::STATUS_OK) continue;

      byte buffer[18];
      byte size = sizeof(buffer);
      auto status = reader.MIFARE_Read(block, buffer, &size);

      if (status == MFRC522Constants::STATUS_OK && size >= 16) {
        memcpy(reads[r], buffer, 16);
        success[r] = true;
      }
    }

    // Compare 3 reads, accept if at least 2 match
    bool accepted = false;
    if (success[0] && success[1] && memcmp(reads[0], reads[1], 16) == 0) {
      memcpy(blockData[block], reads[0], 16);
      accepted = true;
    } else if (success[0] && success[2] && memcmp(reads[0], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[0], 16);
      accepted = true;
    } else if (success[1] && success[2] && memcmp(reads[1], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[1], 16);
      accepted = true;
    }

    if (accepted) {
      blockValid[block] = true;
      Serial.printf("[Reader %d] Block %d HEX: ", readerNum, block);
      for (byte i = 0; i < 16; i++) {
        Serial.printf("%02X ", blockData[block][i]);
      }
      Serial.println();
    } else {
      Serial.printf("[Reader %d] Block %d failed validation.\n", readerNum, block);
    }
  }

  // Reconstruct raw
  raw = "";
  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3 || !blockValid[block]) continue;
    for (byte i = 0; i < 16; i++) raw += (char)blockData[block][i];
  }

  Serial.printf("\n[Reader %d] Final RAW NDEF (len: %d bytes):\n", readerNum, raw.length());
  Serial.println(raw);

  // Extract and verify JSON
  String cleanJson = extractJsonFromNdefText(raw);
  Serial.println("[DEBUG] Extracted JSON:");
  Serial.println(cleanJson);

  bool validStart = cleanJson.startsWith("{");
  bool validEnd = cleanJson.endsWith("}");
  bool hasSn = cleanJson.indexOf("\"sn\"") > 0;
  bool hasCc = cleanJson.indexOf("\"cc\"") > 0;
  bool hasU = cleanJson.indexOf("\"u\"") > 0;

  Serial.printf("[DEBUG] Starts with '{': %s\n", validStart ? "YES" : "NO");
  Serial.printf("[DEBUG] Ends with '}': %s\n", validEnd ? "YES" : "NO");
  Serial.printf("[DEBUG] Contains 'sn': %s\n", hasSn ? "YES" : "NO");
  Serial.printf("[DEBUG] Contains 'cc': %s\n", hasCc ? "YES" : "NO");
  Serial.printf("[DEBUG] Contains 'u': %s\n", hasU ? "YES" : "NO");

  if (validStart && validEnd && hasSn && hasCc && hasU) {
    Serial.printf("\n----- Reader %d CLEANED JSON -----\n", readerNum);
    Serial.println(cleanJson);
    Serial.println(F("----------------------------------"));
    printParsedBatteryJson(cleanJson);
  } else {
    Serial.printf("[Reader %d] JSON failed validation.\n", readerNum);
  }

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
  delay(500);
}


// Reuse your existing JSON parsing code...
// (omitted for brevity but you can paste your `printParsedBatteryJson`, `extractInt`, and `extractString` here)


void printParsedBatteryJson(const String& json) {
  Serial.println(F("======= Battery Info ======="));

  int snIndex = json.indexOf("\"sn\":\"");
  if (snIndex != -1) {
    String sn = json.substring(snIndex + 6, json.indexOf("\"", snIndex + 6));
    Serial.print(F("Serial Number: ")); Serial.println(sn);
  }

  int fuIndex = json.indexOf("\"fu\":\"");
  if (fuIndex != -1) {
    String fu = json.substring(fuIndex + 6, json.indexOf("\"", fuIndex + 6));
    Serial.print(F("First Use: ")); Serial.println(fu);
  }

  int ccIndex = json.indexOf("\"cc\":");
  if (ccIndex != -1) {
    int start = ccIndex + 5;
    int end = json.indexOf(",", start);
    if (end == -1) end = json.indexOf("}", start);
    if (end != -1) {
      String cc = json.substring(start, end);
      Serial.print(F("Cycle Count: ")); Serial.println(cc);
    } else {
      Serial.println(F("Cycle Count: Not found"));
    }
  }

  int nIndex = json.indexOf("\"n\":");
  if (nIndex != -1) {
    int note = json.substring(nIndex + 4, json.indexOf(",", nIndex)).toInt();
    Serial.print(F("Note Type: "));
    switch (note) {
      case 0: Serial.println(F("Normal")); break;
      case 1: Serial.println(F("Practice Only")); break;
      case 2: Serial.println(F("Scrap")); break;
      case 3: Serial.println(F("Other")); break;
      default: Serial.println(F("Unknown")); break;
    }
  }

  int uIndex = json.indexOf("\"u\":[");
  if (uIndex != -1) {
    Serial.println(F("--- Usage Log ---"));
    int pos = uIndex;
    while (true) {
      int entryStart = json.indexOf("{", pos);
      int entryEnd = json.indexOf("}", entryStart);
      if (entryStart == -1 || entryEnd == -1) break;

      String entry = json.substring(entryStart, entryEnd + 1);
      int id = extractInt(entry, "\"i\":");
      String time = extractString(entry, "\"t\":\"");
      int device = extractInt(entry, "\"d\":");

      Serial.print(F("  #")); Serial.print(id);
      Serial.print(F(" at ")); Serial.print(time);
      Serial.print(F(" on "));
      Serial.println(device == 1 ? "Robot" : "Charger");

      pos = entryEnd + 1;
    }
  }

  Serial.println(F("============================\n"));
}

int extractInt(const String& src, const char* key) {
  int i = src.indexOf(key);
  if (i == -1) return -1;
  return src.substring(i + strlen(key), src.indexOf(",", i + strlen(key))).toInt();
}

String extractString(const String& src, const char* key) {
  int i = src.indexOf(key);
  if (i == -1) return "";
  int start = i + strlen(key);
  int end = src.indexOf("\"", start);
  return src.substring(start, end);
}


String extractJsonFromNdefText(const String& raw) {
  String cleaned = "";

  // Keep only printable ASCII characters
  for (int i = 0; i < raw.length(); i++) {
    char c = raw[i];
    if (c >= 32 && c <= 126) {
      cleaned += c;
    }
  }

  // Find the first valid JSON object using brace matching
  int start = cleaned.indexOf('{');
  if (start == -1) return "";

  int depth = 0;
  for (int i = start; i < cleaned.length(); i++) {
    if (cleaned[i] == '{') depth++;
    else if (cleaned[i] == '}') depth--;

    if (depth == 0) {
      // Return the first full JSON block
      return cleaned.substring(start, i + 1);
    }
  }

  Serial.println("[NDEF] JSON braces not balanced");
  return "";
}



void writeNewUsageLog() {
  const int MAX_BLOCKS = 64;
  const int MAX_LOGS = 14;
  MFRC522& reader = mfrc1;

  reader.PCD_Reset();
  reader.PCD_Init();

  if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) {
    Serial.println("[WRITE] No tag present");
    return;
  }

  byte blockData[MAX_BLOCKS][16];
  bool blockValid[MAX_BLOCKS] = {false};
  String raw = "";

  // Step 1: Read and reconstruct JSON
  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3) continue;
    byte trailer = (block / 4) * 4 + 3;
    byte reads[3][16];
    bool success[3] = {false};

    for (int r = 0; r < 3; r++) {
      if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK)
        continue;
      byte buffer[18]; byte size = sizeof(buffer);
      if (reader.MIFARE_Read(block, buffer, &size) == MFRC522Constants::STATUS_OK && size >= 16) {
        memcpy(reads[r], buffer, 16);
        success[r] = true;
      }
    }

    bool accepted = false;
    if (success[0] && success[1] && memcmp(reads[0], reads[1], 16) == 0) {
      memcpy(blockData[block], reads[0], 16); accepted = true;
    } else if (success[0] && success[2] && memcmp(reads[0], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[0], 16); accepted = true;
    } else if (success[1] && success[2] && memcmp(reads[1], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[1], 16); accepted = true;
    }

    if (accepted) blockValid[block] = true;
  }

  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3 || !blockValid[block]) continue;
    for (byte i = 0; i < 16; i++) raw += (char)blockData[block][i];
  }

  String cleanJson = extractJsonFromNdefText(raw);
  if (cleanJson == "") {
    Serial.println("[WRITE] Invalid JSON. Abort.");
    return;
  }

  // Step 2: Parse and modify JSON
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, cleanJson)) {
    Serial.println("[WRITE] JSON parse failed");
    return;
  }

  JsonArray logs = doc["u"].as<JsonArray>();
  if (!logs) {
    Serial.println("[WRITE] Usage log array invalid");
    return;
  }

  int maxId = 0;
  while (logs.size() > MAX_LOGS - 1) {
    int minId = INT_MAX;
    size_t minIndex = 0;
    for (size_t i = 0; i < logs.size(); i++) {
      int id = logs[i]["i"] | 0;
      if (id < minId) {
        minId = id;
        minIndex = i;
      }
    }
    logs.remove(minIndex);
  }

  for (JsonObject obj : logs) {
    int id = obj["i"] | 0;
    if (id > maxId) maxId = id;
  }

  JsonObject newEntry = logs.createNestedObject();
  newEntry["i"] = maxId + 1;
  newEntry["t"] = "0000000000";
  newEntry["d"] = 1;
  newEntry["e"] = 0;
  newEntry["v"] = 0;

  String newJson;
  serializeJson(doc, newJson);
  Serial.println("[DEBUG] JSON:");
  Serial.println(newJson);

  // Step 3: NDEF encode
  NdefMessage ndef;
  NdefRecord record = NdefRecord();
  record.setTnf(TNF_WELL_KNOWN);
  const char* typeStr = "T";
  record.setType((const byte*)typeStr, 1);
  String lang = "en";
  String fullPayload = (char)lang.length() + lang + newJson;
  record.setPayload((const byte*)fullPayload.c_str(), fullPayload.length());
  ndef.addRecord(record);

  int msgLen = ndef.getEncodedSize();
  uint8_t buffer[msgLen + 6];  // for TLV + terminator + pad
  int totalLen = 0;

  if (msgLen < 0xFF) {
    buffer[0] = 0x03;           // NDEF TLV
    buffer[1] = msgLen;
    ndef.encode(&buffer[2]);
    totalLen = msgLen + 2;
  } else {
    buffer[0] = 0x03;
    buffer[1] = 0xFF;
    buffer[2] = (msgLen >> 8) & 0xFF;
    buffer[3] = msgLen & 0xFF;
    ndef.encode(&buffer[4]);
    totalLen = msgLen + 4;
  }

  if (totalLen < 752) buffer[totalLen++] = 0xFE;  // TLV terminator

  while (totalLen % 16 != 0) buffer[totalLen++] = 0x00;

  // Step 4: Write NDEF to blocks
  int block = 4;
  for (int i = 0; i < totalLen; i += 16) {
    if (block % 4 == 3) block++;  // skip trailer
    byte trailer = (block / 4) * 4 + 3;
    if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Auth fail block %d\n", block);
      block++;
      continue;
    }
    if (reader.MIFARE_Write(block, &buffer[i], 16) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Failed block %d\n", block);
    } else {
      Serial.printf("[WRITE] Wrote block %d\n", block);
    }
    block++;
  }

  // Step 5: Pad rest of tag with 0x00
  for (; block < 64; block++) {
    if (block % 4 == 3) continue;
    byte trailer = (block / 4) * 4 + 3;
    if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK) {

      Serial.printf("[WRITE] Auth fail on pad block %d\n", block);
      continue;
    }
    byte blank[16] = {0};
    if (reader.MIFARE_Write(block, blank, 16) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Pad fail block %d\n", block);
    } else {
      Serial.printf("[WRITE] Padded block %d with 0x00\n", block);
    }
  }

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
  Serial.println("[WRITE] Write complete.");
}
