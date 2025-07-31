#include <Arduino.h>
#include <SPI.h>
#include <rfal_nfc.h>
#include <rfal_nfca.h>
#include <rfal_nfcb.h>
#include <rfal_nfcv.h>
#include <rfal_rf.h>
#include <rfal_rfst25r3916.h>
#include <st_errno.h>
#include "demo_ce.h"

#define RXD2 (16) // Pin for recieved data from card reader
#define TXD2 (17) // Pin for send data from card reader

// buffer to save data that already read by card reader
byte dataBuffer[128];

// Array for save UID card after parsing
uint8_t cardUID[4] = {0xAB, 0xCD, 0x00, 0x01};

// command to get Mifare data card
const byte COMMAND_READ_CSN[] = {
    0x7E, 0x00, 0x6F, 0x05, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xCA, 0x00, 0x00,
    0x00, 0x47, 0x09, 0x7E};

// command to get SEOS data card
const byte COMMAND_READ_SEOS[] = {
    0x7E, 0x00, 0x6F, 0x0D, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    0xFF, 0x70, 0x07, 0x6B, 0x07, 0xA0, 0x05, 0xA1, 0x03, 0x80, 0x01, 0x04, 0x00,
    0x68, 0xEF, 0x7E};

// Data received when the card is brought close to the reader
const byte RESPONSE_CARD_DETECTED[7] = {0x7E, 0x00, 0x50, 0x03, 0x27, 0xA0, 0x7E};

// Data received when the card is removed from the reader
const byte RESPONSE_CARD_REMOVED[] = {0x7E, 0x00, 0x50, 0x02, 0x36, 0x29, 0x7E};

void demoNotif(rfalNfcState st);
void handleDataExchange();
void restartDiscovery();
void startCardEmulation();
const char *rfalNfcState2Str(rfalNfcState st);
bool updateEmulationUID(String newUID);
ReturnCode demoTransceiveBlocking(uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt);

const int kPinMOSI = 23;
const int kPinMISO = 19;
const int kPinSCK = 18;
const int kPinSS = 5;
const int kPinIRQ = 4;
const int kPinLED = 2;

SPIClass gSPI(VSPI);
RfalRfST25R3916Class gReaderHardware(&gSPI, kPinSS, kPinIRQ);
RfalNfcClass gNFCReader(&gReaderHardware);

#define DEMO_LM_SEL_RES 0x20U
#define DEMO_LM_NFCID2_BYTE1 0x02U

static uint8_t ceNFCF_nfcid2[] = {DEMO_LM_NFCID2_BYTE1, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static uint8_t ceNFCA_SENS_RES[] = {0x04, 0x00};
static uint8_t ceNFCA_SEL_RES = DEMO_LM_SEL_RES;
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80,
                       0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

bool systemInitialized = false;
bool discoveryActive = false;
bool cardActivated = false;
bool emulationRequested = false;
unsigned long lastActivity = 0;
const unsigned long DISCOVERY_RESTART_INTERVAL = 1000;
uint8_t consecutiveErrors = 0;
const uint8_t MAX_CONSECUTIVE_ERRORS = 3;

// Default UID for emulation
static uint8_t dynamicNFCID[RFAL_LM_NFCID_LEN_04] = {0xA8, 'S', 'T', 'M'};
bool uidUpdateRequired = false;

// Timing debug
unsigned long startTimeEmu = 0;
unsigned long endTimeEmu = 0;
unsigned long emulationStartTime = 0;
unsigned long state2AchievedTime = 0;
bool timingDebugActive = false;
int lastDeviceState = -1;
bool state2Detected = false;
int state2Count = 0;
const int STATE2_TARGET_COUNT = 10;
unsigned long currentTime = 0;
bool skipHeartbeat = false;
unsigned long lastWorkerCall = 0;
const unsigned long WORKER_INTERVAL = 1;

enum CardType
{
    CARD_SEOS,
    CARD_MIFARE_CLASSIC,
    CARD_UNKNOWN
};

template <size_t N>
bool isSameData(const byte (&dataReferensi)[N])
{

    for (uint8_t i = 0; i < N; i++)
    {
        if (dataBuffer[i] != dataReferensi[i])
        {
            return false;
        }
    }
    return true;
}

uint8_t readDataFromCardReader()
{

    memset(dataBuffer, 0, sizeof(dataBuffer));
    uint8_t jumlahByte = 0;

    Serial.print("📡 DATA RECIEVED: ");

    while (Serial2.available() > 0)
    {
        const byte singleByte = Serial2.read();
        dataBuffer[jumlahByte] = singleByte;
        jumlahByte++;

        Serial.print(singleByte, HEX);
        Serial.print(" ");
    }
    Serial.println();

    return jumlahByte;
}

bool waitForResponse(unsigned long timeoutMs = 1000)
{
    unsigned long startTime = millis();

    while (!Serial2.available() && (millis() - startTime) < timeoutMs)
    {
        delay(1);
    }

    return Serial2.available() > 0;
}

String convertBytesToBinaryString(byte *arrayByte, int length)
{
    String result = "";

    for (int i = 0; i < length; i++)
    {

        for (int bit = 7; bit >= 0; bit--)
        {
            result += String(bitRead(arrayByte[i], bit));
        }
    }

    return result;
}

unsigned long convertBinaryToDecimal(String stringBiner)
{
    unsigned long result = 0;

    for (unsigned int i = 0; i < stringBiner.length(); i++)
    {
        char bit = stringBiner.charAt(i);
        if (bit == '1')
        {
            result = (result << 1) | 1;
        }
        else if (bit == '0')
        {
            result = result << 1;
        }
        else
        {
            return 0;
        }
    }

    return result;
}

void convertNumberToByteArray(unsigned long number, uint8_t *output)
{
    output[3] = (number >> 24) & 0xFF; // Byte tertinggi
    output[2] = (number >> 16) & 0xFF;
    output[1] = (number >> 8) & 0xFF;
    output[0] = number & 0xFF; // Byte terendah
}

/*
   Function to detect card type based on response
   bufferLength: number of bytes received
   isSEOSCommand: true if this is a response from a SEOS command
*/
CardType detectCardType(uint8_t bufferLength, bool isSEOSCommand)
{
    Serial.print("🔍 DETECTING CARD TYPE...");
    Serial.print(" (Buffer: ");
    Serial.print(bufferLength);
    Serial.print(" bytes, Command: ");
    Serial.println(isSEOSCommand ? "SEOS" : "CSN");

    // If this is a response from a SEOS command
    if (isSEOSCommand)
    {
        // Check if SEOS response data is valid
        if (bufferLength >= 17)
        {
            const int csnLength = dataBuffer[3];
            Serial.print("   📏 CSN length from buffer[3]: ");
            Serial.println(csnLength);

            // SEOS typically has more complex data
            if (csnLength >= 6 && bufferLength > 20)
            {
                Serial.println("   ✅ Pattern matches SEOS card");
                return CARD_SEOS;
            }
        }

        Serial.println("   ❌ Not a SEOS card or command failed");
        return CARD_UNKNOWN;
    }

    // If this is a response from a CSN command (e.g., Mifare Classic)
    else
    {
        bool found9000 = false;
        int statusPosition = -1;

        // Search for status "90 00" indicating success
        for (int i = bufferLength - 2; i >= 12; i--)
        {
            if (dataBuffer[i] == 0x90 && dataBuffer[i + 1] == 0x00)
            {
                found9000 = true;
                statusPosition = i;
                Serial.print("   📍 Status 9000 found at position: ");
                Serial.println(statusPosition);
                break;
            }
        }

        // If 9000 status is found and there's valid payload before it
        if (found9000 && statusPosition > 12)
        {
            int payloadLength = statusPosition - 12;
            Serial.print("   📏 Payload length: ");
            Serial.println(payloadLength);

            if (payloadLength > 0 && payloadLength <= 16)
            {
                Serial.println("   ✅ Pattern matches Mifare Classic card");
                return CARD_MIFARE_CLASSIC;
            }
        }

        Serial.println("   ❌ Not a Mifare Classic or command failed");
        return CARD_UNKNOWN;
    }
}

/*
   Function to process SEOS card data
   bufferLength: number of bytes received
   Return: true if successful, false if failed
*/
bool processSEOSData(uint8_t bufferLength)
{
    Serial.println("🔧 PROCESSING SEOS DATA...");

    const int csnLength = dataBuffer[3];
    if (bufferLength < 17 || csnLength < 6)
    {
        Serial.println("❌ Not enough SEOS data to process");
        return false;
    }

    // Extract specific data based on SEOS structure
    const int dataStart = 15;
    const int dataSize = 4;
    byte extractedData[dataSize] = {0};
    memcpy(extractedData, &dataBuffer[dataStart], dataSize);

    // Convert to binary string and extract the significant part
    String binaryString = convertBytesToBinaryString(extractedData, sizeof(extractedData));
    binaryString = binaryString.substring(1, binaryString.length() - 7);              // Remove start & end bits
    const String significantBits = binaryString.substring(12, binaryString.length()); // Core data

    // Convert binary to decimal
    const unsigned long decimalNumber = convertBinaryToDecimal(significantBits);
    Serial.print("🔢 SEOS Card Number (Decimal): ");
    Serial.println(decimalNumber);

    // Generate UID
    convertNumberToByteArray(decimalNumber, cardUID);

    Serial.print("🆔 SEOS UID: ");
    for (uint8_t i = 0; i < sizeof(cardUID); i++)
    {
        if (cardUID[i] < 0x10)
            Serial.print("0");
        Serial.print(cardUID[i], HEX);
    }
    Serial.println();

    return true;
}

/*
   Function to process Mifare Classic card data
   bufferLength: number of bytes received
   Return: true if successful, false if failed
*/
bool processMifareData(uint8_t bufferLength)
{
    Serial.println("🔧 PROCESSING MIFARE DATA...");

    const int payloadStart = 12;
    const int minLength = 18;

    if (bufferLength < minLength)
    {
        Serial.println("❌ Mifare response is too short");
        return false;
    }

    // Print buffer for debugging
    Serial.print("📋 Buffer for parsing: ");
    for (int i = 0; i < bufferLength; i++)
    {
        Serial.print(dataBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Find "90 00" status indicating success
    int statusPosition = -1;
    for (int i = bufferLength - 3; i >= payloadStart; i--)
    {
        if (dataBuffer[i] == 0x90 && dataBuffer[i + 1] == 0x00)
        {
            statusPosition = i;
            Serial.print("📍 Valid 9000 status at position: ");
            Serial.println(statusPosition);
            break;
        }
    }

    if (statusPosition == -1)
    {
        Serial.println("❌ Success status (9000) not found");
        return false;
    }

    int payloadLength = statusPosition - payloadStart;

    if (payloadLength <= 0 || payloadLength > 16)
    {
        Serial.print("❌ Invalid payload length: ");
        Serial.println(payloadLength);
        return false;
    }

    // Extract CSN from payload
    uint8_t csn[16];
    memcpy(csn, &dataBuffer[payloadStart], payloadLength);

    Serial.print("🏷️ Mifare CSN (HEX): ");
    for (int i = 0; i < payloadLength; i++)
    {
        if (csn[i] < 0x10)
            Serial.print("0");
        Serial.print(csn[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Show UID (first 4 bytes)
    Serial.print("🆔 Mifare UID (first 4 bytes): ");
    for (int i = 0; i < min(4, payloadLength); i++)
    {
        if (csn[i] < 0x10)
            Serial.print("0");
        Serial.print(csn[i], HEX);
    }
    Serial.println();

    // USE UID card for UID card emulation
    String uidString = "";
    for (int i = 0; i < min(4, payloadLength); i++)
    {
        if (csn[i] < 0x10)
            uidString += "0";
        uidString += String(csn[i], HEX);
    }

    Serial.printf("🔄 Converting CSN to UID string: %s\n", uidString.c_str());

    bool uidUpdated = updateEmulationUID(uidString);
    if (uidUpdated)
    {
        Serial.println("✅ UID emulation updated successfully");
    }
    else
    {
        Serial.println("❌ Failed to update UID emulation");
    }
    startCardEmulation();
    // USE UID card for UID card emulation

    // Convert entire CSN to decimal
    String binaryString = convertBytesToBinaryString(csn, payloadLength);
    unsigned long decimalNumber = convertBinaryToDecimal(binaryString);

    return true;
}

/*
   Function to try reading a card as SEOS
   Return: true if successful, false if failed
*/
bool tryReadSEOS()
{

    // Send SEOS command to card reader
    Serial2.write(COMMAND_READ_SEOS, sizeof(COMMAND_READ_SEOS));

    // Wait for response
    if (!waitForResponse(1000))
    {

        return false;
    }

    // Read the response
    uint8_t dataLength = readDataFromCardReader();
    CardType detectedType = detectCardType(dataLength, true);

    // If detected as SEOS, process the data
    if (detectedType == CARD_SEOS)
    {
        Serial.println("✅ SEOS CARD DETECTED!");
        return processSEOSData(dataLength);
    }

    return false;
}

/*
   Function to try reading a card as Mifare Classic
   Return: true if successful, false if failed
*/
bool tryReadMifare()
{

    // Send CSN command to card reader
    Serial2.write(COMMAND_READ_CSN, sizeof(COMMAND_READ_CSN));

    // Wait for response
    if (!waitForResponse(1000))
    {
        Serial.println("❌ No response to CSN command");
        return false;
    }

    // Read the response
    uint8_t dataLength = readDataFromCardReader();
    CardType detectedType = detectCardType(dataLength, false);

    // If detected as Mifare Classic, process the data
    if (detectedType == CARD_MIFARE_CLASSIC)
    {
        Serial.println("✅ MIFARE CLASSIC CARD DETECTED!");
        return processMifareData(dataLength);
    }

    return false;
}

/*
   Main function to read a card with automatic type detection
   This is the "brain" of the program that controls the reading flow
*/
void autoReadCard()
{
    Serial.println("🤖 ===== STARTING AUTOMATIC DETECTION =====");

    bool success = false;

    // STEP 1: Try reading as SEOS first
    success = tryReadSEOS();

    // STEP 2: If failed, try as Mifare Classic
    if (!success)
    {

        delay(100); // Brief delay
        success = tryReadMifare();
    }

    // STEP 3: Final result
    if (!success)
    {
        Serial.println("⚠️ CARD NOT RECOGNIZED or FAILED TO READ");
        Serial.println("   Possible causes:");
        Serial.println("   - Unsupported card");
        Serial.println("   - Damaged card");
        Serial.println("   - Connection issue");
    }

    Serial.println("🤖 ===== AUTOMATIC DETECTION COMPLETE =====\n");
}

const char *rfalNfcState2Str(rfalNfcState st)
{
    switch (st)
    {
    case RFAL_NFC_STATE_NOTINIT:
        return "NOTINIT";
    case RFAL_NFC_STATE_START_DISCOVERY:
        return "START_DISCOVERY";
    case RFAL_NFC_STATE_WAKEUP_MODE:
        return "WAKEUP_MODE";
    case RFAL_NFC_STATE_POLL_TECHDETECT:
        return "POLL_TECHDETECT";
    case RFAL_NFC_STATE_POLL_COLAVOIDANCE:
        return "POLL_COLAVOIDANCE";
    case RFAL_NFC_STATE_POLL_SELECT:
        return "POLL_SELECT";
    case RFAL_NFC_STATE_POLL_ACTIVATION:
        return "POLL_ACTIVATION";
    case RFAL_NFC_STATE_LISTEN_TECHDETECT:
        return "LISTEN_TECHDETECT";
    case RFAL_NFC_STATE_LISTEN_COLAVOIDANCE:
        return "LISTEN_COLAVOIDANCE";
    case RFAL_NFC_STATE_LISTEN_ACTIVATION:
        return "LISTEN_ACTIVATION";
    case RFAL_NFC_STATE_LISTEN_SLEEP:
        return "LISTEN_SLEEP";
    case RFAL_NFC_STATE_ACTIVATED:
        return "ACTIVATED";
    case RFAL_NFC_STATE_DATAEXCHANGE:
        return "DATAEXCHANGE";
    case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
        return "DATAEXCHANGE_DONE";
    default:
        return "UNKNOWN";
    }
}

// Convert hex string UID to byte array for NFC emulation
bool convertUIDStringToBytes(String uidString, uint8_t *uidBytes, int maxLen)
{
    uidString.trim();
    uidString.toUpperCase();

    if (uidString.length() % 2 != 0 || uidString.length() == 0)
        return false;

    int byteCount = uidString.length() / 2;

    if (byteCount > maxLen)
    {
        byteCount = maxLen;
    }

    for (int i = 0; i < byteCount; i++)
    {
        String hexByte = uidString.substring(i * 2, i * 2 + 2);
        char *endPtr;
        long value = strtol(hexByte.c_str(), &endPtr, 16);

        if (*endPtr != '\0')
            return false;

        uidBytes[i] = (uint8_t)value;
    }

    for (int i = byteCount; i < maxLen; i++)
    {
        uidBytes[i] = 0x00;
    }

    return true;
}

// Update emulation UID
bool updateEmulationUID(String newUID)
{
    if (newUID.length() == 0)
    {
        Serial.println("⚠️  Empty UID - using default");
        return false;
    }

    Serial.printf("🔄 Updating emulation UID to %s\n", newUID.c_str());

    uint8_t backupNFCID[RFAL_LM_NFCID_LEN_04];
    memcpy(backupNFCID, dynamicNFCID, RFAL_LM_NFCID_LEN_04);

    if (convertUIDStringToBytes(newUID, dynamicNFCID, RFAL_LM_NFCID_LEN_04))
    {
        bool conversionValid = false;
        for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++)
        {
            if (dynamicNFCID[i] != 0x00)
            {
                conversionValid = true;
                break;
            }
        }

        if (!conversionValid)
        {
            memcpy(dynamicNFCID, backupNFCID, RFAL_LM_NFCID_LEN_04);

            return false;
        }

        uidUpdateRequired = true;

        for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++)
        {
            Serial.printf("%02X ", dynamicNFCID[i]);
        }
        Serial.println();

        return true;
    }
    else
    {
        memcpy(dynamicNFCID, backupNFCID, RFAL_LM_NFCID_LEN_04);
        Serial.println("❌ UID conversion failed - keeping backup");
        return false;
    }
}

void startTimingDebug()
{
    emulationStartTime = millis();
    state2AchievedTime = 0;
    timingDebugActive = true;
    lastDeviceState = -1;
    state2Detected = false;
    state2Count = 0;
}

void onDeviceStateChange(int newState)
{
    if (!timingDebugActive)
        return;

    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - emulationStartTime;

    if (newState != lastDeviceState)
    {
        lastDeviceState = newState;
        if (newState != 2)
        {
            state2Count = 0;
        }
    }

    if (newState == 2)
    {
        state2Count++;

        if (state2Count == 1 && state2AchievedTime == 0)
        {
            state2AchievedTime = currentTime;
            unsigned long timeDiff = state2AchievedTime - emulationStartTime;
            Serial.printf("⏱️  Device State 2 in %lu ms\n", timeDiff);
        }

        if (state2Count >= STATE2_TARGET_COUNT && !state2Detected)
        {
            unsigned long finalTime = currentTime;
            unsigned long totalTimeDiff = finalTime - emulationStartTime;

            Serial.printf("🎯 Stabilized in %lu ms\n", totalTimeDiff);

            if (totalTimeDiff <= 1000)
            {
                Serial.println("✅ EXCELLENT: Sub-1 second");
            }
            else if (totalTimeDiff <= 2000)
            {
                Serial.println("✅ GOOD: 1-2 seconds");
            }
            else
            {
                Serial.println("⚠️  SLOW: >2 seconds");
            }

            state2Detected = true;
            timingDebugActive = false;
        }
    }
}

void startCardEmulation()
{
    if (!systemInitialized)
    {
        Serial.println("System not initialized");
        return;
    }

    Serial.println("=== STARTING CARD EMULATION ===");

    startTimingDebug();

    emulationRequested = true;

    if (discoveryActive)
    {
        Serial.println("Discovery already active");
        return;
    }

    restartDiscovery();
}

void stopCardEmulation()
{
    Serial.println("=== STOPPING CARD EMULATION ===");

    emulationRequested = false;

    if (cardActivated || discoveryActive)
    {
        ReturnCode deactErr = gNFCReader.rfalNfcDeactivate(false);
        if (deactErr != ERR_NONE)
        {
            Serial.printf("Deactivation warning: %d\n", deactErr);
        }
    }

    cardActivated = false;
    discoveryActive = false;
    digitalWrite(kPinLED, LOW);

    Serial.println("Card emulation stopped");
}

void processSerialCommand()
{
    if (Serial.available())
    {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();

        if (command == "START" || command == "R")
        {
            startCardEmulation();
        }
        else if (command == "STOP" || command == "S")
        {
            stopCardEmulation();
        }
        else if (command.startsWith("UID "))
        {
            String uidValue = command.substring(4);
            uidValue.trim();
            if (uidValue.length() > 0)
            {
                updateEmulationUID(uidValue);
                Serial.printf("UID set to: %s\n", uidValue.c_str());
            }
            else
            {
                Serial.println("Usage: UID <hex_string>");
                Serial.println("Example: UID 04184F0A");
            }
        }
        else if (command == "STATUS")
        {
            Serial.printf("System: %s\n", systemInitialized ? "OK" : "FAIL");
            Serial.printf("Discovery: %s\n", discoveryActive ? "ACTIVE" : "INACTIVE");
            Serial.printf("Card: %s\n", cardActivated ? "CONNECTED" : "IDLE");
            Serial.printf("Emulation: %s\n", emulationRequested ? "ACTIVE" : "OFF");
            Serial.printf("Current UID: ");
            for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++)
            {
                Serial.printf("%02X ", dynamicNFCID[i]);
            }
            Serial.println();
        }
        else if (command == "HELP")
        {
            Serial.println("Commands:");
            Serial.println("START or R     - Start card emulation");
            Serial.println("STOP or S      - Stop card emulation");
            Serial.println("UID <hex>      - Set UID for emulation");
            Serial.println("STATUS         - Show system status");
            Serial.println("HELP           - Show this help");
            Serial.println("");
            Serial.println("Examples:");
            Serial.println("UID 04184F0A   - Set 4-byte UID");
            Serial.println("UID 04184F0AE16E80 - Set UID (truncated to 4 bytes)");
        }
        else
        {
            Serial.println("Unknown command. Type HELP for available commands.");
        }
    }
}

void resetSystem()
{
    digitalWrite(kPinLED, LOW);
    systemInitialized = false;
    discoveryActive = false;
    cardActivated = false;
    consecutiveErrors = 0;

    gSPI.end();
    delay(200);

    gSPI.begin(kPinSCK, kPinMISO, kPinMOSI, kPinSS);
    gSPI.setFrequency(1000000);
    gSPI.setDataMode(SPI_MODE0);
    gSPI.setBitOrder(MSBFIRST);

    digitalWrite(kPinSS, HIGH);
    delay(100);
}

bool initializeNFC()
{
    ReturnCode init_err = gNFCReader.rfalNfcInitialize();

    if (init_err == ERR_NONE)
    {
        demoCeInit(ceNFCF_nfcid2);
        systemInitialized = true;
        consecutiveErrors = 0;
        return true;
    }
    else
    {
        consecutiveErrors++;
        systemInitialized = false;

        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS)
        {
            resetSystem();
            delay(1000);
        }

        return false;
    }
}

ReturnCode demoTransceiveBlocking(uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt)
{
    ReturnCode err;
    uint32_t startTime = millis();
    const uint32_t EXCHANGE_TIMEOUT = 2000;

    err = gNFCReader.rfalNfcDataExchangeStart(txBuf, txBufSize, rxData, rcvLen, fwt);

    if (err == ERR_NONE)
    {
        do
        {
            gNFCReader.rfalNfcWorker();
            err = gNFCReader.rfalNfcDataExchangeGetStatus();

            if (millis() - startTime > EXCHANGE_TIMEOUT)
            {
                return ERR_TIMEOUT;
            }

            yield();
        } while (err == ERR_BUSY);
    }

    return err;
}

void restartDiscovery()
{
    if (!systemInitialized)
        return;

    rfalNfcDiscoverParam discover_params;
    memset(&discover_params, 0, sizeof(discover_params));

    unsigned long lastIRQTime = 0;
    unsigned int irqSpamCount = 0;
    const unsigned long IRQ_RATE_LIMIT = 50; // ms between processing

    // Modify worker call in loop() (REPLACE existing worker call):
    if (emulationRequested && currentTime - lastWorkerCall >= WORKER_INTERVAL)
    {
        // Rate limit worker calls during spam
        if (currentTime - lastIRQTime > IRQ_RATE_LIMIT)
        {
            gNFCReader.rfalNfcWorker();
            lastWorkerCall = currentTime;
            lastIRQTime = currentTime;
            irqSpamCount = 0;
        }
        else
        {
            irqSpamCount++;
            if (irqSpamCount > 10)
            {
                // Skip worker during heavy spam
                delay(10);
                irqSpamCount = 0;
            }
        }
    }

    // OPTIMIZED: Faster discovery parameters
    discover_params.compMode = RFAL_COMPLIANCE_MODE_ISO; // Try EMV mode
    discover_params.devLimit = 1;
    discover_params.techs2Find = RFAL_NFC_LISTEN_TECH_A;
    discover_params.totalDuration = 25U; // REDUCED from 100U for faster response
    discover_params.wakeupEnabled = false;

    // Use current UID (either default or updated)
    ST_MEMCPY(&discover_params.lmConfigPA.nfcid, dynamicNFCID, RFAL_LM_NFCID_LEN_04);

    ST_MEMCPY(&discover_params.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN);
    discover_params.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;
    discover_params.lmConfigPA.SEL_RES = ceNFCA_SEL_RES;
    discover_params.notifyCb = demoNotif;

    Serial.printf("Starting discovery with UID: ");
    for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++)
    {
        Serial.printf("%02X ", dynamicNFCID[i]);
    }
    Serial.println();

    ReturnCode result = gNFCReader.rfalNfcDiscover(&discover_params);
    discoveryActive = true;

    // ADD: Debug discovery result
    Serial.printf("🔍 Discovery result: %d\n", result);
}

void demoNotif(rfalNfcState st)
{
    static rfalNfcState lastState = RFAL_NFC_STATE_NOTINIT;
    static unsigned long lastWaitingLog = 0;
    unsigned long currentTime = millis();

    // ONLY log state changes, bukan spam
    if (st != lastState)
    {
        // Reduce verbose logging untuk active states
        if (st == RFAL_NFC_STATE_START_DISCOVERY)
        {
            Serial.println("*** WAITING FOR READER ***");
        }
        else
        {
            Serial.printf("STATE: %s -> %s\n", rfalNfcState2Str(lastState), rfalNfcState2Str(st));
        }
        lastState = st;
    }

    // Limit *** WAITING FOR READER *** spam to every 2 seconds
    if (st == RFAL_NFC_STATE_START_DISCOVERY &&
        currentTime - lastWaitingLog > 2000)
    {
        // Don't spam - just update timestamp
        lastWaitingLog = currentTime;
    }

    lastActivity = millis();

    switch (st)
    {
    case RFAL_NFC_STATE_ACTIVATED:
        if (emulationRequested)
        {
            endTimeEmu = millis();
            float durationSeconds = (endTimeEmu - startTimeEmu) / 1000.0;
            Serial.print("Timer stopped. Duration: ");
            Serial.print(durationSeconds);
            Serial.println(" seconds");
            Serial.println("*** READER CONNECTED ***");
            digitalWrite(kPinLED, HIGH);
            cardActivated = true;
            skipHeartbeat = false;
        }
        break;

    case RFAL_NFC_STATE_DATAEXCHANGE:
        if (emulationRequested)
        {
            Serial.println("*** DATA EXCHANGE ***");
            handleDataExchange();
        }
        break;

    case RFAL_NFC_STATE_LISTEN_TECHDETECT:
        if (emulationRequested)
        {
            discoveryActive = true;
        }
        break;

    case RFAL_NFC_STATE_LISTEN_SLEEP:
        Serial.println("*** READER DISCONNECTED ***");
        digitalWrite(kPinLED, LOW);
        cardActivated = false;
        discoveryActive = false;
        skipHeartbeat = false;
        break;

    case RFAL_NFC_STATE_LISTEN_ACTIVATION:
        if (emulationRequested)
        {
            Serial.println("*** READER ACTIVATING ***");
        }
        break;

    default:
        break;
    }
}

void handleDataExchange()
{
    Serial.println("🔄 handleDataExchange() CALLED!");
    ReturnCode err;
    uint8_t *rxData;
    uint16_t *rcvLen;
    uint8_t txBuf[256];
    uint16_t txLen = 0;

    err = demoTransceiveBlocking(NULL, 0, &rxData, &rcvLen, 0);

    if (err == ERR_NONE && rcvLen && *rcvLen > 0)
    {
        Serial.printf("Received %d bytes from reader\n", *rcvLen);

        txLen = demoCeT4T(rxData, *rcvLen, txBuf, sizeof(txBuf));

        if (txLen > 0)
        {
            Serial.printf("Sending %d bytes response\n", txLen);
            err = demoTransceiveBlocking(txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE);

            if (err != ERR_NONE)
            {
                Serial.printf("Failed to send response: %d\n", err);
            }
            else
            {
                Serial.println("Response sent successfully");
            }
        }
        else
        {
            Serial.println("No response generated");
        }
    }
    else if (err != ERR_BUSY && err != ERR_NONE)
    {
        Serial.printf("Data exchange error: %d\n", err);
    }
}

void setup()
{
    delay(2000);
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial.println("========================================");
    Serial.println("ESP32 ST25R3916 Card Emulation Only");
    Serial.println("Manual Control Version");
    Serial.println("========================================");

    Serial.printf("MOSI:%d MISO:%d SCK:%d SS:%d IRQ:%s LED:%d\n",
                  kPinMOSI, kPinMISO, kPinSCK, kPinSS,
                  (kPinIRQ == -1) ? "DISABLED" : String(kPinIRQ).c_str(),
                  kPinLED);

    pinMode(kPinLED, OUTPUT);
    pinMode(kPinSS, OUTPUT);
    digitalWrite(kPinLED, LOW);
    digitalWrite(kPinSS, HIGH);

    gSPI.begin(kPinSCK, kPinMISO, kPinMOSI, kPinSS);
    gSPI.setFrequency(1000000);
    gSPI.setDataMode(SPI_MODE0);
    gSPI.setBitOrder(MSBFIRST);

    if (!initializeNFC())
    {
        Serial.println("FATAL: NFC initialization failed");
        while (1)
        {
            digitalWrite(kPinLED, HIGH);
            delay(200);
            digitalWrite(kPinLED, LOW);
            delay(200);
        }
    }

    Serial.println("========================================");
    Serial.println("SYSTEM READY");
    Serial.println("");
    Serial.println("Commands:");
    Serial.println("START or R     - Start card emulation");
    Serial.println("STOP or S      - Stop card emulation");
    Serial.println("UID <hex>      - Set UID for emulation");
    Serial.println("STATUS         - Show system status");
    Serial.println("HELP           - Show all commands");
    Serial.println("");
    Serial.println("Example: UID 04184F0A");
    Serial.println("         START");
    Serial.println("========================================");

    emulationRequested = false;
    uidUpdateRequired = false;
}

void loop()
{

    static unsigned long lastHeartbeat = 0;
    static unsigned long lastRestartCheck = 0;
 currentTime = millis();

    processSerialCommand();

    if (!skipHeartbeat && currentTime - lastHeartbeat > 15000)
    {
        Serial.printf("Status: Sys:%s Emulation:%s Card:%s\n",
                      systemInitialized ? "OK" : "FAIL",
                      emulationRequested ? "ACTIVE" : "IDLE",
                      cardActivated ? "CONNECTED" : "STANDBY");
        lastHeartbeat = currentTime;
    }

    if (!systemInitialized)
    {
        if (currentTime - lastRestartCheck > 5000)
        {
            if (initializeNFC())
            {
                Serial.println("System recovered");
            }
            lastRestartCheck = currentTime;
        }
        delay(100);
        return;
    }

    if (emulationRequested && currentTime - lastWorkerCall >= WORKER_INTERVAL)
    {
        gNFCReader.rfalNfcWorker();
        lastWorkerCall = currentTime;
    }

    if (emulationRequested)
    {
        rfalNfcState currentState = gNFCReader.rfalNfcGetState();

        if (!discoveryActive && currentState == RFAL_NFC_STATE_LISTEN_SLEEP)
        {
            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL)
            {
                Serial.println("Auto-restarting from sleep state");
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }

        if (!discoveryActive && !cardActivated &&
            currentState != RFAL_NFC_STATE_START_DISCOVERY &&
            currentState != RFAL_NFC_STATE_LISTEN_TECHDETECT &&
            currentState != RFAL_NFC_STATE_LISTEN_COLAVOIDANCE &&
            currentState != RFAL_NFC_STATE_LISTEN_ACTIVATION)
        {

            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL)
            {
                Serial.printf("Unexpected state (%s) - restarting discovery\n", rfalNfcState2Str(currentState));
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }
    }
    if (!Serial2.available() > 0)
    {
        return;
    }

    readDataFromCardReader();

    if (isSameData(RESPONSE_CARD_REMOVED))
    {
        Serial.println("👋 --- CARD REMOVED ---\n");
        stopCardEmulation();

        return;
    }

    if (isSameData(RESPONSE_CARD_DETECTED))
    {
        startTimeEmu = millis();
        Serial.println("📱 --- CARD DETECHTED---");
        delay(100);

        autoReadCard();
    }
    yield();
}