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

void demoNotif(rfalNfcState st);
void handleDataExchange();
void restartDiscovery();
const char* rfalNfcState2Str(rfalNfcState st);
ReturnCode demoTransceiveBlocking(uint8_t* txBuf, uint16_t txBufSize, uint8_t** rxData, uint16_t** rcvLen, uint32_t fwt);

#define RXD2 16
#define TXD2 17

const byte COMMAND_READ_CSN[] = {
    0x7E, 0x00, 0x6F, 0x05, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xCA, 0x00, 0x00,
    0x00, 0x47, 0x09, 0x7E
};

const int kPinMOSI = 23;
const int kPinMISO = 19;
const int kPinSCK = 18;
const int kPinSS = 5;
const int kPinIRQ = -1;
const int kPinLED = 2;

SPIClass gSPI(VSPI);
RfalRfST25R3916Class gReaderHardware(&gSPI, kPinSS, kPinIRQ);
RfalNfcClass gNFCReader(&gReaderHardware);

#define DEMO_LM_SEL_RES 0x20U
#define DEMO_LM_NFCID2_BYTE1 0x02U

static uint8_t ceNFCF_nfcid2[] = {DEMO_LM_NFCID2_BYTE1, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static uint8_t ceNFCA_SENS_RES[] = {0x04, 0x00};
static uint8_t ceNFCA_NFCID[] = {0x08, 'S', 'T', 'M'};
static uint8_t ceNFCA_SEL_RES = DEMO_LM_SEL_RES;
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 
                       0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

bool systemInitialized = false;
bool discoveryActive = false;
bool cardActivated = false;
bool emulationRequested = false;
unsigned long lastActivity = 0;
unsigned long cardEmulationStartTime = 0;
const unsigned long DISCOVERY_RESTART_INTERVAL = 1000;
uint8_t consecutiveErrors = 0;
const uint8_t MAX_CONSECUTIVE_ERRORS = 3;

byte responseBuffer[128];
bool isCardPresent = false;
String currentCardUID = "";
unsigned long lastPollTime = 0;
const unsigned long POLL_INTERVAL = 300;
bool cardReadingEnabled = true;

String detectedCardUID = "";
bool autoEmulationTriggered = false;
const unsigned long AUTO_STOP_TIMEOUT = 1500;
unsigned long emulationAutoStartTime = 0;
bool cardRemovedDuringEmulation = false;
unsigned long lastCardPresentTime = 0;
const unsigned long CARD_REMOVAL_TIMEOUT = 1000;

static uint8_t dynamicNFCID[RFAL_LM_NFCID_LEN_04] = {0x08, 'S', 'T', 'M'};
bool uidUpdateRequired = false;

unsigned long emulationStartTime = 0;
unsigned long state2AchievedTime = 0;
bool timingDebugActive = false;
int lastDeviceState = -1;
bool state2Detected = false;
int state2Count = 0;
const int STATE2_TARGET_COUNT = 10;

bool skipHeartbeat = false;
unsigned long lastWorkerCall = 0;
const unsigned long WORKER_INTERVAL = 1;

// PHASE 2.6: UID Validation Variables
String lastValidUID = "";
String previousValidUID = "";
uint8_t uidValidationFailCount = 0;
const uint8_t MAX_UID_VALIDATION_FAILS = 3;
bool preserveLastUID = false;
unsigned long lastValidUIDTime = 0;
const unsigned long UID_VALIDITY_TIMEOUT = 5000;

// PHASE 2.6: Enhanced Card Removal Detection
uint8_t consecutiveInvalidResponses = 0;
const uint8_t MAX_INVALID_RESPONSES = 2;
bool cardRemovalConfirmed = false;
unsigned long cardRemovalStartTime = 0;
const unsigned long CARD_REMOVAL_CONFIRM_TIME = 500;
uint8_t validResponseCount = 0;
const uint8_t MIN_VALID_RESPONSES_FOR_PRESENCE = 2;

const char* rfalNfcState2Str(rfalNfcState st) {
    switch (st) {
        case RFAL_NFC_STATE_NOTINIT: return "NOTINIT";
        case RFAL_NFC_STATE_START_DISCOVERY: return "START_DISCOVERY";
        case RFAL_NFC_STATE_WAKEUP_MODE: return "WAKEUP_MODE";
        case RFAL_NFC_STATE_POLL_TECHDETECT: return "POLL_TECHDETECT";
        case RFAL_NFC_STATE_POLL_COLAVOIDANCE: return "POLL_COLAVOIDANCE";
        case RFAL_NFC_STATE_POLL_SELECT: return "POLL_SELECT";
        case RFAL_NFC_STATE_POLL_ACTIVATION: return "POLL_ACTIVATION";
        case RFAL_NFC_STATE_LISTEN_TECHDETECT: return "LISTEN_TECHDETECT";
        case RFAL_NFC_STATE_LISTEN_COLAVOIDANCE: return "LISTEN_COLAVOIDANCE";
        case RFAL_NFC_STATE_LISTEN_ACTIVATION: return "LISTEN_ACTIVATION";
        case RFAL_NFC_STATE_LISTEN_SLEEP: return "LISTEN_SLEEP";
        case RFAL_NFC_STATE_ACTIVATED: return "ACTIVATED";
        case RFAL_NFC_STATE_DATAEXCHANGE: return "DATAEXCHANGE";
        case RFAL_NFC_STATE_DATAEXCHANGE_DONE: return "DATAEXCHANGE_DONE";
        default: return "UNKNOWN";
    }
}

// PHASE 2.6: UID Validation
bool isValidUIDString(String uidString) {
    uidString.trim();
    uidString.toUpperCase();
    
    if (uidString.length() < 8 || uidString.length() > 32) return false;
    if (uidString.length() % 2 != 0) return false;
    
    bool allZeros = true;
    bool allFs = true;
    for (int i = 0; i < uidString.length(); i++) {
        char c = uidString.charAt(i);
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
        if (c != '0') allZeros = false;
        if (c != 'F') allFs = false;
    }
    
    if (allZeros || allFs) return false;
    
    if (uidString.length() >= 4) {
        String firstByte = uidString.substring(0, 2);
        bool isRepeating = true;
        for (int i = 2; i < uidString.length(); i += 2) {
            if (i + 1 < uidString.length()) {
                String currentByte = uidString.substring(i, i + 2);
                if (currentByte != firstByte) {
                    isRepeating = false;
                    break;
                }
            }
        }
        if (isRepeating && uidString.length() > 4) return false;
    }
    
    return true;
}

bool isValidHIDResponse(byte *data, int dataLength) {
    if (dataLength < 18) return false;
    if (data[0] != 0x7E) return false;
    
    bool hasValidFrame = false;
    for (int i = dataLength - 4; i < dataLength; i++) {
        if (data[i] == 0x7E) {
            hasValidFrame = true;
            break;
        }
    }
    if (!hasValidFrame) return false;
    
    for (int i = dataLength - 3; i >= 12; i--) {
        if (data[i] == 0x90 && data[i + 1] == 0x00) {
            return true;
        }
    }
    return false;
}

String extractValidatedCardUID(byte *data, int dataLength) {
    const int PAYLOAD_START_OFFSET = 12;
    
    if (!isValidHIDResponse(data, dataLength)) return "";
    
    int statusPosition = -1;
    for (int i = dataLength - 3; i >= PAYLOAD_START_OFFSET; i--) {
        if (data[i] == 0x90 && data[i + 1] == 0x00) {
            statusPosition = i;
            break;
        }
    }
    
    if (statusPosition == -1) return "";
    
    int payloadLength = statusPosition - PAYLOAD_START_OFFSET;
    if (payloadLength <= 0 || payloadLength > 16) return "";
    
    String cardUID = "";
    for (int i = 0; i < payloadLength; i++) {
        byte currentByte = data[PAYLOAD_START_OFFSET + i];
        if (currentByte < 0x10) cardUID += "0";
        cardUID += String(currentByte, HEX);
    }
    
    cardUID.toUpperCase();
    
    if (!isValidUIDString(cardUID)) return "";
    
    return cardUID;
}

String getReliableUID(String newUID) {
    unsigned long currentTime = millis();
    
    if (newUID.length() > 0 && isValidUIDString(newUID)) {
        uidValidationFailCount = 0;
        validResponseCount++;
        
        if (newUID != lastValidUID) {
            previousValidUID = lastValidUID;
            lastValidUID = newUID;
            lastValidUIDTime = currentTime;
        }
        
        preserveLastUID = false;
        consecutiveInvalidResponses = 0;
        cardRemovalConfirmed = false;
        
        return newUID;
    } else {
        uidValidationFailCount++;
        consecutiveInvalidResponses++;
        validResponseCount = 0;
        
        if (uidValidationFailCount >= MAX_UID_VALIDATION_FAILS) {
            if (lastValidUID.length() > 0 && 
                (currentTime - lastValidUIDTime) < UID_VALIDITY_TIMEOUT) {
                
                preserveLastUID = true;
                return lastValidUID;
            } else {
                preserveLastUID = false;
                return "";
            }
        }
        
        if (lastValidUID.length() > 0 && (currentTime - lastValidUIDTime) < UID_VALIDITY_TIMEOUT) {
            return lastValidUID;
        }
        
        return "";
    }
}

bool detectCardRemoval() {
    unsigned long currentTime = millis();
    
    if (consecutiveInvalidResponses >= MAX_INVALID_RESPONSES) {
        if (!cardRemovalConfirmed) {
            if (cardRemovalStartTime == 0) {
                cardRemovalStartTime = currentTime;
                return false;
            } else if (currentTime - cardRemovalStartTime >= CARD_REMOVAL_CONFIRM_TIME) {
                cardRemovalConfirmed = true;
                return true;
            }
        } else {
            return true;
        }
    }
    
    if (validResponseCount < MIN_VALID_RESPONSES_FOR_PRESENCE && 
        (currentTime - lastCardPresentTime) > CARD_REMOVAL_TIMEOUT) {
        
        if (!cardRemovalConfirmed) {
            cardRemovalConfirmed = true;
        }
        return true;
    }
    
    if (uidValidationFailCount >= MAX_UID_VALIDATION_FAILS && 
        (currentTime - lastValidUIDTime) > UID_VALIDITY_TIMEOUT) {
        
        if (!cardRemovalConfirmed) {
            cardRemovalConfirmed = true;
        }
        return true;
    }
    
    return false;
}

void resetCardDetectionState() {
    consecutiveInvalidResponses = 0;
    cardRemovalConfirmed = false;
    cardRemovalStartTime = 0;
    uidValidationFailCount = 0;
    validResponseCount = 0;
    preserveLastUID = false;
}

bool convertUIDStringToBytes(String uidString, uint8_t* uidBytes, int maxLen) {
    uidString.trim();
    uidString.toUpperCase();
    
    if (uidString.length() % 2 != 0 || uidString.length() == 0) return false;
    
    int byteCount = uidString.length() / 2;
    
    if (byteCount > maxLen) {
        byteCount = maxLen;
    }
    
    for (int i = 0; i < byteCount; i++) {
        String hexByte = uidString.substring(i * 2, i * 2 + 2);
        char* endPtr;
        long value = strtol(hexByte.c_str(), &endPtr, 16);
        
        if (*endPtr != '\0') return false;
        
        uidBytes[i] = (uint8_t)value;
    }
    
    for (int i = byteCount; i < maxLen; i++) {
        uidBytes[i] = 0x00;
    }
    
    return true;
}

bool updateEmulationUIDV26(String detectedUID) {
    if (detectedUID.length() == 0) return false;
    
    if (!isValidUIDString(detectedUID)) {
        Serial.printf("❌ UID validation failed: %s\n", detectedUID.c_str());
        return false;
    }
    
    uint8_t backupNFCID[RFAL_LM_NFCID_LEN_04];
    memcpy(backupNFCID, dynamicNFCID, RFAL_LM_NFCID_LEN_04);
    
    if (convertUIDStringToBytes(detectedUID, dynamicNFCID, RFAL_LM_NFCID_LEN_04)) {
        bool conversionValid = false;
        for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++) {
            if (dynamicNFCID[i] != 0x00) {
                conversionValid = true;
                break;
            }
        }
        
        if (!conversionValid) {
            memcpy(dynamicNFCID, backupNFCID, RFAL_LM_NFCID_LEN_04);
            return false;
        }
        
        uidUpdateRequired = true;
        Serial.printf("✅ UID updated: %s\n", detectedUID.c_str());
        return true;
    } else {
        memcpy(dynamicNFCID, backupNFCID, RFAL_LM_NFCID_LEN_04);
        return false;
    }
}

void startTimingDebug() {
    emulationStartTime = millis();
    state2AchievedTime = 0;
    timingDebugActive = true;
    lastDeviceState = -1;
    state2Detected = false;
    state2Count = 0;
}

void onDeviceStateChange(int newState) {
    if (!timingDebugActive) return;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - emulationStartTime;
    
    if (newState != lastDeviceState) {
        lastDeviceState = newState;
        if (newState != 2) {
            state2Count = 0;
        }
    }
    
    if (newState == 2) {
        state2Count++;
        
        if (state2Count == 1 && state2AchievedTime == 0) {
            state2AchievedTime = currentTime;
            unsigned long timeDiff = state2AchievedTime - emulationStartTime;
            Serial.printf("⏱️  Device State 2 in %lu ms\n", timeDiff);
        }
        
        if (state2Count >= STATE2_TARGET_COUNT && !state2Detected) {
            unsigned long finalTime = currentTime;
            unsigned long totalTimeDiff = finalTime - emulationStartTime;
            
            Serial.printf("🎯 Stabilized in %lu ms\n", totalTimeDiff);
            
            if (totalTimeDiff <= 1000) {
                Serial.println("✅ EXCELLENT: Sub-1 second");
            } else if (totalTimeDiff <= 2000) {
                Serial.println("✅ GOOD: 1-2 seconds");
            } else {
                Serial.println("⚠️  SLOW: >2 seconds");
            }
            
            state2Detected = true;
            timingDebugActive = false;
        }
    }
}

void autoStopCardEmulation(String reason) {
    if (!emulationRequested) return;
    
    Serial.printf("🛑 Auto-stopping: %s\n", reason.c_str());
    
    emulationRequested = false;
    
    if (cardActivated || discoveryActive) {
        ReturnCode deactErr = gNFCReader.rfalNfcDeactivate(false);
        if (deactErr != ERR_NONE) {
            Serial.printf("Deactivation warning: %d\n", deactErr);
        }
    }
    
    cardActivated = false;
    discoveryActive = false;
    digitalWrite(kPinLED, LOW);
    
    autoEmulationTriggered = false;
    cardRemovedDuringEmulation = false;
    emulationAutoStartTime = 0;
}

void startCardEmulation() {
    if (!systemInitialized) return;
    
    Serial.println("=== STARTING EMULATION ===");
    
    startTimingDebug();
    
    emulationRequested = true;
    emulationAutoStartTime = millis();
    cardRemovedDuringEmulation = false;
    
    if (discoveryActive) {
        return;
    }
    
    restartDiscovery();
}

void processReaderResponseV26() {
    unsigned long currentTime = millis();
    
    if (emulationRequested || cardActivated) {
        while (Serial2.available()) Serial2.read();
        return;
    }

    memset(responseBuffer, 0, sizeof(responseBuffer));
    uint8_t totalBytesReceived = 0;

    delay(15);

    while (Serial2.available() > 0 && totalBytesReceived < sizeof(responseBuffer) - 1) {
        responseBuffer[totalBytesReceived] = Serial2.read();
        totalBytesReceived++;
    }

    if (totalBytesReceived == 0) {
        consecutiveInvalidResponses++;
        validResponseCount = 0;
        
        if (isCardPresent && detectCardRemoval()) {
            Serial.println("🚨 Card removed (no response)");
            isCardPresent = false;
            currentCardUID = "";
            detectedCardUID = "";
            
            if (autoEmulationTriggered && emulationRequested) {
                cardRemovedDuringEmulation = true;
            }
            
            autoEmulationTriggered = false;
            resetCardDetectionState();
        }
        return;
    }

    String extractedUID = extractValidatedCardUID(responseBuffer, totalBytesReceived);
    String reliableUID = getReliableUID(extractedUID);

    if (reliableUID.length() > 0) {
        lastCardPresentTime = currentTime;
        
        if (!isCardPresent || reliableUID != currentCardUID) {
            Serial.printf("Card: %s%s\n", 
                         reliableUID.c_str(),
                         preserveLastUID ? " (preserved)" : "");
            
            isCardPresent = true;
            currentCardUID = reliableUID;
            detectedCardUID = reliableUID;
            
            resetCardDetectionState();
            updateEmulationUIDV26(reliableUID);
            
            if (!autoEmulationTriggered && !emulationRequested) {
                Serial.println("🚀 Auto-triggering emulation...");
                autoEmulationTriggered = true;
                cardRemovedDuringEmulation = false;
                startCardEmulation();
            }
        }
    } else {
        if (isCardPresent && detectCardRemoval()) {
            Serial.println("🚨 Card removed (validation failed)");
            isCardPresent = false;
            currentCardUID = "";
            detectedCardUID = "";
            
            if (autoEmulationTriggered && emulationRequested) {
                cardRemovedDuringEmulation = true;
            }
            
            autoEmulationTriggered = false;
            resetCardDetectionState();
        }
    }
}

void stopCardEmulation() {
    Serial.println("=== STOPPING EMULATION ===");
    
    emulationRequested = false;
    
    if (cardActivated || discoveryActive) {
        ReturnCode deactErr = gNFCReader.rfalNfcDeactivate(false);
        if (deactErr != ERR_NONE) {
            Serial.printf("Deactivation warning: %d\n", deactErr);
        }
    }
    
    cardActivated = false;
    discoveryActive = false;
    digitalWrite(kPinLED, LOW);
    
    autoEmulationTriggered = false;
    cardRemovedDuringEmulation = false;
    emulationAutoStartTime = 0;
}

void processSerialCommand() {
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();
        
        if (command == "R") {
            startCardEmulation();
        } else if (command == "S") {
            stopCardEmulation();
        } else if (command == "STATUS") {
            Serial.printf("System: %s, Discovery: %s, Card: %s, Emulation: %s\n",
                          systemInitialized ? "OK" : "FAIL",
                          discoveryActive ? "ACTIVE" : "INACTIVE",
                          cardActivated ? "CONNECTED" : "IDLE",
                          emulationRequested ? "REQUESTED" : "OFF");
            Serial.printf("Card Reader: Present=%s, UID=%s\n",
                          isCardPresent ? "YES" : "NO",
                          currentCardUID.c_str());
        } else if (command == "PHASE26") {
            Serial.println("=== PHASE 2.6 STATUS ===");
            Serial.printf("Current UID: %s\n", currentCardUID.c_str());
            Serial.printf("Last Valid UID: %s (age: %lu ms)\n", 
                          lastValidUID.c_str(), 
                          lastValidUID.length() > 0 ? (millis() - lastValidUIDTime) : 0);
            Serial.printf("UID Validation Fails: %d/%d\n", uidValidationFailCount, MAX_UID_VALIDATION_FAILS);
            Serial.printf("Invalid Responses: %d/%d\n", consecutiveInvalidResponses, MAX_INVALID_RESPONSES);
            Serial.printf("Preserve Last UID: %s\n", preserveLastUID ? "YES" : "NO");
            Serial.printf("dynamicNFCID: ");
            for (int i = 0; i < RFAL_LM_NFCID_LEN_04; i++) {
                Serial.printf("%02X ", dynamicNFCID[i]);
            }
            Serial.println();
        } else if (command == "RESET") {
            autoEmulationTriggered = false;
            detectedCardUID = "";
            cardRemovedDuringEmulation = false;
            emulationAutoStartTime = 0;
            resetCardDetectionState();
            if (emulationRequested) {
                stopCardEmulation();
            }
        } else if (command == "HELP") {
            Serial.println("Commands:");
            Serial.println("R - Start emulation");
            Serial.println("S - Stop emulation");
            Serial.println("STATUS - Show status");
            Serial.println("PHASE26 - Show Phase 2.6 status");
            Serial.println("RESET - Reset states");
        }
    }
}

void resetSystem() {
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

bool initializeNFC() {
    ReturnCode init_err = gNFCReader.rfalNfcInitialize();
    
    if (init_err == ERR_NONE) {
        demoCeInit(ceNFCF_nfcid2);
        systemInitialized = true;
        consecutiveErrors = 0;
        return true;
    } else {
        consecutiveErrors++;
        systemInitialized = false;
        
        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            resetSystem();
            delay(1000);
        }
        
        return false;
    }
}

ReturnCode demoTransceiveBlocking(uint8_t* txBuf, uint16_t txBufSize, uint8_t** rxData, uint16_t** rcvLen, uint32_t fwt) {
    ReturnCode err;
    uint32_t startTime = millis();
    const uint32_t EXCHANGE_TIMEOUT = 2000;
    
    err = gNFCReader.rfalNfcDataExchangeStart(txBuf, txBufSize, rxData, rcvLen, fwt);
    
    if (err == ERR_NONE) {
        do {
            gNFCReader.rfalNfcWorker();
            err = gNFCReader.rfalNfcDataExchangeGetStatus();
            
            if (millis() - startTime > EXCHANGE_TIMEOUT) {
                return ERR_TIMEOUT;
            }
            
            yield();
        } while (err == ERR_BUSY);
    }
    
    return err;
}

void restartDiscovery() {
    if (!systemInitialized) return;
    
    rfalNfcDiscoverParam discover_params;
    memset(&discover_params, 0, sizeof(discover_params));
    
    discover_params.compMode = RFAL_COMPLIANCE_MODE_NFC;
    discover_params.devLimit = 1;
    discover_params.techs2Find = RFAL_NFC_LISTEN_TECH_A;
    discover_params.totalDuration = 100U;
    discover_params.wakeupEnabled = false;
    
    if (uidUpdateRequired) {
        ST_MEMCPY(&discover_params.lmConfigPA.nfcid, dynamicNFCID, RFAL_LM_NFCID_LEN_04);
    } else {
        static uint8_t defaultNFCID[] = {0x08, 'S', 'T', 'M'};
        ST_MEMCPY(&discover_params.lmConfigPA.nfcid, defaultNFCID, RFAL_LM_NFCID_LEN_04);
    }
    
    ST_MEMCPY(&discover_params.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN);
    discover_params.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;
    discover_params.lmConfigPA.SEL_RES = ceNFCA_SEL_RES;
    discover_params.notifyCb = demoNotif;
    
    ReturnCode result = gNFCReader.rfalNfcDiscover(&discover_params);
    discoveryActive = true;
}

void demoNotif(rfalNfcState st) {
    static rfalNfcState lastState = RFAL_NFC_STATE_NOTINIT;
    
    if (st != lastState) {
        lastState = st;
    }
    
    lastActivity = millis();
    
    switch (st) {
        case RFAL_NFC_STATE_ACTIVATED:
            if (emulationRequested) {
                Serial.println("*** READER CONNECTED ***");
                digitalWrite(kPinLED, HIGH);
                cardActivated = true;
                skipHeartbeat = false;
            }
            break;
            
        case RFAL_NFC_STATE_DATAEXCHANGE:
            if (emulationRequested) {
                handleDataExchange();
            }
            break;
            
        case RFAL_NFC_STATE_START_DISCOVERY:
            if (emulationRequested) {
                Serial.println("*** WAITING FOR READER ***");
                digitalWrite(kPinLED, LOW);
                cardActivated = false;
                discoveryActive = true;
                skipHeartbeat = true;
            }
            break;
            
        case RFAL_NFC_STATE_LISTEN_TECHDETECT:
            if (emulationRequested) {
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
            if (emulationRequested) {
                Serial.println("*** READER ACTIVATING ***");
            }
            break;
            
        default:
            break;
    }
}

void handleDataExchange() {
    ReturnCode err;
    uint8_t* rxData;
    uint16_t* rcvLen;
    uint8_t txBuf[256];
    uint16_t txLen = 0;
    
    err = demoTransceiveBlocking(NULL, 0, &rxData, &rcvLen, 0);
    
    if (err == ERR_NONE && rcvLen && *rcvLen > 0) {
        txLen = demoCeT4T(rxData, *rcvLen, txBuf, sizeof(txBuf));
        
        if (txLen > 0) {
            err = demoTransceiveBlocking(txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE);
        }
    }
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("========================================");
    Serial.println("ESP32 ST25R3916 Card Emulation v2.6");
    Serial.println("PHASE 2.6: UID VALIDATION & ROBUST REMOVAL");
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
    
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    
    if (!initializeNFC()) {
        Serial.println("FATAL: NFC initialization failed");
        while (1) {
            digitalWrite(kPinLED, HIGH);
            delay(200);
            digitalWrite(kPinLED, LOW);
            delay(200);
        }
    }
    
    restartDiscovery();
    
    Serial.println("========================================");
    Serial.println("PHASE 2.6 READY");
    Serial.println("");
    Serial.println("✅ UID Validation & Corruption Protection");
    Serial.println("✅ Multi-Method Card Removal Detection");
    Serial.println("✅ Last Good UID Preservation");
    Serial.println("✅ Enhanced HID Response Validation");
    Serial.println("");
    Serial.println("Commands: R=Start, S=Stop, STATUS=Info, PHASE26=Debug");
    Serial.println("========================================");
    
    emulationRequested = false;
    autoEmulationTriggered = false;
    detectedCardUID = "";
    cardRemovedDuringEmulation = false;
    emulationAutoStartTime = 0;
    uidUpdateRequired = false;
    lastCardPresentTime = millis();
    
    // Initialize Phase 2.6 variables
    lastValidUID = "";
    previousValidUID = "";
    uidValidationFailCount = 0;
    preserveLastUID = false;
    lastValidUIDTime = 0;
    consecutiveInvalidResponses = 0;
    cardRemovalConfirmed = false;
    cardRemovalStartTime = 0;
    validResponseCount = 0;
}

void loop() {
    static unsigned long lastHeartbeat = 0;
    static unsigned long lastRestartCheck = 0;
    unsigned long currentTime = millis();
    
    processSerialCommand();
    
    if (emulationRequested && emulationAutoStartTime > 0) {
        if (currentTime - emulationAutoStartTime >= AUTO_STOP_TIMEOUT) {
            autoStopCardEmulation("1.5-second timeout");
        }
        else if (cardRemovedDuringEmulation) {
            autoStopCardEmulation("card removed");
        }
    }
    
    if (cardReadingEnabled && !emulationRequested && !cardActivated) {
        if (currentTime - lastPollTime >= POLL_INTERVAL) {
            Serial2.write(COMMAND_READ_CSN, sizeof(COMMAND_READ_CSN));
            lastPollTime = currentTime;
        }
        
        if (Serial2.available()) {
            processReaderResponseV26();
        }
    } else if (emulationRequested || cardActivated) {
        while (Serial2.available()) Serial2.read();
    }
    
    if (!skipHeartbeat && currentTime - lastHeartbeat > 15000) {
        Serial.printf("Status: Sys:%s Emulation:%s Card:%s CardReader:%s\n",
                      systemInitialized ? "OK" : "FAIL",
                      emulationRequested ? "ACTIVE" : "IDLE",
                      cardActivated ? "CONNECTED" : "STANDBY",
                      (cardReadingEnabled && !emulationRequested) ? "POLLING" : "PAUSED");
        
        if (emulationRequested && emulationAutoStartTime > 0) {
            unsigned long elapsed = currentTime - emulationAutoStartTime;
            unsigned long remaining = (elapsed < AUTO_STOP_TIMEOUT) ? (AUTO_STOP_TIMEOUT - elapsed) : 0;
            Serial.printf("Auto-stop: %lu ms remaining\n", remaining);
        }
        
        lastHeartbeat = currentTime;
    }
    
    if (!systemInitialized) {
        if (currentTime - lastRestartCheck > 5000) {
            if (initializeNFC()) {
                Serial.println("System recovered");
            }
            lastRestartCheck = currentTime;
        }
        delay(100);
        return;
    }
    
    if (emulationRequested && currentTime - lastWorkerCall >= WORKER_INTERVAL) {
        gNFCReader.rfalNfcWorker();
        lastWorkerCall = currentTime;
    }
    
    if (emulationRequested) {
        rfalNfcState currentState = gNFCReader.rfalNfcGetState();
        
        if (!discoveryActive && currentState == RFAL_NFC_STATE_LISTEN_SLEEP) {
            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL) {
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }
        
        if (!discoveryActive && !cardActivated && 
            currentState != RFAL_NFC_STATE_START_DISCOVERY && 
            currentState != RFAL_NFC_STATE_LISTEN_TECHDETECT &&
            currentState != RFAL_NFC_STATE_LISTEN_COLAVOIDANCE &&
            currentState != RFAL_NFC_STATE_LISTEN_ACTIVATION) {
            
            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL) {
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }
    }
    
    yield();
}