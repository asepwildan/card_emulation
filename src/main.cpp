#include <Arduino.h>
#include <SPI.h>

// Core RFAL and ST25R3916 Headers
#include <rfal_nfc.h>
#include <rfal_nfca.h>
#include <rfal_nfcb.h>
#include <rfal_nfcv.h>
#include <rfal_rf.h>
#include <rfal_rfst25r3916.h>
#include <st_errno.h>
#include "demo_ce.h"

// Forward declarations
void demoNotif(rfalNfcState st);
void handleDataExchange();
void restartDiscovery();
const char* rfalNfcState2Str(rfalNfcState st);
ReturnCode demoTransceiveBlocking(uint8_t* txBuf, uint16_t txBufSize, uint8_t** rxData, uint16_t** rcvLen, uint32_t fwt);

// ESP32 SPI pin configuration - OPTIMIZED
const int kPinMOSI = 23;
const int kPinMISO = 19;
const int kPinSCK = 18;
const int kPinSS = 5;
const int kPinIRQ = -1;    // DISABLED for HID OMNIKEY compatibility
const int kPinLED = 2;

// Use VSPI peripheral for SPI communication on ESP32
SPIClass gSPI(VSPI);

// Hardware and NFC Layer
RfalRfST25R3916Class gReaderHardware(&gSPI, kPinSS, kPinIRQ);
RfalNfcClass gNFCReader(&gReaderHardware);

// Card Emulation Configuration - OPTIMIZED
#define DEMO_LM_SEL_RES 0x20U
#define DEMO_LM_NFCID2_BYTE1 0x02U

// IMPORTANT: Proper initialization arrays
static uint8_t ceNFCF_nfcid2[] = {DEMO_LM_NFCID2_BYTE1, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static uint8_t ceNFCA_SENS_RES[] = {0x04, 0x00};  // Changed for better compatibility
static uint8_t ceNFCA_NFCID[] = {0x08, 'S', 'T', 'M'}; // Custom UID: 08STM (mimic Mifare Classic)
static uint8_t ceNFCA_SEL_RES = DEMO_LM_SEL_RES;

static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 
                       0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

// State management variables - MANUAL TRIGGER MODE
bool cardActivated = false;
unsigned long lastActivity = 0;
unsigned long cardEmulationStartTime = 0;
const unsigned long DISCOVERY_RESTART_INTERVAL = 1000; // 1 second for quick restart
bool discoveryActive = false;
bool systemInitialized = false;
uint8_t consecutiveErrors = 0;
const uint8_t MAX_CONSECUTIVE_ERRORS = 3;
bool emulationRequested = false; // Manual trigger flag

// Performance optimization flags
bool skipHeartbeat = false;
unsigned long lastWorkerCall = 0;
const unsigned long WORKER_INTERVAL = 1; // 1ms for responsive operation

// Helper function to convert RFAL NFC state enum to string
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

// RADICAL FIX: Completely bypass restart mechanism
void startCardEmulation() {
    if (!systemInitialized) {
        Serial.println("System not ready");
        return;
    }
    
    Serial.println("=== STARTING CARD EMULATION ===");
    
    emulationRequested = true;
    
    // BYPASS restartDiscovery() completely - try direct approach
    // The 5+ second delay is likely in the restart mechanism itself
    
    // If discovery is already running, just enable emulation
    if (discoveryActive) {
        Serial.println("Discovery active - emulation enabled immediately");
        return;
    }
    
    // Otherwise, try the minimal restart
    restartDiscovery();
}

// MANUAL TRIGGER: Stop card emulation
void stopCardEmulation() {
    Serial.println("=== STOPPING CARD EMULATION ===");
    
    emulationRequested = false;
    
    // Deactivate current session
    if (cardActivated || discoveryActive) {
        ReturnCode deactErr = gNFCReader.rfalNfcDeactivate(false);
        if (deactErr != ERR_NONE) {
            Serial.printf("Deactivation warning: %d\n", deactErr);
        }
    }
    
    // Reset states
    cardActivated = false;
    discoveryActive = false;
    digitalWrite(kPinLED, LOW);
    
    Serial.println("Card emulation stopped");
}

// MANUAL TRIGGER: Process serial commands
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
        } else if (command == "HELP") {
            Serial.println("Commands:");
            Serial.println("R      - Start card emulation");
            Serial.println("S      - Stop card emulation");
            Serial.println("STATUS - Show system status");
            Serial.println("HELP   - Show this help");
        } else {
            Serial.println("Unknown command. Type HELP for available commands.");
        }
    }
}
void resetSystem() {
    Serial.println("=== SYSTEM RESET ===");
    
    // Reset all states
    digitalWrite(kPinLED, LOW);
    systemInitialized = false;
    discoveryActive = false;
    cardActivated = false;
    consecutiveErrors = 0;
    
    // Clean SPI reset
    gSPI.end();
    delay(200); // Longer delay for proper reset
    
    // Reinitialize SPI with proper settings
    gSPI.begin(kPinSCK, kPinMISO, kPinMOSI, kPinSS);
    gSPI.setFrequency(1000000); // 1MHz for stability
    gSPI.setDataMode(SPI_MODE0);
    gSPI.setBitOrder(MSBFIRST);
    
    // Reset GPIO states
    digitalWrite(kPinSS, HIGH);
    delay(100);
    
    Serial.println("Hardware reset completed");
}

// OPTIMIZED NFC initialization
bool initializeNFC() {
    Serial.print("Initializing NFC stack... ");
    
    ReturnCode init_err = gNFCReader.rfalNfcInitialize();
    
    if (init_err == ERR_NONE) {
        Serial.println("SUCCESS");
        
        // CRITICAL: Proper card emulation initialization
        Serial.print("Initializing T4T card emulation... ");
        demoCeInit(ceNFCF_nfcid2);
        Serial.println("SUCCESS");
        
        systemInitialized = true;
        consecutiveErrors = 0;
        return true;
    } else {
        Serial.printf("FAILED (error: %d)\n", init_err);
        consecutiveErrors++;
        systemInitialized = false;
        
        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            Serial.println("Max errors reached - performing system reset");
            resetSystem();
            delay(1000);
        }
        
        return false;
    }
}

// OPTIMIZED data exchange with better error handling
ReturnCode demoTransceiveBlocking(uint8_t* txBuf, uint16_t txBufSize, uint8_t** rxData, uint16_t** rcvLen, uint32_t fwt) {
    ReturnCode err;
    uint32_t startTime = millis();
    const uint32_t EXCHANGE_TIMEOUT = 2000; // 2 second timeout for gate application
    
    err = gNFCReader.rfalNfcDataExchangeStart(txBuf, txBufSize, rxData, rcvLen, fwt);
    
    if (err == ERR_NONE) {
        do {
            gNFCReader.rfalNfcWorker();
            err = gNFCReader.rfalNfcDataExchangeGetStatus();
            
            if (millis() - startTime > EXCHANGE_TIMEOUT) {
                Serial.println("Data exchange timeout!");
                return ERR_TIMEOUT;
            }
            
            yield();
        } while (err == ERR_BUSY);
    }
    
    if (err != ERR_NONE && err != ERR_BUSY) {
        Serial.printf("Data Exchange Error: %d\n", err);
    }
    
    return err;
}

// RADICAL FIX: Minimal discovery restart - ATTACK THE 5+ SECOND DELAY
void restartDiscovery() {
    if (!systemInitialized) {
        return;
    }
    
    // COMPLETELY SKIP deactivation - this is likely the 5+ second culprit
    // NO state waiting, NO delays, NO complex checks
    
    // Immediate parameter setup
    rfalNfcDiscoverParam discover_params;
    memset(&discover_params, 0, sizeof(discover_params));
    
    // Minimal required configuration
    discover_params.compMode = RFAL_COMPLIANCE_MODE_NFC;
    discover_params.devLimit = 1;
    discover_params.techs2Find = RFAL_NFC_LISTEN_TECH_A;
    discover_params.totalDuration = 100U; // MINIMAL duration
    discover_params.wakeupEnabled = false;
    
    // Essential Listen Mode Configuration only
    ST_MEMCPY(&discover_params.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN);
    ST_MEMCPY(&discover_params.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04);
    discover_params.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;
    discover_params.lmConfigPA.SEL_RES = ceNFCA_SEL_RES;
    discover_params.notifyCb = demoNotif;
    
    // IMMEDIATE start - no error checking that could delay
    gNFCReader.rfalNfcDiscover(&discover_params);
    discoveryActive = true;
}

// ENHANCED notification callback with READER DETECTION
void demoNotif(rfalNfcState st) {
    static rfalNfcState lastState = RFAL_NFC_STATE_NOTINIT;
    
    // Only log state changes for cleaner output
    if (st != lastState) {
        Serial.printf("STATE: %s -> %s\n", rfalNfcState2Str(lastState), rfalNfcState2Str(st));
        lastState = st;
    }
    
    lastActivity = millis();
    
    switch (st) {
        case RFAL_NFC_STATE_ACTIVATED:
            if (emulationRequested) {
                Serial.println("*** READER CONNECTED - CARD ACTIVATED ***");
                digitalWrite(kPinLED, HIGH);
                cardActivated = true;
                skipHeartbeat = false;
            }
            break;
            
        case RFAL_NFC_STATE_DATAEXCHANGE:
            if (emulationRequested) {
                Serial.println("*** READER COMMUNICATING ***");
                handleDataExchange();
            }
            break;
            
        case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
            if (emulationRequested) {
                Serial.println("*** READER COMMUNICATION COMPLETE ***");
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
                // Card emulation is active and waiting for reader
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
                Serial.println("*** READER ACTIVATING CARD ***");
            }
            break;
            
        default:
            // Handle other states silently
            break;
    }
}

// OPTIMIZED data exchange handler
void handleDataExchange() {
    ReturnCode err;
    uint8_t* rxData;
    uint16_t* rcvLen;
    uint8_t txBuf[256];
    uint16_t txLen = 0;
    
    // Receive data from reader
    err = demoTransceiveBlocking(NULL, 0, &rxData, &rcvLen, 0);
    
    if (err == ERR_NONE && rcvLen && *rcvLen > 0) {
        Serial.printf("Received %d bytes from reader\n", *rcvLen);
        
        // Process APDU using T4T handler
        txLen = demoCeT4T(rxData, *rcvLen, txBuf, sizeof(txBuf));
        
        if (txLen > 0) {
            Serial.printf("Sending %d bytes response\n", txLen);
            
            // Send response back to reader
            err = demoTransceiveBlocking(txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE);
            
            if (err != ERR_NONE) {
                Serial.printf("Failed to send response: %d\n", err);
            } else {
                Serial.println("Response sent successfully");
            }
        } else {
            Serial.println("No response generated");
        }
    } else if (err != ERR_BUSY && err != ERR_NONE) {
        Serial.printf("Data exchange error: %d\n", err);
    }
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("========================================");
    Serial.println("ESP32 ST25R3916 Card Emulation v2.1");
    Serial.println("RADICAL DELAY FIX ATTEMPT");
    Serial.println("========================================");
    
    // Print configuration
    Serial.printf("MOSI:%d MISO:%d SCK:%d SS:%d IRQ:%s LED:%d\n", 
                  kPinMOSI, kPinMISO, kPinSCK, kPinSS, 
                  (kPinIRQ == -1) ? "DISABLED" : String(kPinIRQ).c_str(), 
                  kPinLED);
    
    // Initialize GPIO
    pinMode(kPinLED, OUTPUT);
    pinMode(kPinSS, OUTPUT);
    digitalWrite(kPinLED, LOW);
    digitalWrite(kPinSS, HIGH);
    
    // Initialize SPI
    gSPI.begin(kPinSCK, kPinMISO, kPinMOSI, kPinSS);
    gSPI.setFrequency(1000000);
    gSPI.setDataMode(SPI_MODE0);
    gSPI.setBitOrder(MSBFIRST);
    
    Serial.println("SPI initialized");
    
    // Initialize NFC stack
    if (!initializeNFC()) {
        Serial.println("FATAL: NFC initialization failed");
        while (1) {
            digitalWrite(kPinLED, HIGH);
            delay(200);
            digitalWrite(kPinLED, LOW);
            delay(200);
        }
    }
    
    // EXPERIMENT: Start discovery at boot and keep it running
    // Theory: The 5+ second delay happens during discovery restart
    // Solution: Never restart, just enable/disable emulation flag
    Serial.println("Starting persistent discovery...");
    restartDiscovery();
    
    Serial.println("========================================");
    Serial.println("READY - Discovery should be persistent now");
    Serial.println("R - Enable emulation (should be instant)");
    Serial.println("S - Disable emulation");
    Serial.println("========================================");
    
    emulationRequested = false;
}

void loop() {
    static unsigned long lastHeartbeat = 0;
    static unsigned long lastRestartCheck = 0;
    unsigned long currentTime = millis();
    
    // Process serial commands
    processSerialCommand();
    
    // OPTIMIZED heartbeat - minimal noise
    if (!skipHeartbeat && currentTime - lastHeartbeat > 15000) { // 15 seconds
        Serial.printf("Status: Sys:%s Emulation:%s Card:%s\n",
                      systemInitialized ? "OK" : "FAIL",
                      emulationRequested ? "ACTIVE" : "IDLE",
                      cardActivated ? "CONNECTED" : "STANDBY");
        lastHeartbeat = currentTime;
    }
    
    // System recovery
    if (!systemInitialized) {
        if (currentTime - lastRestartCheck > 5000) {
            Serial.println("Attempting system recovery...");
            if (initializeNFC()) {
                Serial.println("System recovered");
            }
            lastRestartCheck = currentTime;
        }
        delay(100);
        return;
    }
    
    // CRITICAL: Run NFC worker only when emulation is requested
    if (emulationRequested && currentTime - lastWorkerCall >= WORKER_INTERVAL) {
        gNFCReader.rfalNfcWorker();
        lastWorkerCall = currentTime;
    }
    
    // Only process NFC states when emulation is active
    if (emulationRequested) {
        rfalNfcState currentState = gNFCReader.rfalNfcGetState();
        
        // Auto-restart discovery if in sleep state during active emulation
        if (!discoveryActive && currentState == RFAL_NFC_STATE_LISTEN_SLEEP) {
            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL) {
                Serial.println("Auto-restarting from sleep state");
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }
        
        // Recovery from unexpected states during active emulation
        if (!discoveryActive && !cardActivated && 
            currentState != RFAL_NFC_STATE_START_DISCOVERY && 
            currentState != RFAL_NFC_STATE_LISTEN_TECHDETECT &&
            currentState != RFAL_NFC_STATE_LISTEN_COLAVOIDANCE &&
            currentState != RFAL_NFC_STATE_LISTEN_ACTIVATION) {
            
            if (currentTime - lastRestartCheck > DISCOVERY_RESTART_INTERVAL) {
                Serial.printf("Unexpected state (%s) - restarting discovery\n", rfalNfcState2Str(currentState));
                restartDiscovery();
                lastRestartCheck = currentTime;
            }
        }
    }
    
    // Minimal delay for responsive operation
    yield();
}
