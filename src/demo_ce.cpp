#include "demo_ce.h"

#include <Arduino.h>

#include "nfc_utils.h"
#include "rfal_nfca.h"
#include "rfal_nfcf.h"
#include "rfal_rf.h"
#include "st_errno.h"

#define NDEF_SIZE 2048      /*!< Max NDEF size emulated. Range: 0005h - 7FFFh    */
#define T4T_CLA_00 0x00     /*!< CLA value for type 4 command                    */
#define T4T_INS_SELECT 0xA4 /*!< INS value for select command                    */
#define T4T_INS_READ 0xB0   /*!< INS value for reabbinary command                */
#define T4T_INS_UPDATE 0xD6 /*!< INS value for update command                    */
#define FID_CC 0xE103       /*!< File ID number for CCFile                       */
#define FID_NDEF 0x0001     /*!< File ID number for NDEF file                    */
#define T3T_BLOCK_SIZE 0x10 /*!< Block size in Type 3 Tag                        */

static uint8_t gNfcfNfcid[RFAL_NFCF_NFCID2_LEN];
static uint8_t ndefFile[NDEF_SIZE]; /*!< Buffer to store NDEF File                 */
static const uint8_t ndef_uri[] = {
    0x00, 0x15,                   /* NDEF length                */
    0xD1,                         /* NDEF Header                */
    0x01,                         /* NDEF type length           */
    0x11,                         /* NDEF payload length        */
    0x55,                         /* NDEF Type                  */
    0x01,                         /* NDEF URI abreviation field */
    0x73, 0x74, 0x2E, 0x63, 0x6F, /* NDEF URI string            */
    0x6D, 0x2F, 0x73, 0x74, 0x32,
    0x35, 0x2D, 0x64, 0x65, 0x6D, 0x6F};

const uint8_t *demoNdefFile = ndef_uri;
uint32_t demoNdefFileLen = sizeof(ndef_uri);

static uint8_t InformationBlock[] = {
    0x10,                   /* Ver        */
    0x08,                   /* Nbr        */
    0x08,                   /* Nbw        */
    0x00, 0x0F,             /* NmaxB      */
    0x00, 0x00, 0x00, 0x00, /* RFU        */
    0x00,                   /* WriteFlag  */
    0x01,                   /* RWFlag     */
    0x00, 0x00, 0x15,       /* Ln         */
    0x00, 0x45              /* Checksum   */
};

enum States {
    STATE_IDLE = 0,         /*!< Emulated Tag state idle                  */
    STATE_APP_SELECTED = 1, /*!< Emulated Tag state application selected  */
    STATE_CC_SELECTED = 2,  /*!< Emulated Tag state CCFile selected       */
    STATE_FID_SELECTED = 3, /*!< Emulated Tag state FileID selected       */
};

static int8_t nState = STATE_IDLE; /*!< Type 4 tag emulation status               */
static int32_t nSelectedIdx = -1;  /*!< current file selected                     */
static int32_t nFiles = 2;         /*!< Number of file emulated                   */

static uint8_t ccfile[] = {
    0x00, 0x0F,                                      /* CCLEN      */
    0x20,                                            /* T4T_VNo    */
    0x00, 0x7F,                                      /* MLe        */
    0x00, 0x7F,                                      /* MLc        */
    0x04,                                            /* T          */
    0x06,                                            /* L          */
    (FID_CC & 0xFF00) >> 8, (FID_CC & 0x00FF),       /* V1         */
    (NDEF_SIZE & 0xFF00) >> 8, (NDEF_SIZE & 0x00FF), /* V2         */
    0x00,                                            /* V3         */
    0x00                                             /* V4         */
};

static uint32_t pdwFileSize[] = {sizeof(ccfile), NDEF_SIZE}; /*!< Current emulated files size     */

void demoCeInit(uint8_t *nfcfNfcid) {
    Serial.println("Initializing Card Emulation T4T...");
    
    if (nfcfNfcid != NULL) {
        ST_MEMCPY(gNfcfNfcid, nfcfNfcid, RFAL_NFCF_NFCID2_LEN);
    }

    ST_MEMCPY(ndefFile, (uint8_t *)demoNdefFile, demoNdefFileLen);
    
    /* Update AIB Ln with actual NDEF length */
    InformationBlock[12] = demoNdefFile[0];
    InformationBlock[13] = demoNdefFile[1];
    uint16_t checksum = 0;
    for (int i = 0; i < 14; i++) {
        checksum += InformationBlock[i];
    }
    InformationBlock[14] = checksum >> 8;
    InformationBlock[15] = checksum & 0xFF;
    
    // Reset state
    nState = STATE_IDLE;
    nSelectedIdx = -1;
    
    Serial.println("Card Emulation T4T initialized successfully");
}

static bool cmdCompare(uint8_t *cmd, uint8_t *find, uint16_t len) {
    for (int i = 0; i < 20; i++) {
        if (!memcmp(&cmd[i], find, len)) {
            return true;
        }
    }
    return false;
}

static uint16_t demoCeT4TSelect(uint8_t *cmdData, uint8_t *rspData) {
    bool success = false;
    /*
     * Cmd: CLA(1) | INS(1) | P1(1) | P2(1) | Lc(1) | Data(n) | [Le(1)]
     * Rsp: [FCI(n)] | SW12
     *
     * Select App by Name NDEF:       00 A4 04 00 07 D2 76 00 00 85 01 01 00
     * Select App by Name NDEF 4 ST:  00 A4 04 00 07 A0 00 00 00 03 00 00 00
     * Select CC FID:                 00 A4 00 0C 02 xx xx
     * Select NDEF FID:               00 A4 00 0C 02 xx xx
     */

    uint8_t aid[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
    uint8_t fidCC[] = {FID_CC >> 8, FID_CC & 0xFF};
    uint8_t fidNDEF[] = {FID_NDEF >> 8, FID_NDEF & 0xFF};
    uint8_t selectFileId[] = {0xA4, 0x00, 0x0C, 0x02, 0x00, 0x01};

    if (cmdCompare(cmdData, aid, sizeof(aid))) { /* Select Application */
        Serial.println("T4T: Select NDEF Application");
        nState = STATE_APP_SELECTED;
        success = true;
    } else if ((nState >= STATE_APP_SELECTED) && cmdCompare(cmdData, fidCC, sizeof(fidCC))) { /* Select CC */
        Serial.println("T4T: Select CC File");
        nState = STATE_CC_SELECTED;
        nSelectedIdx = 0;
        success = true;
    } else if ((nState >= STATE_APP_SELECTED) && (cmdCompare(cmdData, fidNDEF, sizeof(fidNDEF)) || cmdCompare(cmdData, selectFileId, sizeof(selectFileId)))) { /* Select NDEF */
        Serial.println("T4T: Select NDEF File");
        nState = STATE_FID_SELECTED;
        nSelectedIdx = 1;
        success = true;
    } else {
        Serial.println("T4T: Select failed - unknown target");
        nState = STATE_IDLE;
    }

    rspData[0] = (success ? 0x90 : 0x6A);
    rspData[1] = (success ? 0x00 : 0x82);

    return 2;
}

static uint16_t demoCeT4TRead(uint8_t *cmdData, uint8_t *rspData, uint16_t rspDataLen) {
    /*
     * Cmd: CLA(1) | INS(1) | P1(1).. offset inside file high | P2(1).. offset inside file low | Le(1).. nBytes to read
     * Rsp: BytesRead | SW12
     */
    unsigned short offset = (cmdData[2] << 8) | cmdData[3];
    unsigned short toRead = cmdData[4];
    uint8_t *ppbMemory;

    Serial.printf("T4T: Read request - offset: %d, length: %d\n", offset, toRead);

    if (rspDataLen < 2) {
        Serial.println("T4T: Read error - buffer too small");
        rspData[0] = 0x6F;
        rspData[1] = 0x00;
        return 2;
    }

    /* Any file selected */
    if (nSelectedIdx < 0 || nSelectedIdx >= nFiles) {
        Serial.println("T4T: Read error - no file selected");
        rspData[0] = 0x6A;
        rspData[1] = 0x82;
        return 2;
    }

    /* offset + length exceed file size */
    if ((unsigned long)(offset + toRead) > pdwFileSize[nSelectedIdx]) {
        toRead = pdwFileSize[nSelectedIdx] - offset;
        Serial.printf("T4T: Read truncated to %d bytes\n", toRead);
    }

    if (rspDataLen < (toRead + 2)) {
        Serial.println("T4T: Read error - response buffer too small");
        rspData[0] = 0x6F;
        rspData[1] = 0x00;
        return 2;
    }

    ppbMemory = (nSelectedIdx == 0 ? ccfile : ndefFile);
    
    /* read data */
    ST_MEMCPY(rspData, &ppbMemory[offset], toRead);

    Serial.printf("T4T: Read successful - %d bytes from file %d\n", toRead, nSelectedIdx);

    rspData[toRead] = 0x90;
    rspData[toRead + 1] = 0x00;
    return toRead + 2;
}

static uint16_t demoCeT4TUpdate(uint8_t *cmdData, uint8_t *rspData) {
    uint32_t offset = (cmdData[2] << 8) | cmdData[3];
    uint32_t length = cmdData[4];

    Serial.printf("T4T: Update request - offset: %d, length: %d\n", offset, length);

    if (nSelectedIdx != 1) {
        Serial.println("T4T: Update error - wrong file selected");
        rspData[0] = 0x6A;
        rspData[1] = 0x82;
        return 2;
    }

    if ((unsigned long)(offset + length) > pdwFileSize[nSelectedIdx]) {
        Serial.println("T4T: Update error - data too large");
        rspData[0] = 0x62;
        rspData[1] = 0x82;
        return 2;
    }

    ST_MEMCPY((ndefFile + offset), &cmdData[5], length);

    Serial.println("T4T: Update successful");
    rspData[0] = 0x90;
    rspData[1] = 0x00;
    return 2;
}

uint16_t demoCeT4T(uint8_t *rxData, uint16_t rxDataLen, uint8_t *txBuf, uint16_t txBufLen) {
    if ((txBuf == NULL) || (txBufLen < 2)) {
        Serial.println("T4T: Error - invalid buffer");
        return 0;
    }

    // Print received APDU for debugging
    if (rxData != NULL && rxDataLen > 0) {
        Serial.print("T4T: Received APDU: ");
        for (int i = 0; i < rxDataLen; i++) {
            Serial.printf("%02X ", rxData[i]);
        }
        Serial.println();
    }

    if ((rxData != NULL) && (rxDataLen >= 4)) {
        if (rxData[0] == T4T_CLA_00) {
            switch (rxData[1]) {
                case T4T_INS_SELECT:
                    Serial.println("T4T: Processing SELECT command");
                    return demoCeT4TSelect(rxData, txBuf);

                case T4T_INS_READ:
                    Serial.println("T4T: Processing READ command");
                    return demoCeT4TRead(rxData, txBuf, txBufLen);

                case T4T_INS_UPDATE:
                    Serial.println("T4T: Processing UPDATE command");
                    return demoCeT4TUpdate(rxData, txBuf);

                default:
                    Serial.printf("T4T: Unknown instruction: 0x%02X\n", rxData[1]);
                    break;
            }
        } else {
            Serial.printf("T4T: Wrong CLA: 0x%02X\n", rxData[0]);
        }
    } else {
        Serial.println("T4T: Invalid APDU - too short");
    }

    /* Function not supported */
    Serial.println("T4T: Command not supported - sending error response");
    txBuf[0] = 0x68;
    txBuf[1] = 0x00;
    return 2;
}
