#ifndef DEMO_CE_H
#define DEMO_CE_H

#include <stdint.h>
#include <stdbool.h>

void demoCeInit(uint8_t* nfcfNfcid);
uint16_t demoCeT4T(uint8_t *rxData, uint16_t rxDataLen, uint8_t *txBuf, uint16_t txBufLen);

#endif
