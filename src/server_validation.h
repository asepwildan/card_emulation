#ifndef SERVER_VALIDTION_H
#define SERVER_VALIDATION_H

#include <Arduino.h>

bool validateCSN(const String& csn);
// Utility functions
String arrayToHexString(uint8_t* data, uint8_t length);
String formatCSNForServer(uint8_t* csn, uint8_t length);
#endif
