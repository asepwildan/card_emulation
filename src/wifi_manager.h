#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>

extern const char *ssid;
extern const char *password;

void initWiFi();
void checkWiFiConnection();
String getWiFiStatus();
IPAddress getLocalIP();
bool isWiFiConnected();

#endif
