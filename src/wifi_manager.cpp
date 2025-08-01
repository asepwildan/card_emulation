#include "wiFi_manager.h"

// WiFi credentials
const char* ssid = "TeravinNetwork";
const char* password = "1234567890";

void initWiFi() {
    Serial.println("📶 ===== INITIALIZING WIFI =====");
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    int attempts = 0;
    const int maxAttempts = 20;  // Max 10 seconds (20 x 500ms)

    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    Serial.println("");

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ WiFi connected!");
        Serial.print("📡 IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("📶 Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("❌ Failed to connect to WiFi!");
        Serial.println("⚠️ System will continue running without WiFi");
    }

    Serial.println("================================\n");
}

void checkWiFiConnection() {
    static unsigned long lastCheck = 0;
    const unsigned long checkInterval = 30000;  // Check every 30 seconds

    if (millis() - lastCheck > checkInterval) {
        lastCheck = millis();

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("⚠️ WiFi disconnected, attempting to reconnect...");
            WiFi.reconnect();

            // Wait a maximum of 5 seconds to reconnect
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 10) {
                delay(500);
                Serial.print(".");
                attempts++;
            }

            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\n✅ WiFi reconnected!");
                Serial.print("📡 IP Address: ");
                Serial.println(WiFi.localIP());
            } else {
                Serial.println("\n❌ WiFi reconnect failed");
            }
        }
    }
}

String getWiFiStatus() {
    switch (WiFi.status()) {
        case WL_CONNECTED:
            return "Connected";
        case WL_NO_SSID_AVAIL:
            return "SSID not available";
        case WL_CONNECT_FAILED:
            return "Connection failed";
        case WL_CONNECTION_LOST:
            return "Connection lost";
        case WL_DISCONNECTED:
            return "Disconnected";
        case WL_IDLE_STATUS:
            return "Idle";
        default:
            return "Unknown";
    }
}

IPAddress getLocalIP() {
    return WiFi.localIP();
}

bool isWiFiConnected() {
    return (WiFi.status() == WL_CONNECTED);
}
