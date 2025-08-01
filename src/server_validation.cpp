#include "server_validation.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>

bool validateCSN(const String& csn) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi not connected!");
        return false;
    }

    HTTPClient http;
    http.begin("http://192.168.6.138:3000/api/v1/validation");
    http.addHeader("Content-Type", "application/json");

    // Create JSON body
    String cardDataSample = "v|" + csn + "|1";
    StaticJsonDocument<200> doc;
    doc["cardData"] = cardDataSample;
    doc["qrData"] = nullptr;

    String requestBody;
    serializeJson(doc, requestBody);
    Serial.println("Request body: " + requestBody);

    // Send POST request
    int responseCode = http.POST(requestBody);
    if (responseCode == 200) {
        String response = http.getString();
        Serial.println("✅ Server response: " + response);

        // If response is only string "OK"
        response.trim();  // Remove newline/whitespace
        if (response == "OK") {
            http.end();
            return true;
        }

        // If server sends JSON response
        StaticJsonDocument<256> resDoc;
        DeserializationError error = deserializeJson(resDoc, response);
        if (!error && resDoc["data"] == "granted") {
            http.end();
            return true;
        }

    } else if (responseCode == 401) {
        String errorResponse = http.getString();
        Serial.println("❌ Unauthorized: " + errorResponse);
    } else {
        Serial.println("❌ Request failed: " + String(responseCode));
    }

    http.end();
    return false;
}

String arrayToHexString(uint8_t* data, uint8_t length) {
    String hexString = "";
    for (int i = 0; i < length; i++) {
        if (data[i] < 0x10) {
            hexString += "0";
        }
        hexString += String(data[i], HEX);
    }
    hexString.toUpperCase();
    return hexString;
}

String formatCSNForServer(uint8_t* csn, uint8_t length) {
    return arrayToHexString(csn, length);
}
