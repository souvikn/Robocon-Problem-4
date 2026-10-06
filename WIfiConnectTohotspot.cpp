#include <WiFi.h>


const char* ssid     = "YOUR_HOTSPOT_NAME";
const char* password = "YOUR_HOTSPOT_PASSWORD";

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.print("Connecting to Hotspot: ");
    Serial.println(ssid);

  
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

   
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 30) { 
            Serial.println("\nFailed to connect. Check SSID/Password or 2.4GHz settings.");
            return;
        }
    }

    Serial.println("");
    Serial.println("Wi-Fi Connected Successfully!");
    Serial.print("ESP32-S3 IP Address: ");
    Serial.println(WiFi.localIP());
}

void loop() {

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi lost! Reconnecting...");
        WiFi.reconnect();
        delay(2000);
    }
}