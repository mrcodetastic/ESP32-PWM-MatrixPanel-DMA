/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>

void connectWiFi()
{

    // We start by connecting to a WiFi network
    // WiFi.begin("Tardigrade2", "chocolate-Milk2@");
    WiFi.begin("Pixel_9363", "yellow22");

    Serial.println();
    Serial.println();
    Serial.print("Waiting for WiFi... ");

    while(WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    delay(500);
}
