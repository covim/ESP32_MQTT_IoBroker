#include "EspMQTTClient.h"
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(05, 23, 19, 18);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 1000.0

// MQTT-Client-Konfiguration
EspMQTTClient client(
  "Muehle",                    // WiFi SSID
  "mute4harfe",                // WiFi Passwort
  "192.168.1.88",              // MQTT Broker IP
  "",                          // MQTT Benutzername (nicht benötigt)
  "",                          // MQTT Passwort (nicht benötigt)
  "HeizungVorlauf_OG_ESP32-MCU",  // MQTT Client Name
  1883                         // MQTT Port (Standard 1883)
);

void setup() {
  Serial.begin(9600);
  thermo.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary

  // Optionale Funktionalitäten des EspMQTTClient aktivieren
  client.enableDebuggingMessages(); // Debugging-Nachrichten an den seriellen Ausgang senden
}

// Diese Funktion wird aufgerufen, sobald alles verbunden ist (WLAN und MQTT)
void onConnectionEstablished() {
  // Abonnieren eines MQTT-Themas und Anzeigen der empfangenen Nachricht auf Serial
  client.subscribe("Fussbodenheizung_OG/OG_Vorlauf", [](const String & payload) {
    Serial.println(payload);
  });
}

// Funktion zum Veröffentlichen der Temperaturdaten über MQTT
bool publishTemperature(const char* topic, float temperature) {
  bool result = client.publish(topic, String(temperature)); // Nachricht senden
  delay(100); // Kurze Verzögerung, um sicherzustellen, dass die Nachricht gesendet wird
  return result;
}

void loop() {
  client.loop(); // MQTT-Client loop

  // Überprüfen, ob WLAN und MQTT verbunden sind
  if (client.isConnected()) {
    uint16_t rtd = thermo.readRTD();
    float ratio = rtd;
    ratio /= 32768;
    float temperatur = thermo.temperature(RNOMINAL, RREF);

    // Temperaturdaten veröffentlichen
    bool isPublished = publishTemperature("Fussbodenheizung_OG/OG_Vorlauf", temperatur);

    if (isPublished) {
      Serial.println(temperatur);
      delay(500); // Zusätzliche Verzögerung, um sicherzustellen, dass die Nachricht gesendet wurde

      // In den Deep Sleep Modus für 30 Sekunden gehen
      unsigned long sleepDuration = 30 * 1000000; // 30 Sekunden in Mikrosekunden
      esp_sleep_enable_timer_wakeup(sleepDuration);
      esp_deep_sleep_start();
    }
  } else {
    // Kurz warten, bevor erneut geprüft wird
    delay(1000); // 1 Sekunde warten
  }
}
