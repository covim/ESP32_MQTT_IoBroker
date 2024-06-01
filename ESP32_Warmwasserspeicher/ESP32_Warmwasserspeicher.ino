#include "EspMQTTClient.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_sleep.h>

// Konfiguration des MQTT-Clients
EspMQTTClient client(
  "Muehle",                  // WiFi SSID
  "mute4harfe",              // WiFi Passwort
  "192.168.1.88",            // MQTT Broker IP
  "",                        // MQTT Benutzername (nicht benötigt)
  "",                        // MQTT Passwort (nicht benötigt)
  "Warmwasserspeicher_ESP32-MCU", // MQTT Client Name
  1883                       // MQTT Port (Standard 1883)
);

const int SENSOR_PIN = 13; // ESP32-Pin, verbunden mit dem DQ-Pin des DS18B20-Sensors
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer; // Variable für die Sensoradresse

// Sensoradressen
uint8_t sensor1[8] = { 0x28, 0x42, 0x4D, 0x0E, 0x5F, 0x20, 0x01, 0xAE };
uint8_t sensor2[8] = { 0x28, 0xC2, 0x82, 0x1B, 0x5F, 0x20, 0x01, 0xB0 };
uint8_t sensor3[8] = { 0x28, 0x7A, 0xD6, 0x07, 0x5F, 0x20, 0x01, 0xB6 };
uint8_t sensor4[8] = { 0x28, 0x56, 0xB0, 0x44, 0x5F, 0x20, 0x01, 0x98 };
uint8_t sensor5[8] = { 0x28, 0x1E, 0xAF, 0x1C, 0x5F, 0x20, 0x01, 0x9B };
uint8_t sensor6[8] = { 0x28, 0x65, 0x0D, 0x03, 0x5F, 0x20, 0x01, 0xE9 };
uint8_t sensor7[8] = { 0x28, 0x93, 0x72, 0x03, 0x5F, 0x20, 0x01, 0x2B };
uint8_t sensor8[8] = { 0x28, 0xE7, 0x86, 0xFF, 0x5E, 0x20, 0x01, 0x1E };

void setup() {
  // Serielle Kommunikation starten
  Serial.begin(9600);
  sensors.begin(); // Sensoren initialisieren
  
  // Anzahl der Sensoren auf dem Bus ermitteln
  Serial.print("# Sensoren: ");
  int AnzahlSensoren = sensors.getDeviceCount();
  Serial.println(AnzahlSensoren);

  // Adressen der gefundenen Sensoren ausgeben
  Serial.println("Locating devices...");
  Serial.print("Found ");
  Serial.print(AnzahlSensoren, DEC);
  Serial.println(" devices.");
  Serial.println("");

  Serial.println("Printing addresses...");
  for (int i = 0; i < AnzahlSensoren; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" : ");
    sensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }

  // Optionale Funktionalitäten des EspMQTTClient aktivieren
  client.enableDebuggingMessages(); // Debugging-Nachrichten an den seriellen Ausgang senden
}

// Diese Funktion wird aufgerufen, sobald alles verbunden ist (WLAN und MQTT)
void onConnectionEstablished() {
  // Abonnieren eines MQTT-Themas und Anzeigen der empfangenen Nachricht auf Serial
  client.subscribe("Warmwasserspeicher/OneWireSensoren", [](const String & payload) {
    Serial.println(payload);
  });
}

// Funktion zum Veröffentlichen der Temperaturdaten über MQTT
bool publishTemperature(const char* topic, float temperature) {
  bool result = client.publish(topic, String(temperature)); // Nachricht senden
  delay(100); // Kurze Verzögerung, um sicherzustellen, dass die Nachricht gesendet wird
  return result;
}

// Funktion zum Abrufen und Veröffentlichen der Temperatur
bool getAndPublish(const uint8_t* sensor, const char* name) {
  float temperature = sensors.getTempC(sensor);
  Serial.print(name);
  Serial.print(": ");
  Serial.println(temperature);
  bool isPublished = publishTemperature(name, temperature);
  return isPublished;
}

void loop() {
  client.loop(); // MQTT-Client loop
  
  // Überprüfen, ob WLAN und MQTT verbunden sind
  if (client.isConnected()) {
    sensors.requestTemperatures(); // Temperaturen anfordern

    // Temperaturen der einzelnen Sensoren lesen und veröffentlichen
    bool sensor1Published = getAndPublish(sensor1, "Warmwasserspeicher/Rechts_unten");
    bool sensor2Published = getAndPublish(sensor2, "Warmwasserspeicher/Rechts_mitte");
    bool sensor3Published = getAndPublish(sensor3, "Warmwasserspeicher/Rechts_oben");
    bool sensor4Published = getAndPublish(sensor4, "Warmwasserspeicher/Links_unten");
    bool sensor5Published = getAndPublish(sensor5, "Warmwasserspeicher/Links_mitte");
    bool sensor6Published = getAndPublish(sensor6, "Warmwasserspeicher/Links_oben");
    bool sensor7Published = getAndPublish(sensor7, "Solaranlage/Ruecklauf");
    bool sensor8Published = getAndPublish(sensor8, "Solaranlage/Vorlauf");

    // Warten, bis alle Nachrichten veröffentlicht wurden
    if (sensor1Published && sensor2Published && 
        sensor3Published && sensor4Published && 
        sensor5Published && sensor6Published && 
        sensor7Published && sensor8Published) {
      delay(500); // Zusätzliche Verzögerung, um sicherzustellen, dass alle Nachrichten gesendet wurden

      // In den Deep Sleep Modus für 10 Sekunden gehen
      unsigned long sleepDuration = 30 * 1000000; // 30 Sekunden in Mikrosekunden
      esp_sleep_enable_timer_wakeup(sleepDuration);
      esp_deep_sleep_start();
    }
  } else {
    // Kurz warten, bevor erneut geprüft wird
    delay(1000); // 1 Sekunde warten
  }
}

// Funktion zum Drucken der Sensoradresse
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}
