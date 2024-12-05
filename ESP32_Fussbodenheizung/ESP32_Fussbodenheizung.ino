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
  "FussbodenHeizung_OG_ESP32-MCU", // MQTT Client Name
  1883                       // MQTT Port (Standard 1883)
);

const int SENSOR_PIN = 13; // ESP32-Pin, verbunden mit dem DQ-Pin des DS18B20-Sensors
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer; // Variable für die Sensoradresse

// Sensoradressen
uint8_t sensor1[8] = { 0x28, 0xC0, 0x03, 0x62, 0x32, 0x14, 0x01, 0x40 };
uint8_t sensor2[8] = { 0x28, 0xE2, 0xEC, 0x3C, 0x5F, 0x20, 0x01, 0x43 };
uint8_t sensor3[8] = { 0x28, 0xDA, 0x3A, 0x10, 0x5F, 0x20, 0x01, 0xAF };
uint8_t sensor4[8] = { 0x28, 0x71, 0x3F, 0xA6, 0x32, 0x14, 0x01, 0xE2 };

// Variablen für die Temperaturen
float OST;
float MITTE;
float WEST;
float RESERVE;

void setup()
{
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
  for (int i = 0; i < AnzahlSensoren; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" : ");
    sensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }

  // Debugging-Nachrichten für MQTT aktivieren
  client.enableDebuggingMessages();
}

// Funktion, die aufgerufen wird, wenn die Verbindung hergestellt ist
void onConnectionEstablished()
{
  // Abonnieren eines MQTT-Themas
  client.subscribe("Fussbodenheizung_OG/OneWireSensoren", [](const String & payload) {
    Serial.println(payload);
  });
}

// Funktion zum Veröffentlichen der Temperaturdaten über MQTT
bool publishTemperature(const char* topic, float temperature)
{
  bool result = client.publish(topic, String(temperature)); // Nachricht senden
  delay(100); // Kurze Verzögerung, um sicherzustellen, dass die Nachricht gesendet wird
  return result;
}

void loop()
{
  client.loop(); // MQTT-Client loop

  // Überprüfen, ob WLAN und MQTT verbunden sind
  if (client.isConnected()) {
    // Temperaturen anfordern
    sensors.requestTemperatures();

    // Temperaturen der einzelnen Sensoren lesen und veröffentlichen
    OST = sensors.getTempC(sensor1);
    Serial.print("Heizkreis_OST: ");
    Serial.println(OST);
    bool ostPublished = publishTemperature("Fussbodenheizung_OG/HK_OST", OST);

    MITTE = sensors.getTempC(sensor2);
    Serial.print("Heizkreis_MITTE: ");
    Serial.println(MITTE);
    bool mittePublished = publishTemperature("Fussbodenheizung_OG/HK_MITTE", MITTE);

    WEST = sensors.getTempC(sensor3);
    Serial.print("Heizkreis_WEST: ");
    Serial.println(WEST);
    bool westPublished = publishTemperature("Fussbodenheizung_OG/HK_WEST", WEST);

    RESERVE = sensors.getTempC(sensor4);
    Serial.print("ReserveSensor: ");
    Serial.println(RESERVE);
    bool reservePublished = publishTemperature("Fussbodenheizung_OG/ReserveSensor", RESERVE);

    // Warten, bis alle Nachrichten veröffentlicht wurden
    if (ostPublished && mittePublished && westPublished && reservePublished) {
      delay(500); // Zusätzliche Verzögerung, um sicherzustellen, dass alle Nachrichten gesendet wurden

      // In den Deep Sleep Modus für 5 Minuten gehen
      unsigned long sleepDuration = 5 * 60 * 1000000; // 5min in Mikrosekunden
      esp_sleep_enable_timer_wakeup(sleepDuration);
      esp_deep_sleep_start();
    }
  } else {
    // Kurz warten, bevor erneut geprüft wird
    delay(1000); // 1 Sekunde warten
  }
}

// Funktion zum Drucken der Sensoradresse
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}
