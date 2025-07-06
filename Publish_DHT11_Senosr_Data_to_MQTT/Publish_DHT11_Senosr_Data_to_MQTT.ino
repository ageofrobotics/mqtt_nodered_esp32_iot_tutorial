/*************************************************************
  Project : ESP32-S3 MQTT Publisher with DHT11 (Unified API)
  Description : Publishes temperature and humidity separately
                to two MQTT topics every 1 second using
                Adafruit Unified Sensor API for DHT11.
  Hardware : ESP32-S3-WROOM-1, DHT11 Sensor on GPIO 10
  Author : Age of Robotics (Modified with Unified Comments)
*************************************************************/

#include <WiFi.h>           // WiFi library for ESP32 (by Espressif)
#include <PubSubClient.h>   // MQTT client library (by Nick O'Leary)
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor base class
#include <DHT.h>            // Low-level DHT support
#include <DHT_U.h>          // Unified DHT wrapper over Adafruit_Sensor

/************** WiFi Network Credentials ****************/
const char* ssid = "1123";              // <-- Replace with your WiFi SSID
const char* password = "AAABBBHHH";  // <-- Replace with your WiFi Password

/************** MQTT Broker Settings *******************/
const char* mqtt_server_ip = " 192.168.83.XXX"; // <-- Replace with your MQTT Broker IP
const int mqtt_port = 1883;                     // Default MQTT port

/************** MQTT Topics ***************************/
const char* temp_topic = "sensor/dht11/temperature";  // Topic to publish temperature
const char* humid_topic = "sensor/dht11/humidity";    // Topic to publish humidity

/************** DHT Sensor Configuration ***************/
#define DHTPIN 10         // GPIO 10 connected to DHT11 data pin
#define DHTTYPE DHT11     // Specify DHT11 sensor type
DHT_Unified dht(DHTPIN, DHTTYPE);  // Create DHT object using unified API
uint32_t delayMS;         // Delay between readings based on sensor spec

/************** MQTT Client Declarations ***************/
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/************** Connect to WiFi Network ***************/
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password); // Start WiFi connection

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/************** Reconnect to MQTT Broker ***************/
void reconnectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT Client...");
    if (mqttClient.connect("ESP32_S3_Client")) {
      Serial.println("connected to MQTT broker!");
    } else {
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait before retrying
    }
  }
}

/************** Blink LED on GPIO 13 ******************/
void blinkLed() {
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

/************** Arduino Setup Function ****************/
void setup() {
  Serial.begin(115200);   // Initialize Serial Monitor
  delay(100);

  pinMode(13, OUTPUT);    // Setup GPIO 13 as output for LED
  dht.begin();            // Initialize DHT sensor (Unified)

  connectToWiFi();        // Connect to WiFi
  mqttClient.setServer(mqtt_server_ip, mqtt_port); // Set MQTT server

  // Get the minimum required delay between reads
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  Serial.println("DHT11 sensor initialized using Adafruit Unified Sensor API.");
}

/************** Arduino Main Loop *********************/
void loop() {
  if (!mqttClient.connected()) {
    reconnectToMQTT();  // Ensure MQTT connection
  }
  mqttClient.loop();     // Keep MQTT client alive

  static unsigned long lastPublishTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastPublishTime >= delayMS) { // Every min_delay milliseconds
    lastPublishTime = currentMillis;

    // Read temperature and humidity events
    sensors_event_t tempEvent, humidEvent;
    dht.temperature().getEvent(&tempEvent);
    dht.humidity().getEvent(&humidEvent);

    // Validate readings
    if (isnan(tempEvent.temperature) || isnan(humidEvent.relative_humidity)) {
      Serial.println("Failed to read from DHT11 sensor!");
      return;
    }

    // MQTT requires data to be in string format
    // dtostrf() converts float to char array (string) to publish via MQTT

    // Syntax: dtostrf(floatVal, width, precision, buffer)
    // - floatVal: the float number to convert
    // - width: total number of characters (including decimal point and digits)
    // - precision: number of digits after decimal point
    // - buffer: character array to store the converted string

    // Example: dtostrf(25.73, 4, 2, tempStr) → "25.73"
    char tempStr[10], humidStr[10];
    dtostrf(tempEvent.temperature, 4, 2, tempStr);        // Convert temperature to string
    dtostrf(humidEvent.relative_humidity, 4, 2, humidStr); // Convert humidity to string

    // Publish converted strings to respective topics
    mqttClient.publish(temp_topic, tempStr);
    mqttClient.publish(humid_topic, humidStr);

    // Print to Serial for monitoring
    Serial.print("Published Temperature to ");
    Serial.print(temp_topic);
    Serial.print(": ");
    Serial.println(tempStr);

    Serial.print("Published Humidity to ");
    Serial.print(humid_topic);
    Serial.print(": ");
    Serial.println(humidStr);

    blinkLed(); // Blink LED to indicate successful publish
  }
}
