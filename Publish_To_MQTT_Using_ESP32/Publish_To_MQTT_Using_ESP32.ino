/*************************************************************
  Project : ESP32-S3 MQTT Publisher
  Description : Publishes "Hello World!" every 1 second 
                to topic "myMessage" using Mosquitto Broker
  Hardware : ESP32-S3
  Author : Age of Robotics
*************************************************************/

#include <WiFi.h>          // WiFi library for ESP32
#include <PubSubClient.h>  // MQTT library

/************** WiFi Network Credentials ****************/
const char* ssid = "XXXX";        // <-- Replace with your WiFi SSID
const char* password = "XXXXXXXXXXXX"; // <-- Replace with your WiFi Password

/************** MQTT Broker Settings *******************/
const char* mqtt_server_ip = "XXX.YYY.ZZZ.WWW";  // <-- Replace with your Broker IP address
const int mqtt_port = 1883;                     // Default MQTT port

/************** Object Declarations ********************/
WiFiClient wifiClient;         // Create a WiFi client
PubSubClient mqttClient(wifiClient);  // Create an MQTT client using WiFi

/************** Function to Connect to WiFi *************/
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  // Wait until WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/************** Function to Reconnect to MQTT Broker *************/
void reconnectToMQTT() {
  // Keep trying to connect until successful
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT Client...");

    // Attempt to connect with a client ID
    if (mqttClient.connect("ESP32_S3_Client")) {
      Serial.println("connected to MQTT broker!");
    }
    else {
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait before retrying
    }
  }
}

/**************Blink Led Function************/
void blinkLed(){
  digitalWrite(13,HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

/************** Arduino Setup Function *************/
void setup() {
  Serial.begin(115200); // Start Serial Monitor
  delay(100);           // Short delay for stability

  pinMode(13, OUTPUT); //Setup LED Pin as Outut

  connectToWiFi();       // Connect to WiFi
  mqttClient.setServer(mqtt_server_ip, mqtt_port); // Set MQTT Broker
}

/************** Arduino Loop Function *************/
void loop() {
  // Ensure connection to MQTT server
  if(!mqttClient.connected()) {
    reconnectToMQTT();
  }
  mqttClient.loop(); // Keep the client alive

  // Publishing message every 1 second
  static unsigned long lastPublishTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastPublishTime >= 1000){ // Check if 1 second passed
    lastPublishTime = currentMillis;             // Update the last publish time

    String outgoingMessage = "Hello World!";     // Prepare the message
    Serial.print("Publishing to topic 'myMessage': ");
    Serial.println(outgoingMessage);
    mqttClient.publish("myMessage", outgoingMessage.c_str()); // Publish to topic
    blinkLed();
  }
}
