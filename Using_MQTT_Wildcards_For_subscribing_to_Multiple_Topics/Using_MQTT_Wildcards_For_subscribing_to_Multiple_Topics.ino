/*************************************************************
  Project : ESP32 MQTT Wildcard Subscriber
  Description : Subscribes to "home/+/temperature"
                Displays temperature data from kitchen or hall
                Prints topic and message to Serial Monitor
  Hardware : ESP32
  Author : 
*************************************************************/

#include <WiFi.h>           // WiFi library for ESP32
#include <PubSubClient.h>   // MQTT library

/************** WiFi Network Credentials ****************/
const char* ssid = "XXX";           // <-- Replace with your WiFi SSID
const char* password = "XXXXX";  // <-- Replace with your WiFi Password

/************** MQTT Broker Settings *******************/
const char* mqtt_server_ip = "192.168.XXX.YYY";  // <-- Replace with your Broker IP address
const int mqtt_port = 1883;                     // Default MQTT port

/************** Object Declarations ********************/
WiFiClient wifiClient;               // Create a WiFi client
PubSubClient mqttClient(wifiClient); // Create an MQTT client using WiFi

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

/************** Function to Handle Incoming MQTT Messages *************/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage = "";

  // Convert byte array payload to String
  for (unsigned int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }

  incomingMessage.trim(); // Remove any whitespace

  // Print the topic and message to Serial Monitor
  Serial.print("Message received on topic [");
  Serial.print(topic);
  Serial.print("] : ");
  Serial.println(incomingMessage);
}

/************** Function to Reconnect to MQTT Broker *************/
void reconnectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT Client...");

    // Attempt to connect with a unique client ID
    if (mqttClient.connect("ESP32_Wildcard_Client")) {
      Serial.println("connected to MQTT broker!");
      //do not uncomment the following two lines together
      
      //Uncomment following line for Single Level wildcard
      mqttClient.subscribe("home/+/temperature"); // Subscribe using single-level wildcard
      //Uncomment following line for multi level wildcard
      //mqttClient.subscribe("home/kitchen/#"); // Subscribe using multi-level wildcard
    }
    else {
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait before retrying
    }
  }
}

/************** Arduino Setup Function *************/
void setup() {
  Serial.begin(115200);              // Start Serial Monitor
  delay(100);                        // Short delay for stability

  connectToWiFi();                              // Connect to WiFi
  mqttClient.setServer(mqtt_server_ip, mqtt_port); // Set MQTT Broker details
  mqttClient.setCallback(mqttCallback);        // Set the callback function
}

/************** Arduino Loop Function *************/
void loop() {
  // Ensure connection to MQTT server
  if (!mqttClient.connected()) {
    reconnectToMQTT();
  }

  mqttClient.loop(); // Handle incoming MQTT messages
}
