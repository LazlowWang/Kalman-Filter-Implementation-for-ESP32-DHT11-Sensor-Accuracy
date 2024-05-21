#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHT.h> 
#include <Arduino.h> // Arduino standard library

DHT dht(26, DHT11); //DHT function from library, PIN 26 on ESP, DHT11 type sensor

// Kalman filter variables for temperature
float varMeasure_temp = 1e-5; // Measurement variance for temperature
float varProcess_temp = 1e-8; // Process variance for temperature
float k_temp = 1.0; // Estimate correlation matrix of error for temperature
float X_est_temp = 0.0; // Estimated temperature
float G_temp = 0.0 ; // Kalman gain for temperature
float k_temp_pred = 0.0;  /// Update correlation matrix of error
float sigma_temp = 5;

// Kalman filter variables for humidity
float varMeasure_humidity = 1e-5;  // Measurement variance for humidity
float varProcess_humidity = 1e-8; // Process variance for humidity
float k_humidity = 1.0; // Estimate correlation matrix of error for humidity
float X_est_humidity = 0.0; // Estimated humidity
float G_humidity = 0.0 ; // Kalman gain for humidity
float k_humidity_pred = 0.0;  // Update correlation matrix of error
float sigma_humidity = 5;

const char *ssid = "traphouse:)$$";
const char *password = "idontknowmaybe2";
const char* mqtt_server = "10.0.0.156"; //raspi

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;

void setup() {            //customize part
  dht.begin(); 
  delay (500);
  Serial.begin(115200); //Serial BaudRate

  // Connect to a WiFi network
  Serial.println("");
  Serial.print("Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  
  // check connect every half second
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  //set MQTT Server details
  client.setServer(mqtt_server, 1883); //port of the MQTT_Server
}

// Function to get noise
float getNoise(float noiseLevel) {
    return (random(1000) / 500.0 - 1) * noiseLevel; // Generates values between -noiseLevel and +noiseLevel
}

void loop() {
  
  //if client is not connected 
  if (!client.connected()) {
    reconnect();
  }

  //Real Main loop starts// // JSON file may have error if the lenght of the JSON is too short
  StaticJsonDocument<200> doc; //static Jason file readed from sensors
  char output[200]; // char array which will store the Jason message into the MQTT Server

  //Dealy between sending messages
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

 //First order Kalman filter implementation
    
 //Sensors operation  
  float temp = dht.readTemperature();  // Read temperature
  float humidity = dht.readHumidity(); // Read humidity

  // Kalman filter process for temperature
  // (F=1). Temperature does vary drastically; No control input to the system.
  float temp_noisy = temp + getNoise(sigma_temp); // Add noise to temperature measurement
  k_temp_pred = k_temp + varProcess_temp; // Predicted correlation matrix of error
  G_temp = k_temp_pred / (k_temp_pred + varMeasure_temp); // Kalman gain
  k_temp = k_temp_pred *(1 - G_temp) ; // Update correlation matrix of error
  X_est_temp = X_est_temp + G_temp * (temp_noisy - X_est_temp); // Update estimate with measurement
 float EK_temp = abs(temp - X_est_temp); //Error of Kalman Filtered Temperature Value
 float EN_temp = abs(temp - temp_noisy); //Error of Noisy Measured Temperature Value

  // Kalman filter process for humidity
  //(F=1). Humidity does vary drastically; No control input to the system.
  float humidity_noisy = humidity + getNoise(sigma_humidity); // Add noise to humidity measurement
  k_humidity_pred = k_humidity + varProcess_humidity; // Predicted correlation matrix of error
  G_humidity = k_humidity_pred / (k_humidity_pred + varMeasure_humidity); // Kalman gain
  k_humidity = k_humidity_pred * (1 - G_humidity); // Update correlation matrix of error
  X_est_humidity = X_est_humidity + G_humidity * (humidity_noisy - X_est_humidity); // Update estimate with measurement
  float EK_humidity = abs(humidity - X_est_humidity);  //Error of Kalman Filtered Humidity Value
  float EN_humidity = abs(humidity - humidity_noisy);  // Error of Noisy Measured Humidity Value
  
  // Output the raw and filtered temperature values
  Serial.print("Temp: ");
  Serial.print(temp); 
  Serial.print("\t");
  Serial.print("Filtered Temp: ");
  Serial.print(X_est_temp); 
  Serial.print("\t");


  // Output the raw and filtered humidity values
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("\t");
  Serial.print("Filtered Humidity: ");
  Serial.print(X_est_humidity);
  Serial.println("\t");


  delay(10);

    //Add variable to JSON documents. Each value has a key associated to it to indicate what value has been stored.
    
    doc["t"] = temp; 
    doc["p"] = X_est_temp;
    doc["x"] = temp_noisy;
    doc["h"] = humidity;
    doc["g"] = X_est_humidity;
    doc["y"] = humidity_noisy;
    doc["m"] = EK_temp;
    doc["n"] = EN_temp;
    doc["c"] = EK_humidity;
    doc["d"] = EN_humidity;
    
    //Serialise JSON document into String type message that can be sent across the network
    //Use client.publish function, which takes the argument of the message
    
    serializeJson(doc, output);
    Serial.println(output);
    client.publish("/home/sensors", output);
  }
    
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}
