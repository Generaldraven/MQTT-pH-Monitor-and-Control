#include <Arduino.h>
#include "arduino_secrets.h"
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "ph_grav.h"
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DS18B20.h>
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"
#include "DallasTemperature.h"
#include "Adafruit_ADS1X15.h"

#define ONE_WIRE_BUS 5

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 WTsensor(&oneWire);

DFRobot_ESP_EC ec;
Adafruit_ADS1115 ads;

float voltage, ecValue, temperature = 18;

#define DHTPIN 15
#define DHTTYPE DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte MQTTserver[] = {192,168,0,200}; //Local Mosquitto server
int port = 1883; //the port of the MQTT broker

const char* adminTopic = "Hydro/Admin";

Gravity_pH pH = Gravity_pH(34);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long previousMillis = 0;
const char* topic = "Hydro/Adjust";
const char* pHTopic = "Hydro/phRead";
const char* ATTopic = "Hydro/ATRead";
const char* AHTopic = "Hydro/AHRead";
const char* WTTopic = "Hydro/WTRead";
const char* TDSTopic = "Hydro/TDSRead";
const long interval =5000; // Interval to check take a pH reading in ms

int phDnPIN = 33;  // Defines pins for pH up and pH down pins
int phUpPIN = 25;
int doseTimeSm = 1500;
int doseTimeMed = 3000;
int doseTimeLg = 4500;

// the number of the LED pin
const int ledPin = 12;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 1;
const int resolution = 8;

AsyncWebServer server(80);

void callback(char* topic, byte* payload, unsigned int length) {
  String result;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    result += (char)payload[i];
  }

  Serial.println();

  if (result == ("UP MED")) {                                           // Commands received via MQTT "topic"
    Serial.println("Adjust pH UP a medium amount!");
    digitalWrite(phUpPIN, HIGH);
    delay(doseTimeMed);
    digitalWrite(phUpPIN, LOW);
  } else if (result == ("DOWN MED")) {
    Serial.println("Adjusting pH DOWN a medium amount!");
    digitalWrite(phDnPIN, HIGH);
    delay(doseTimeMed);
    digitalWrite(phDnPIN, LOW);
  } else if (result == ("UP SM")) {
    Serial.println("Adjust pH UP a small amount!");
    digitalWrite(phUpPIN, HIGH);
    delay(doseTimeSm);
    digitalWrite(phUpPIN, LOW);
  } else if (result == ("DOWN SM")) {
    Serial.println("Adjusting pH DOWN a small amount!");
    digitalWrite(phDnPIN, HIGH);
    delay(doseTimeSm);
    digitalWrite(phDnPIN, LOW);
  } else if (result == ("UP LG")) {
    Serial.println("Adjust pH UP a large amount!");
    digitalWrite(phUpPIN, HIGH);
    delay(doseTimeLg);
    digitalWrite(phUpPIN, LOW);
  } else if (result == ("DOWN LG")) {
    Serial.println("Adjusting pH DOWN a lage amount!");
    digitalWrite(phDnPIN, HIGH);
    delay(doseTimeLg);
    digitalWrite(phDnPIN, LOW);
  } else if (result.endsWith("LIGHT")) {
     Serial.println("Setting light level to ");
     byte lightSetting = result.toInt();
     ledcWrite(ledChannel, lightSetting);
     Serial.println(lightSetting);
  } else {
    Serial.println("Do nothing");
  }
}

int count = 0;                                 // Defines Serial commands used to calibrate pH probe as well as any similar
uint8_t user_bytes_received = 0;               // untility functions for future components 
const uint8_t bufferlen = 32;                   
char user_data[bufferlen];  

void parse_cmd(char* string) {                   
  strupr(string);                                
  if (strcmp(string, "CAL,7") == 0) {       
    pH.cal_mid();                                
    Serial.println("MID CALIBRATED");
  }
  else if (strcmp(string, "CAL,4") == 0) {            
    pH.cal_low();                                
    Serial.println("LOW CALIBRATED");
  }
  else if (strcmp(string, "CAL,10") == 0) {      
    pH.cal_high();                               
    Serial.println("HIGH CALIBRATED");
  }
  else if (strcmp(string, "CAL,CLEAR") == 0) { 
    pH.cal_clear();                              
    Serial.println("CALIBRATION CLEARED");
  }
  else if (strcmp(string, "PHUP") == 0) {
    digitalWrite(phUpPIN, HIGH);
    delay(doseTimeSm);
    digitalWrite(phUpPIN, LOW); // Dose of pH up
  }   
  else if (strcmp(string, "PHDN") == 0) {
    digitalWrite(phDnPIN, HIGH);
    delay(doseTimeSm);
    digitalWrite(phDnPIN, LOW); // Dose of pH down
  } 
mqttClient.setStream(Serial);
mqttClient.setCallback(callback);    
}

int connection_count = 0;                                         // MQTT reconnection function
void reconnect() {
  while (!mqttClient.connected()) {                               // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");                // Attempt to connect
      ++connection_count;
      Serial.println("Arduino" + String (connection_count));
      String clientID = "Arduino" + String(connection_count);
    if (mqttClient.connect( clientID.c_str())) {
      Serial.println("Reconnected");
      mqttClient.subscribe(adminTopic);                                 // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 15 seconds");
      delay(15000);                                                 // Wait 15 seconds before retrying
    }
  }
}

void WiFiReconnect() {                                              // Checks for WiFi connection and reconnects if needed
  int status;
  status=WiFi.status();
  if (status==WL_DISCONNECTED || status==WL_CONNECTION_LOST) {
    Serial.println("Launching WiFi reconnect function");
    while ( status != WL_CONNECTED){
    status = WiFi.begin(ssid, password);
    delay(10000);
    }
  }
}

void setup(void) {
  pinMode(phUpPIN, OUTPUT);
  pinMode(phDnPIN, OUTPUT);
  ledcSetup(ledChannel, freq, resolution);

  dht.begin();
  sensor_t sensor;
  
  ledcAttachPin(ledPin, ledChannel);   // attach the channel to the GPIO to be controlled

  Serial.begin(115200);
	EEPROM.begin(32);
	ads.setGain(GAIN_ONE);
	ads.begin();

  Serial.println(__FILE__);
  Serial.print("DS18B20 Library version: ");
  Serial.println(DS18B20_LIB_VERSION);

  WTsensor.begin();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is a sample response.");
  });

  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  mqttClient.setServer(MQTTserver, port);
  mqttClient.setCallback(callback);

  //Local Mosquitto Connection -- Start
  if (mqttClient.connect("arduino_OG")) {
    // connection succeeded
    Serial.println("Connection succeeded.");
    Serial.print("Subscribing to the topic [");
    Serial.print(topic);
    Serial.println("]");
    mqttClient.subscribe(topic);
    Serial.println("Successfully subscribed to the topic.");
    //mqttClient* setCallback(callback());
   
   } else {
      // connection failed
      // mqttClient.state() will provide more information
      // on why it failed.
      Serial.print("Connection failed. MQTT client state is: ");
      Serial.println(mqttClient.state());
   }
  //Local Mosquitto Connection -- End
}

void loop(void) {
  WiFiReconnect();
  if (!mqttClient.connected()) {
    reconnect();
  } else {
  mqttClient.loop();
  callback;

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
 
    if (Serial.available() > 0) {                                                      
     user_bytes_received = Serial.readBytesUntil(13, user_data, sizeof(user_data));   
    }

    if (user_bytes_received) {                                                      
     parse_cmd(user_data);                                                          
     user_bytes_received = 0;                                                        
     memset(user_data, 0, sizeof(user_data));                                         
    }  

    Serial.print("Sending message: ");

    float newpH = pH.read_ph();

      Serial.print("pH: ");
      Serial.println(String(newpH).c_str());
      mqttClient.publish(pHTopic, String(newpH).c_str(), true); // Might add time stamp to see when sent (MQTT client stamps time received)

    Serial.println();
    delay(500);
    // Ambient Temp/Humidity Stuff
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
      mqttClient.publish(ATTopic, String(event.temperature).c_str(), true);
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    } else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
      mqttClient.publish(AHTopic, String(event.relative_humidity).c_str(), true);
    }
    delay(500);
    WTsensor.requestTemperatures();

      while (!WTsensor.isConversionComplete());  // wait until sensor is ready

      Serial.print("Temp: ");
      Serial.println(WTsensor.getTempC());
      mqttClient.publish(WTTopic, String(WTsensor.getTempC()).c_str(), true);
      delay(500);
    //TDS stuff
    {
      {
      voltage = ads.readADC_SingleEnded(0) / 10;
      Serial.print("voltage:");
      Serial.println(voltage, 4);

      //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
      Serial.print("temperature:");
      Serial.print(temperature, 1);
      Serial.println("^C");

      ecValue = ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation
      Serial.print("EC:");
      Serial.print(ecValue, 4);
      Serial.println("ms/cm");
      }
	  ec.calibration(voltage, temperature); // calibration process by Serail CMD
    }

    float readTemperature(sensors_event_t); 
    {
    //add your code here to get the temperature from your temperature sensor
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
      }
      else {
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
      }
    } 
  mqttClient.publish(TDSTopic, String(ecValue, 4).c_str(), true);
  }
 }
}
