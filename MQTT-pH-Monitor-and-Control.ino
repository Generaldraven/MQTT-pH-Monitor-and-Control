/*
Verision 1.7
Reads pH and sends reading to local MQTT broker
Will accept MQTT broker messages with "UP" or "DOWN" commands to
run pH up/down motors based on time.

This code is a WIP to optimized code and remove extra lines.

5/19/2022: Device was not connected in the morning so I added a "wifiReconnect" function.  
It seems to be working but I have not watched it run on the serial monitor

5/22/2022: swapped position of the two reconnect functions so that it checks for and attempts 
to reconnect to wifi before checking and attempting to connect to the MQTT broker.  
Added small/med/large dose options.
Removed a bunch of functions to print various wifi data since I know or assign that info now (IP address, MAC addres, etc.)
Added an MQTT publish line to send WiFi.getTime(t); but unsure what it will return.

6/6/2022: Updated WiFiReconnect function so it now works
6/12/2022: Added PWM light control to callback
*/

#include <PubSubClient.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <ezTime.h>
#include "ph_grav.h"             
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;       // your network SSID (name)
char pass[] = SECRET_PASS;       // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte server[] = {192,168,0,200}; //Local Mosquitto server
int port = 1883; //the port of the MQTT broker

Gravity_pH pH = Gravity_pH(A0);
WiFiClient wifiClient;

unsigned long previousMillis = 0;
const char* topic = "phControl/phAdjust";
const char* outTopic = "phControl/phRead";
const char* adminTopic = "phControl/Admin";
const long interval =60000;                                      // Interval to check take a pH reading in ms

int phDnPIN = 2;                                                                  // Defines pins for pH up and pH down pins
int phUpPIN = 3;
int doseTimeSm = 3000;
int doseTimeMed = 4500;
int doseTimeLg = 6000;
int PWMpin = 8;

void callback(char* topic, byte* payload, unsigned int length) {                    // Handles messages arrived on subscribed topic(s)
  String result;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    result += (char)payload[i];
  }
  Serial.println("");

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
      uint8_t lightSetting = result.toInt();
      Serial.println(lightSetting);
      analogWrite(PWMpin, lightSetting);
  } else {
    Serial.println("Do nothing");
  }
}

PubSubClient mqttClient(server, port, callback, wifiClient);     //Local Mosquitto Connection (has to come after server and callback are defined)

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

int wifiConnectionCount = 0;
void WiFiReconnect() {                                              // Checks for WiFi connection and reconnects if needed
  int status;
  status=WiFi.status();
  if (status==WL_DISCONNECTED || status==WL_CONNECTION_LOST) {
    Serial.println("Launching WiFi reconnect function");
    while ( status != WL_CONNECTED){
    status = WiFi.begin(ssid, pass);
    delay(10000);
    ++wifiConnectionCount;
    Serial.println(wifiConnectionCount);
    }
  }
}

int connection_count = 0;                                         // MQTT reconnection function
void MqttReconnect() {
  while (!mqttClient.connected()) {                               // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");                // Attempt to connect
      ++connection_count;
      Serial.println("Arduino" + String (connection_count));
      String clientID = "Arduino" + String(connection_count);
    if (mqttClient.connect( clientID.c_str())) {
      Serial.println("Reconnected");
      mqttClient.subscribe(topic);                                 // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 15 seconds");
      delay(15000);                                                 // Wait 15 seconds before retrying
    }
  }
  String wifiReconPayload = "WiFi Recon Count: " + int (wifiConnectionCount);
  mqttClient.publish(adminTopic,String(wifiReconPayload).c_str(), true);
}

void setup() {
  pinMode(phUpPIN, OUTPUT);
  pinMode(phDnPIN, OUTPUT);
  
  Serial.begin(9600);                                                  //Initialize serial and wait for port to open:
  while (!Serial) {;}                                                  // wait for serial port to connect. Needed for native USB port only

  if (WiFi.status() == WL_NO_MODULE) {                                 //WiFi Connection -- Start check for the WiFi module:
    Serial.println("Communication with WiFi module failed!");
    while (true);                                                       // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  while (status != WL_CONNECTED) {                                       // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    delay(15000);                                                         // wait 15 seconds for connection:
  }

  Serial.print("You're connected to the network ");
                                                                          //WiFi Connection -- End
  if (mqttClient.connect( "ArduinoOG" )) {                          //Local Mosquitto Connection -- Start
    Serial.println("Connection succeeded.");                        // connection succeeded
    Serial.print("Subscribing to the topic [");
    Serial.print(topic);
    Serial.println("]");
    mqttClient.subscribe(topic);
    Serial.println("Successfully subscribed to the topic.");
   } else {
      Serial.print("Connection failed. MQTT client state is: ");    // connection failed
      Serial.println(mqttClient.state());
   }                                                                //Local Mosquitto Connection -- End
                                   
  Serial.println();                                                 // pH probe calibration serial commands
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {                                     
  Serial.println("Loaded EEPROM");
  Serial.println();
  }
}                                                                   // pH probe calibration serial commands -- End

void loop() {
 WiFiReconnect();
 if (!mqttClient.connected()) {
  MqttReconnect();
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

    Serial.print("Sending message to topic: ");
    Serial.println(outTopic);

    float newpH = pH.read_ph();

      Serial.print("pH: ");
      Serial.println(String(newpH).c_str());
      mqttClient.publish(outTopic, String(newpH).c_str(), true); // Might add time stamp to see when sent (MQTT client stamps time received)

    Serial.println();
  } 
 }
}
