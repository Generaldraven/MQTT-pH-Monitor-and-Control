#include <SPI.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <ezTime.h>
#include <ArduinoMqttClient.h>
#include "ph_grav.h"             
 Gravity_pH pH = Gravity_pH(A0);

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

WiFiSSLClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "28f063b79e144239854f297f2e91d118.s2.eu.hivemq.cloud";
int        port     = 8883;
const char topic[]  = "phControl";

const long interval = 10000;
unsigned long previousMillis = 0;

int count = 0;

uint8_t user_bytes_received = 0;                
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
}

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  mqttClient.setId("Arduino_MQTT_Client");

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword("XXXX", "XXXX");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println("phControl/phAdjust");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("phControl/phAdjust");

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe("phControl/phAdjust");

  Serial.print("Waiting for messages on topic: ");
  Serial.println("phControl/phAdjust");
  Serial.println();
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {                                     
  Serial.println("Loaded EEPROM");
  }
  // setup for pH up/down
  pinMode(4, OUTPUT);      // set the PH_UP pin mode
  pinMode(5, OUTPUT);      // set the PH_DN pin mode
}

void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

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
  
  Serial.println(pH.read_ph()); 

    time_t t = WiFi.getTime();
    Serial.print("Sending message to topic: ");
    Serial.println("phControl/phRead");
    Serial.println(UTC.dateTime(t));
    Serial.print("pH: ");
    Serial.println(pH.read_ph());

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage("phControl/phRead");
    mqttClient.print(UTC.dateTime(t));
    mqttClient.print("pH: ");
    mqttClient.print(pH.read_ph());
    mqttClient.endMessage();

    Serial.println();

    count++;
  }                                                     
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());}
  if (mqttClient.messageTopic("phControl/phAdjust").c_str() = "PHUP") { digitalWrite(4, HIGH);
          delay(5000);
          digitalWrite(4, LOW);
 }   // Dose of pH up
    if (mqttClient = "off") { digitalWrite(5, HIGH);       
          delay(5000);
          digitalWrite(5, LOW); 
 } // Dose of pH down
    Serial.println();
    Serial.println("-----------DONE!-----------");  
  }
  Serial.println();

  Serial.println();
}