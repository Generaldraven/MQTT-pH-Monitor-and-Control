/*
Verision 1.0
Basic functional starting point.
Reads pH and sends reading to local MQTT broker
Will accept MQTT broker messages with "UP" or "DOWN" commands to
run pH up/down motors based on time.
This code should be optimized and extra lines removed in other branches.
*/

#include <PubSubClient.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <ezTime.h>
#include "ph_grav.h"             
 Gravity_pH pH = Gravity_pH(A0);
#include "arduino_secrets.h"


char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte server[] = {192,168,0,200}; //Local Mosquitto server
int port = 1883; //the port of the MQTT broker
const char* topic = "phControl/phAdjust";
const char* outTopic = "phControl/phRead";
int phDnPIN = 5;
int phUpPIN = 4;
int doseTime = 15000;


// Handles messages arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
  String result;
  Serial.print("Message arrived [");
  Serial.print("phControl/phAdjust");
  Serial.print("]: ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    result += (char)payload[i];
  }
  Serial.println("");

  //Act on the message
  if (result.equalsIgnoreCase("UP")) {
    Serial.println("Adjust pH UP!");
    digitalWrite(phUpPIN, HIGH);
    delay(doseTime);
    digitalWrite(phUpPIN, LOW);
  } else if (result.equalsIgnoreCase("DOWN")) {
    Serial.println("Adjusting pH DOWN!");
    digitalWrite(phDnPIN, HIGH);
    delay(doseTime);
    digitalWrite(phDnPIN, LOW);
  } else {
    Serial.println("Do nothing");
  }
}

WiFiClient wifiClient;
PubSubClient mqttClient(server, port, callback, wifiClient);//Local Mosquitto Connection
//mqttClient.setCallback(callback);

const long interval =60000;
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
  else if (strcmp(string, "PHUP") == 0) {
    digitalWrite(phUpPIN, HIGH);
    delay(doseTime);
    digitalWrite(phUpPIN, LOW);
  }   // Dose of pH up
  else if (strcmp(string, "PHDN") == 0) {
    digitalWrite(phDnPIN, HIGH);
    delay(doseTime);
    digitalWrite(phDnPIN, LOW);
   }   // Dose of pH down
   mqttClient.setStream(Serial);
  mqttClient.setCallback(callback);
}

void setup() {
  mqttClient.setStream(Serial);
  mqttClient.setCallback(callback);
  pinMode(phUpPIN, OUTPUT);
  pinMode(phDnPIN, OUTPUT);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  //WiFi Connection -- Start
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
  //WiFi Connection -- End
   
  //Local Mosquitto Connection -- Start
  if (mqttClient.connect("arduino")) {
    // connection succeeded
    Serial.println("Connection succeeded.");
    Serial.print("Subscribing to the topic [");
    Serial.print("phControl/phAdjust");
    Serial.println("]");
    mqttClient.subscribe("phControl/phAdjust");
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

// pH probe calibration serial commands
  Serial.println();
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {                                     
  Serial.println("Loaded EEPROM");
  Serial.println();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduino")) {
      Serial.println("connected");
      // Subscribe
      mqttClient.subscribe("phControl/phAdjust");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

}


void loop() {
 if (!mqttClient.connected()) {
    reconnect();
  } else {
  mqttClient.loop();
  callback;
  
  mqttClient.setStream(Serial);
  mqttClient.setCallback(callback);
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
    time_t t = WiFi.getTime();
    Serial.print("Sending message to topic: ");
    Serial.println("phControl/phRead");
    Serial.println(UTC.dateTime(t));
    //Serial.print("pH: ");
    //Serial.println(pH.read_ph());

    float newpH = pH.read_ph();

      Serial.print("pH: ");
      Serial.println(String(newpH).c_str());
      mqttClient.publish("phControl/phRead", String(newpH).c_str(), true);

    Serial.println();

    count++;
  }
  mqttClient.setStream(Serial);
  mqttClient.setCallback(callback);
 }
}

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
