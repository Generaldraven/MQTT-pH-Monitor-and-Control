/*
Verision 1.4
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

6/4/2022: Added reset loop so Arduino resets every hour as a work-around until I can figure out why it won't reconnect to WiFi
*/

#include <PubSubClient.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <ezTime.h>
#include <avr/wdt.h>
#include "ph_grav.h"             
 Gravity_pH pH = Gravity_pH(A0);
#include "arduino_secrets.h"


char ssid[] = SECRET_SSID;       // your network SSID (name)
char pass[] = SECRET_PASS;       // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte server[] = {192,168,0,200}; //Local Mosquitto server
int port = 1883; //the port of the MQTT broker
const char* topic = "phControl/phAdjust";
const char* outTopic = "phControl/phRead";
int phDnPIN = 2;
int phUpPIN = 3;
int doseTimeSm = 1500;
int doseTimeMed = 3000;
int doseTimeLg = 4500;

// Handles messages arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
  String result;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    result += (char)payload[i];
  }
  Serial.println("");

  //Act on the message
  if (result == ("UP MED")) {
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
  } else {
    Serial.println("Do nothing");
  }
}

WiFiClient wifiClient;
PubSubClient mqttClient(server, port, callback, wifiClient);//Local Mosquitto Connection

const long interval =60000;
const long resetInterval =3600000;
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

void setup() {
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

    // wait 15 seconds for connection:
    delay(15000);
  }

  Serial.print("You're connected to the network ");
  //WiFi Connection -- End
   
  //Local Mosquitto Connection -- Start
  if (mqttClient.connect("arduino")) {
    // connection succeeded
    Serial.println("Connection succeeded.");
    Serial.print("Subscribing to the topic [");
    Serial.print(topic);
    Serial.println("]");
    mqttClient.subscribe(topic);
    Serial.println("Successfully subscribed to the topic.");
   
   } else {
      // connection failed
      Serial.print("Connection failed. MQTT client state is: ");
      Serial.println(mqttClient.state());
   } //Local Mosquitto Connection -- End

// pH probe calibration serial commands
  Serial.println();
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {                                     
  Serial.println("Loaded EEPROM");
  Serial.println();
  }
}

int connection_count = 0;
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
        ++connection_count;
        Serial.println("arduino" + String(connection_count));
        String clientID = String("arduino") + String(connection_count);
    if (mqttClient.connect( clientID.c_str() )) {
      Serial.println("Reconnected");
      mqttClient.subscribe(topic);    // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 15 seconds");
      // Wait 15 seconds before retrying
      delay(15000);
    }
  }
}

void WiFiReconnect() {
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
  //WiFi ReConnection -- Start attempt to reconnect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to Reconnect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network:
    delay(15000); // wait 15 seconds for connection:
  }
  Serial.print("You're Reconnected to the network "); // you're connected now, so print out the data:
  //WiFi ReConnection -- End
}

void loop() {
 if (status != WL_CONNECTED) {
    WiFiReconnect();
  } else if (!mqttClient.connected()) {
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

    Serial.print("Sending message to topic: ");
    Serial.println(outTopic);

    float newpH = pH.read_ph();

      Serial.print("pH: ");
      Serial.println(String(newpH).c_str());
      mqttClient.publish(outTopic, String(newpH).c_str(), true); // Might add time stamp to see when sent (MQTT client stamps time received)

    Serial.println();
 }
   //unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= resetInterval){
    previousMillis = currentMillis;
    reboot(); //call reset   // reset command
    }
  }
}
void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}
