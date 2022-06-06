/*
Verision 1.5
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
#include "ph_grav.h"             
#include "arduino_secrets.h"

WiFiClient wifiClient;
PubSubClient mqttClient(server, port, callback, wifiClient);//Local Mosquitto Connection
Gravity_pH pH = Gravity_pH(A0);

char ssid[] = SECRET_SSID;       // your network SSID (name)
char pass[] = SECRET_PASS;       // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte server[] = {192,168,0,200}; //Local Mosquitto server
int port = 1883; //the port of the MQTT broker

const char* topic = "phControl/phAdjust";
const char* outTopic = "phControl/phRead";
const char* restartTopic = "phControl/phReboot";

int phDnPIN = 2; //pin connected to pH down pump
int phUpPIN = 3; //pin connected to pH up pump

// Time interval dosing pumps run for. Test each, calibrate the output, and adjust values or concentration of the additive
int doseTimeSm = 1500;
int doseTimeMed = 3000;
int doseTimeLg = 4500;

const long interval =60000;  // Interval in which pH readings will be taken and published
const long resetInterval =3600000; // Interval for reboot loop
unsigned long previousMillis = 0; // For pH read function
unsigned long previousMillis1 = 0; //For reboot function

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

// Functions to clear and calibrate pH probe and prime dosing pumps
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
  Serial.begin(4800); // trying 4800 instead of the standard of 9600
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (WiFi.status() == WL_NO_MODULE) {      //WiFi Connection -- Start
    Serial.println("Communication with WiFi module failed!");
    while (true);  // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  while (status != WL_CONNECTED) {   // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network:
    delay(15000);  // wait 15 seconds for connection:
  }
  Serial.print("You're connected to the network ");  //WiFi Connection -- End
 
  if (mqttClient.connect( "ArduinoOG" )) {   //Local Mosquitto Connection -- Start
    // connection succeeded
    Serial.println("Connection succeeded.");
    Serial.print("Subscribing to the topic [");
    Serial.print(topic);
    Serial.println("]");
    mqttClient.subscribe(topic);
    Serial.println("Successfully subscribed to the topic.");
   
   } else {
      Serial.print("Connection failed. MQTT client state is: ");    // connection failed
      Serial.println(mqttClient.state());
   } //Local Mosquitto Connection -- End

  Serial.println(); // pH probe calibration serial commands
  Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  if (pH.begin()) {                                     
  Serial.println("Loaded EEPROM");
  Serial.println();
  }
}
                                  // Sets initial reconnection count to 0, 
int connection_count = 0;         // a digit is added to the end of the client ID for each reconnect attmpt the program has run
void reconnect() {
  while (!mqttClient.connected()) {    // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
      ++connection_count;
      Serial.println("Arduino" + connection_count);  // Attempt to connect
      String clientID = "Arduino" + String(connection_count);
    if (mqttClient.connect( clientID.c_str())) {
      Serial.println("Reconnected");
      mqttClient.subscribe(topic);    // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 15 seconds");
      delay(15000);       // Wait 15 seconds before retrying
    }
  }
}

void WiFiReconnect() {
  if (WiFi.status() == WL_NO_MODULE) {    //WiFi Connection -- Start
    Serial.println("Communication with WiFi module failed!");
    while (true);     // don't continue
  }
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
 
  while (status != WL_CONNECTED) {    //WiFi ReConnection -- Start attempt to reconnect to Wifi network:
    Serial.print("Attempting to Reconnect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network:
    delay(15000); // wait 15 seconds for connection:
  }
  Serial.print("You're Reconnected to the network ");   // you're connected now, so print out the data:
  mqttClient.publish(restartTopic, "WiFi Reconnect Function Successful!",true);
}                                                             //WiFi ReConnection -- End

void loop() {
 if (status != WL_CONNECTED) {  //check for WiFi connection and run WiFi reconnect loop if not
    WiFiReconnect();
  } else if (!mqttClient.connected()) {  //check for MQTT connection and run MQTT reconnect loop if not
    reconnect();
  } else {
  mqttClient.loop();
  callback;
  
  unsigned long currentMillis = millis();     // pH read routine
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
 }                                        // End pH read routine
 
  void(* resetFunc) (void) = 0;           //Periodic reboot function
 
  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= resetInterval){
    previousMillis1 = currentMillis1;
    mqttClient.publish(restartTopic, "Rebooting!",true);
    delay(200);
    void disconnect();
    delay(200);
    resetFunc(); //call reset
    }
  }
  void(* resetFunc) (void) = 0;        //End Periodic reboot function
 
  unsigned long currentMillis1 = millis();  //reboot routine
  if (currentMillis1 - previousMillis1 >= resetInterval){
    previousMillis1 = currentMillis1;
    mqttClient.publish(restartTopic, "Rebooting!",true);
    delay(200);
    void disconnect();
    delay(200);
    resetFunc(); //call reset 
 }                                        //End reboot routine
}
