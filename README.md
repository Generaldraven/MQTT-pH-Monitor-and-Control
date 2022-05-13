# MQTT-pH-Monitor-and-Control

Project to use Arduino Uno WiFi Rev2 to monitor pH reading and send them to an MQTT broker on my local network
Will also accept pH up/down commands from that broker send via any device on my local network

Readings should be taken once per minute and logged on the broker
Adjustment commands will be somewhat rare, maybe a handful per day
pH up/down commands _must_ be sent and executed only once
