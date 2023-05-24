#ifndef _WIFI_H
#define _WIFI_H

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

//the initial wifi status
int status = WL_IDLE_STATUS;

//WiFiUDP object used for the communication
WiFiUDP Udp;

//your network name (SSID) and password (WPA)
char ssid[] = "The Lanisters send their regards";            
char pass[] = "11992288"; 

//local port to listen on
int localPort = 3002;                               

//IP and port for the server
IPAddress serverIPAddress(172, 20, 10, 2);
int serverPort = 3001;       

//setup: runs only once
void setupWifi() {

  //check the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    
    //don't continue
    while (true);
  }

  //attempt to connect to WiFi network
  while (status != WL_CONNECTED) {

    //connect to WPA/WPA2 network
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    //wait 10 seconds for connection
    delay(10000);
  }
  
  Serial.println("Connected to WiFi");
  
  //if you get a connection, report back via serial:
  Udp.begin(localPort);
}


//listens for incoming UDP messages
int listenForUDPMessage() {
  //on package received
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    //buffer for incoming packages
    char packetBuffer[256]; 
  
    //read the packet into packetBufffer
    int msgLength = Udp.read(packetBuffer, 255);
    if (msgLength > 0) {
      packetBuffer[msgLength] = NULL;
    }

    //print message from packet
    Serial.print("\nReceived message: ");
    Serial.println(packetBuffer);

    //convert message value to int
    int messageValueAsInt = atoi(packetBuffer);

    return messageValueAsInt;
    //send acknowledgement message
    //sendUDPMessage(Udp.remoteIP(), Udp.remotePort(), "ARDUINO: message was received");
  } else {
    return NULL; 
  }
}

//sends a message to the server (UDP)
void sendUDPMessage(IPAddress remoteIPAddress, int remoteport, String message) {
  Serial.println("sendUDPMessageToServer");

  //get message string length (+1 to store a null value indicating the end of the message)
  int messageLength = message.length() + 1;
  
  //create char array 
  char messageBuffer[messageLength];

  //copy string message to char array
  message.toCharArray(messageBuffer, messageLength);

  //send the packet to the server
  Udp.beginPacket(remoteIPAddress, remoteport);
  Udp.write(messageBuffer);
  Udp.endPacket(); 
}


#endif