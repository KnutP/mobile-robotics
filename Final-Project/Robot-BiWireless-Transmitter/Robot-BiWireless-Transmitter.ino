/*Robot-BiWireless.ino
  Authors: Carlotta Berry
  modified: 02/10/17
  This program will show how to use the Arduino Mega on the robot with the Arduino Uno
  attached to the robot to create bi-directional wireless communication.  This will be used
  to send topological path commands from the laptop to the robot, also to send start and goal positions
  from the laptop to the robot, to receive localization information from the robot to the laptop, and
  to receive a map from the robot to the laptop
  For testing:
    - You can connect both devices to your computer
    - open up 2 instances of the Arduino IDE and put the same program on both
    - upload Mega send code to the robot microcontroller
    - upload Uno receive code on the laptop microcontroller
    - open both serial monitors on your laptop and test the communication

*** HARDWARE CONNECTIONS *****
   https://www.arduino.cc/en/Hacking/PinMapping2560
   Arduino MEGA nRF24L01 connections  *********ROBOT CONNECTION ******************
    CE  pin 7         CSN   pin 8
    VCC 3.3 V         GND GND
    MISO  pin 50      MOSI  pin 51
    SCK pin 52

   http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   http://www.theengineeringprojects.com/2015/07/interfacing-arduino-nrf24l01.html

   Arduino Uno nRF24L01 connections *************LAPTOP CONNECTION **************
   CE  pin 7        CSN   pin 8
   VCC 3.3 V        GND GND
   MOSI pin 11      MISO pin 12
   SCK pin 13
*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>//include wireless transceiver library

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

//variables
boolean transmit = true;              //set variable to send or receive data (use same code for both devices but change variable)
boolean uno = true;                   //set variable for type of microcontroller sending (uno-true-laptop,uno-false-robot)
RF24 radio(CE_PIN, CSN_PIN);          //create instance of radio object
#define team_channel 14              //set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
byte addresses[][6] = {"1Node", "2Node"};//unique address between transmitter and receiver
uint8_t data[1];                        //variable to hold transmit data
uint8_t incoming[1];                        //variable to hold receive data
uint8_t state[] = {0, 0};               //variable to hold receive data position
uint8_t mapDat[4][4];                   //variable to hold receive data MAP
uint8_t lastSend;                      // Store last send time

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
char receivedChar;

boolean newData = false;

String slrt = "";

void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
//    Serial.println("***********************************");
//    Serial.println("....Starting nRF24L01 Transmit.....");
//    Serial.println("***********************************");
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
//    Serial.println("***********************************");
//    Serial.println("....Starting nRF24L01 Receive.....");
//    Serial.println("***********************************");
  }

}

void loop() {
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
    readSerial();
    
    if (data[0] > 0) {
      Serial.write(data[0]);
      radio.write(data, sizeof(data));
      
      if(data[0] == 1){
        transmit = false;
      }
      
      data[0] = 0;
    }


  }
  
  else if (!transmit) {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();//start listening for data;

    int count = 0;

    while (radio.available()) {
      radio.read(&incoming, sizeof(incoming));
      
      if (incoming[0] > 0) {
        Serial.println(incoming[0]);
//        Serial.println(count);
        count++;
      }
    }

//    if(count > 5){
//      transmit = true;
//    }

    
  }
  delay(100);//wait so the data is readable
}

void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
  }
}

//
//void establishContact() {
//  while (Serial.available() <= 0) {
//  Serial.println("A");   // send a capital A
//  delay(300);
//  }
//}
