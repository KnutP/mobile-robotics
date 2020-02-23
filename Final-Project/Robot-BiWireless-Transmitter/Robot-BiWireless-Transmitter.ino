/*Robot-BiWireless-Transmitter.ino
  Authors: Knut Peterson, Garrett Jacobs
  Modified: 02/22/2020
  This program handles the transmission of commands sent to the robot via the GUI in Processing.
  There are two types of behaviors:
  Transmission: When in transmission mode, the program will read incoming commands from the Serial
  in Processing, then send them on to the robot.
  Reception: When in reception mode, the program listens for data from the robot, then sends it
  to Processing over Serial.
  Switching to transmission mode, drive forward, turn to a compass angle, or execute a metric path.
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
int count = 0;

boolean newData = false;

String slrt = "";

void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
  }

}


void loop() {
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe

    // read in data from Processing
    readSerial();

    // send the data
    if (data[0] > 0) {
      radio.write(data, sizeof(data));

      // if we get a 1, stop transmitting start listening
      // for data from the robot
      if(data[0] == 1){
        transmit = false;
      }
      
      data[0] = 0;
    }
  }
  
  else if (!transmit) {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();//start listening for data;

    while (radio.available()) {
      // read in data from the robot
      radio.read(&incoming, sizeof(incoming));
      
      if (incoming[0] > 0) {
        // send the data to Processing
        Serial.println(incoming[0]);
        count++;
      }
      if(count == 4){
        // once we get data from all 4 sensors, resume sending data from processing
        radio.stopListening();//start listening for data;
        count = 0;
        transmit = true;
      }
    }
    
  }
  
  delay(100);//wait so the data is readable
}


/*  The readSerial method parses in data sent from Processing
 *  through the Serial monitor and stores it.
 */
void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
  }
}
