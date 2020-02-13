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
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>//include timer interrupt library
#include <math.h>

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

//variables
boolean transmit = false;              //set variable to send or receive data (use same code for both devices but change variable)
boolean uno = false;                   //set variable for type of microcontroller sending (uno-true-laptop,uno-false-robot)
RF24 radio(CE_PIN, CSN_PIN);          //create instance of radio object
#define team_channel 14              //set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
byte addresses[][6] = {"1Node", "2Node"};//unique address between transmitter and receiver
uint8_t data[1];                        //variable to hold transmit data
uint8_t incoming[1];                        //variable to hold receive data
uint8_t state[] = {0, 0};               //variable to hold receive data position
uint8_t mapDat[4][4];                   //variable to hold receive data MAP
uint8_t lastSend;                      // Store last send time

uint8_t outgoingIRData[1];


// NEW STUFF, erase once we actually have stuff to send to robot

char val; // Data received from the serial port
int ledPin = 13; // Set the pin to digital I/O 13
boolean ledState = LOW; //to toggle our LED




//define pin numbers
const int rtStepPin = 44; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 49;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 46; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin
const int stepTime = 300; //delay time between high and low on step pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick
#define redLED 11           //red LED for displaying states
#define grnLED 12         //green LED for displaying states
#define ylwLED 13        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor



void setup() {
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED


  stepperRight.setMaxSpeed(4000);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(4000);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver



  Serial.begin(baud_rate);//start serial communication
  
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Transmit.....");
    Serial.println("***********************************");
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Receive.....");
    Serial.println("***********************************");
  }
  

  // NEW STUFF to be rid of once we have stuff to send
  pinMode(ledPin, OUTPUT); // Set pin as OUTPUT
  
}

void loop() {

  if(transmit){
    delay(5);
    radio.stopListening();
    // read sensors
    double fDist = irRead(0);
    double bDist = irRead(1);
    double rDist = irRead(2);
    double lDist = irRead(3);
    Serial.println(fDist);
    outgoingIRData[0] = fDist;
    outgoingIRData[1] = bDist;
    outgoingIRData[2] = rDist;
    outgoingIRData[3] = lDist;
    
    radio.write(&outgoingIRData, sizeof(outgoingIRData)); // send sensor data
  }

  if (!transmit) {
//    Serial.println("I am recieving");
    while (radio.available()) {
      radio.read(&incoming, 1);
//      Serial.println("radio is available");
      if (incoming[0] > 0) {
//        Serial.println(incoming[0]);
//        Serial.println("NUMBER 1");

        ledState = !ledState; //flip the ledState
        digitalWrite(ledPin, ledState); 

        if(incoming[0] == 1) //if we get a 1
        {
          Serial.println("Got a 1 (S)");
        }
        if(incoming[0] == 2) //if we get a 1
        {
          Serial.println("Got a 2 (L)");
        }
        if(incoming[0] == 3) //if we get a 1
        {
          Serial.println("Got a 3 (R)");
        }
        if(incoming[0] == 4) //if we get a 1
        {
          Serial.println("Got a 4 (T)");
        }
        
        delay(100);
      }
    }//end while

    
  }
  delay(100);//wait so the data is readable
}




/* irRead is a helper function that reads the irSensor value at the given pin,
    converts the distance to inches, and returns the value.
*/
double irRead(int pin) {
  int value = 0;
  for (int i = 0; i < 30; i++) {
    value = value + analogRead(pin);
  }
  value = value / 30;

  if (pin == 0) {
    return 8452.3 * pow(value, -1.235); // front sensor
  } else if (pin == 1) {
    return 8916.1 * pow(value, -1.274); // back sensor
  } else if (pin == 3) {
    return 9992.4 * pow(value, -1.29); // left sensor
  } else if (pin == 2) {
    return 10910 * pow(value, -1.311); // right sensor
  } else {
    return 0;
  }

}


/*
  The forward function takes in a distance to drive in inches,
  then drives forward that distance.
*/
void forward(int distance) {
  int ticksPerInch = 76;
  int ticksToDrive = distance * ticksPerInch;

  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperRight.moveTo(ticksToDrive);//move one full rotation forward relative to current position
  stepperLeft.moveTo(ticksToDrive);//move one full rotation forward relative to current position
  stepperRight.setSpeed(400);//set right motor speed
  stepperLeft.setSpeed(400);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target

}


/*
  The goToAngle function takes in an angle in degrees (positive values are left, negative are right),
  then spins the robot to that angle.
*/
void goToAngle(int angle) {

  double ticksPerDegree = 5.62;
  int ticksToDrive = (int)(angle * ticksPerDegree);

  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero

  stepperRight.setMaxSpeed(500);//set right motor speed
  stepperLeft.setMaxSpeed(500);//set left motor speed
  stepperRight.moveTo(ticksToDrive);//set distance for right wheel to move
  stepperLeft.moveTo(-ticksToDrive);//set distance for left wheel to move
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target

}


/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}
