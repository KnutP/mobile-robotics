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

enum RobotDirection {N, S, E, W};
RobotDirection robotDirection = N;



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

double motorSpeed = 200;
double lError = 0;
double rError = 0;

double lPrevError = 0;
double rPrevError = 0;

double currentTime = 0;
double lastTime = 0;

double drEdt = 0;
double dlEdt = 0;



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
    while (radio.available()) {
      radio.read(&incoming, 1);

      Serial.println(incoming[0]);
      
      if (incoming[0] > 0) {
        

        ledState = !ledState; //flip the ledState
        digitalWrite(ledPin, ledState); 

        //****** Topological ******\\

        if(incoming[0] == 1) //if we get a 1
        {
          Serial.println("Got a 1 (S)");
        }
        if(incoming[0] == 2) //if we get a 1
        {
          Serial.println("Got a 2 (L)");
          while (irRead(3)<15){
           rightWallFollow();
          }
            forward(5);
            goToAngle(90);
            forward(15);
            delay(300);
        }
        if(incoming[0] == 3) //if we get a 1
        {
          Serial.println("Got a 3 (R)");
          while (irRead(2)<15){
            leftWallFollow();
            
          }
            forward(5);
            goToAngle(-90);
            forward(15);
            delay(300);
        }
        if(incoming[0] == 4) //if we get a 1
        {
          Serial.println("Got a 4 (T)");
          while(irRead(0) > 6){
          rightWallFollow();
          }
          stop();
        }

        //****** Metric ******\\

        if(incoming[0] == 5) //if we get a 5
        {
          Serial.println("got a 5 (N)");
          turnToNorth();
          forward(15);
        }
        if(incoming[0] == 6) //if we get a 6
        {
          Serial.println("got a 6 (S)");
          turnToSouth();
          forward(15);
        }
        if(incoming[0] == 7) //if we get a 7
        {
          Serial.println("got a 7 (E)");
          turnToEast();
          forward(15);
        }
        if(incoming[0] == 8) //if we get an 8
        {
          Serial.println("got a 8 (W)");
          turnToWest();
          forward(15);
        }


        
        
        delay(100);
      }
    }//end while

    
  }
  delay(100);//wait so the data is readable
}

void turnToNorth(){

  if(robotDirection == N){
    // do nothing
  } else if (robotDirection == S){
    goToAngle(180);
  } else if (robotDirection == E){
    goToAngle(90);
  } else if (robotDirection == W){
    goToAngle(-90);
  }

  robotDirection = N;
  
}

void turnToSouth(){

  if(robotDirection == N){
    goToAngle(180);
  } else if (robotDirection == S){
    // do nothing
  } else if (robotDirection == E){
    goToAngle(-90);
  } else if (robotDirection == W){
    goToAngle(90);
  }

  robotDirection = S;
  
}

void turnToEast(){

  if(robotDirection == N){
    goToAngle(-90);
  } else if (robotDirection == S){
    goToAngle(90);
  } else if (robotDirection == E){
    // do nothing
  } else if (robotDirection == W){
    goToAngle(-180);
  }

  robotDirection = E;
  
}

void turnToWest(){

  if(robotDirection == N){
    goToAngle(90);
  } else if (robotDirection == S){
    goToAngle(-90);
  } else if (robotDirection == E){
    goToAngle(180);
  } else if (robotDirection == W){
    // do nothing
  }

  robotDirection = W;
  
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
  stepperRight.setMaxSpeed(400);//set right motor speed
  stepperLeft.setMaxSpeed(400);//set left motor speed
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

/*
    aggressiveKid has the robot move until he is within
    a certain range of an object, the stop
*/
void aggressiveKid() {
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);

  // if the front ir sensor sees something, stop
  while (irRead(0) > 6) {

    double leftSpeed = motorSpeed;
    double rightSpeed = motorSpeed;

    stepperRight.setSpeed(rightSpeed);//set right motor speed
    stepperLeft.setSpeed(leftSpeed);//set left motor speed

    double beginMillis = millis();
    while (millis() < beginMillis + 50) {
      stepperRight.runSpeed();//move right motor
      stepperLeft.runSpeed();//move left motor
    }
  }
  stop();
}

void stop() {
  stepperRight.stop();//stop right motor
  stepperLeft.stop();//stop left motor
}

/* 
 *  Follow a wall on the right side of the robot using PD control
 */
void rightWallFollow(){

  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, HIGH);
  
  updateError();

  double kp = 30;
  double kd = 10.0;

  // if we lose the wall, turn toward where it was
  // adjust speed based on distance between robot and wall
  double rightSpeed = (motorSpeed + kp*rError + kd*drEdt);
  double leftSpeed = motorSpeed;
  
  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

   double beginMillis = millis();
    while (millis() < beginMillis + 50) {
      stepperRight.runSpeed();//move right motor
      stepperLeft.runSpeed();//move left motor
    }

}

void leftWallFollow(){

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  
  updateError();

  double kp = 20;
  double kd = 10.0;

  // adjust speed based on distance between robot and wall
  double leftSpeed = (motorSpeed + kp*lError + kd*dlEdt);
  double rightSpeed = motorSpeed;

  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

  double beginMillis = millis();
  while(millis()<beginMillis + 50){
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor 
  }
}


void updateError(){

  double rDist = irRead(2);
  double lDist = irRead(3);

  if(rDist>5){
    rError = 5-rDist;
  }
  else if(rDist<5){
    rError = 5-rDist;
  }

  if(lDist>5){
    lError = 5-lDist;
  }
  else if(lDist<5){
    lError = 5-lDist;
  }

  updateDerivatives();
  
}

/* updateDerivatives is a helper function that calculates
 *  the change in error over the change in time
 *  for the left and right sides of the robot since the
 *  last time the function was called.
 */
double updateDerivatives(){
  currentTime = millis();

  drEdt = (rError-rPrevError)/(currentTime - lastTime);
  dlEdt = (lError-lPrevError)/(currentTime - lastTime);
  
  rPrevError = rError;
  lPrevError = lError;
  lastTime = currentTime;
}
