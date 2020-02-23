/*Robot-BiWireless-Robot.ino
  Authors: Knut Peterson, Garrett Jacobs
  Modified: 02/10/17
  This program handles the execution of commands sent to the robot via the GUI in Processing.
  There are two types of behaviors:
  Transmission: When in transmission mode, the program will read the 4 IR sensors and send the
  data to the GUI.
  Reception: When in reception mode, the robot receives commands from the GUI and executes them.
  The commands it can parse are:
  Switching to transmission mode, drive forward, turn to a compass angle, or execute a metric path.

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
uint8_t metricIncoming[9] = {99, 99, 99, 99, 99, 99, 99, 99, 99};
int metricIncomingIndex = 0;

uint8_t outgoingIRData[1];

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

// variables for PD control
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

}

void loop() {

  if (transmit) {
    // change to transmitting data
    radio.openWritingPipe(pipe);//open up writing pipe
    delay(50);
    radio.stopListening();
    delay(150);
    
    // read sensors
    double fDist = irRead(0);
    double bDist = irRead(1);
    double rDist = irRead(2);
    double lDist = irRead(3);
    delay(150);

    // read and send front sensor data
    if (fDist > 12) {
      outgoingIRData[0] = 5;
    } else {
      outgoingIRData[0] = 9;
    }
    delay(150);
    radio.write(&outgoingIRData, sizeof(outgoingIRData)); // send sensor data
    Serial.println(outgoingIRData[0]);

    // read and send back sensor data
    if (bDist > 12) {
      outgoingIRData[0] = 6;
    } else {
      outgoingIRData[0] = 9;
    }
    delay(150);
    radio.write(&outgoingIRData, sizeof(outgoingIRData)); // send sensor data
    Serial.println(outgoingIRData[0]);

    // read and send right sensor data
    if (rDist > 12) {
      outgoingIRData[0] = 7;
    } else {
      outgoingIRData[0] = 9;
    }
    delay(150);
    radio.write(&outgoingIRData, sizeof(outgoingIRData)); // send sensor data
    Serial.println(outgoingIRData[0]);

    // read and send left sensor data
    if (lDist > 12) {
      outgoingIRData[0] = 8;
    } else {
      outgoingIRData[0] = 9;
    }
    delay(150);
    radio.write(&outgoingIRData, sizeof(outgoingIRData)); // send sensor data
    Serial.println(outgoingIRData[0]);

    // switch back to receiving data
    transmit = false;
  }

  if (!transmit) {

    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();//start listening for data;
    
    while (radio.available()) {
      
      // read incoming commands from the transmitter
      radio.read(&incoming, 1);

      // if we haven't received a command to execute,
      // keep track of commands that have been sent
      if (metricIncomingIndex != 9) {
        metricIncoming[metricIncomingIndex] = incoming[0];
        metricIncomingIndex += 1;
      }
      if (incoming[0] == 9) {
        driveMetricPath();
        metricIncomingIndex = 0;
      }


      if (incoming[0] > 0) {

        //****** Topological ******\\

        // if we get a 1, transmit sensor data on next iteration
        if (incoming[0] == 1){
          Serial.println("Got a 1");
          transmit = true;
        }

        // if we get a 2, follow wall until we can turn left, then do so
        if (incoming[0] == 2){
          
          while (irRead(3) < 15) {
            rightWallFollow();
          }
          forward(5);
          goToAngle(90);
          forward(15);
          delay(300);
          
        }

        // if we get a 3, follow wall until we can turn right, then do so
        if (incoming[0] == 3)
        {
          while (irRead(2) < 15) {
            leftWallFollow();

          }
          forward(5);
          goToAngle(-90);
          forward(15);
          delay(300);
          
        }

        // if we get a 4, drive forward one square
        if (incoming[0] == 4){
//          while (irRead(0) > 6) {
//            rightWallFollow();
//          }
//          stop();
            forward(17);
        }
        
        delay(100);
        
      }
    }//end while
  }
  delay(100);//wait so the data is readable
}


/* The driveMetricPath method executes a list of commands sent by
 *  the GUI to follow a planned metric path.
 */
void driveMetricPath() {
  char current;

  for (int i = 0; i < 9; i++) {
    current = metricIncoming[i];

    // if we get a 5, drive north
    if (current == 5){
      turnToNorth();
      forward(17);
    }

    // if we get a 6, drive south
    if (current == 6){
      turnToSouth();
      forward(17);
    }

    // if we get a 7, drive east
    if (current == 7){
      turnToEast();
      forward(17);
    }

    // if we get an 8, drive west
    if (current == 8){
      turnToWest();
      forward(17);
    }

    // reset commands once they are executed
    metricIncoming[i] = 99;
  }
}


/* The turnToNorth method decides how far to turn to reach North
 *  based on the robot's current orientation, then executes the turn
 */
void turnToNorth() {

  if (robotDirection == N) {
    // do nothing
  } else if (robotDirection == S) {
    goToAngle(180);
  } else if (robotDirection == E) {
    goToAngle(90);
  } else if (robotDirection == W) {
    goToAngle(-90);
  }

  robotDirection = N;
}


/* The turnToSouth method decides how far to turn to reach South
 *  based on the robot's current orientation, then executes the turn
 */
void turnToSouth() {

  if (robotDirection == N) {
    goToAngle(180);
  } else if (robotDirection == S) {
    // do nothing
  } else if (robotDirection == E) {
    goToAngle(-90);
  } else if (robotDirection == W) {
    goToAngle(90);
  }

  robotDirection = S;
}


/* The turnToEast method decides how far to turn to reach East
 *  based on the robot's current orientation, then executes the turn
 */
void turnToEast() {

  if (robotDirection == N) {
    goToAngle(-90);
  } else if (robotDirection == S) {
    goToAngle(90);
  } else if (robotDirection == E) {
    // do nothing
  } else if (robotDirection == W) {
    goToAngle(-180);
  }

  robotDirection = E;
}


/* The turnToWest method decides how far to turn to reach West
 *  based on the robot's current orientation, then executes the turn
 */
void turnToWest() {

  if (robotDirection == N) {
    goToAngle(90);
  } else if (robotDirection == S) {
    goToAngle(-90);
  } else if (robotDirection == E) {
    goToAngle(180);
  } else if (robotDirection == W) {
    // do nothing
  }

  robotDirection = W;
}


/* irRead is a helper function that reads the irSensor value at the given pin,
 * converts the distance to inches, and returns the value.
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


/* This function, runToStop(), will run the robot until the target is achieved and
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
    Follow a wall on the right side of the robot using PD control
*/
void rightWallFollow() {

  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, HIGH);

  updateError();

  double kp = 30;
  double kd = 10.0;

  // if we lose the wall, turn toward where it was
  // adjust speed based on distance between robot and wall
  double rightSpeed = (motorSpeed + kp * rError + kd * drEdt);
  double leftSpeed = motorSpeed;

  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

  double beginMillis = millis();
  while (millis() < beginMillis + 50) {
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor
  }

}


/*
    Follow a wall on the left side of the robot using PD control
*/
void leftWallFollow() {

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);

  updateError();

  double kp = 20;
  double kd = 10.0;

  // adjust speed based on distance between robot and wall
  double leftSpeed = (motorSpeed + kp * lError + kd * dlEdt);
  double rightSpeed = motorSpeed;

  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

  double beginMillis = millis();
  while (millis() < beginMillis + 50) {
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor
  }
}

/* Update errors for the wall following behaviors using the IR sensors
 */
void updateError() {

  double rDist = irRead(2);
  double lDist = irRead(3);

  if (rDist > 5) {
    rError = 5 - rDist;
  }
  else if (rDist < 5) {
    rError = 5 - rDist;
  }

  if (lDist > 5) {
    lError = 5 - lDist;
  }
  else if (lDist < 5) {
    lError = 5 - lDist;
  }

  updateDerivatives();

}

/* updateDerivatives is a helper function that calculates
    the change in error over the change in time
    for the left and right sides of the robot since the
    last time the function was called.
*/
double updateDerivatives() {
  currentTime = millis();

  drEdt = (rError - rPrevError) / (currentTime - lastTime);
  dlEdt = (lError - lPrevError) / (currentTime - lastTime);

  rPrevError = rError;
  lPrevError = lError;
  lastTime = currentTime;
}
