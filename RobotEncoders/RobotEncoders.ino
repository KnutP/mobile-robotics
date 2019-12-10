/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  /*RobotEncoders.ino
   C.A. Berry, 11/20/17
   This program will demonstrate how the retrofit encoders for the CEENBot with the Arduino Mega. The encoders will use an interrupt
   to count the steps moved in order to provide feedback control to correct for odometry error. An interrupt will be used to read the encoder ticks and
   printed to the serial monitor. Try to determine how many ticks for one rotatoin of the wheel.

  encoder links:
    https://www.dfrobot.com/wiki/index.php/Wheel_Encoders_for_DFRobot_3PA_and_4WD_Rovers_(SKU:SEN0038)
    http://image.dfrobot.com/image/data/SEN0038/encoderSketch.zip
  mega links:
    pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560

  Hardware Connection:
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - left stepper motor step pin
  digital pin 51 - left stepper motor direction pin
  digital pin 52 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 2 - left wheel encoder
  digital pin 3 - right wheel encoder

  Key Variables:
  encoder[2]        //variable to hold number of encoder counts (left, right)
  lastSpeed[2]      //variable to hold encoder speed (left, right)

  Functions the program uses:
  move1() - function to move the robot wheels to count encoder ticks
  LWheelSpeed() - interrupt functions to count left encoder ticks, use difference between previous and current to calculate speed
  RWheelSpeed() - interrupt functions to count right encoder ticks, use difference between previous and currenct to calculate speed

  The following functions will be created by using the encoder feedback:
  goToAngle - given an angle in degrees use odomery to turn the robot
  goToGoal - given an x and y position in feet, use the goToAngle() function and trigonometry to move to a goal positoin
  moveSquare - given the side length in feet, move the robot in a square with the given side length
*/

#include <AccelStepper.h>       //include the stepper motor library
#include <MultiStepper.h>       //include multiple stepper motor library
#include <math.h>

//define pin numbers and variables
const int rtStepPin = 50;       //right stepper motor step pin
const int rtDirPin = 51;        // right stepper motor direction pin
const int ltStepPin = 52;       //left stepper motor step pin
const int ltDirPin = 53;        //left stepper motor direction pin
const int stepTime = 500;       //delay time between high and low on step pin
const int ltEncoder = 2;        //left encoder pin
const int rtEncoder = 3;        //right encoder pin
const int stepperEnable = 48;   //stepper enable pin on stepStick
const int enableLED = 13;       //stepper enabled LED

//define global program variables
#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states
#define stepperEnTrue false       //const for enabling stepper motor
#define stepperEnFalse true       //const for disabling stepper motor
#define one_rot 800               //number of counts for one wheel rotation
#define two_rot 1600              //number of counts for 2 wheel rotations
#define half_rot 400              //number of counts for 1/2 a wheel rotation
#define qua_rot 200               //number of counts for 1/4 a wheel rotation
#define LEFT  0                   //constant for left wheel
#define RIGHT 1                   //constant for right wheel
#define FWD 0                     //move constant for forward
#define REV 1                     //move constant for reverse
#define baud 9600                 //serial communication baud rate
#define wait_time 1000              //time to wait between prints and starting robot
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time


//   the setup function runs only once to set up all variables, inputs, outpus, serial communication and interrupts
void setup() {
  pinMode(rtStepPin, OUTPUT);         //sets right stepper pin as output
  pinMode(rtDirPin, OUTPUT);          //sets right stepper direction pin as output
  pinMode(ltStepPin, OUTPUT);         //sets left stepper pin as output
  pinMode(ltDirPin, OUTPUT);          //sets left stepper director pin as output
  pinMode(stepperEnable, OUTPUT);     //sets stepper enable pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);         //sets pin 13 enable LED on microcontroller as output
  digitalWrite(enableLED, LOW);       //turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(500); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED
  
  stepperRight.setMaxSpeed(1500);     //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);      //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000); //set desired acceleration in steps/s^2
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);      //turn on enable LED
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
  Serial.begin(baud);                 //init the Serial port to print the data
  delay(wait_time);                   //wait before moving the robot


  delay(3000);//wait 5 seconds
  // Light up red, yellow, green, then start program
  delay(500);
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);
  delay(500);
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, HIGH);
  delay(500);
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, LOW);
  delay(500);

   goToAngle(90);
//  goToGoal(12,12);
//  moveSquare(24);

}

//the loop funciton runs continuously to move the robot wheels and count encoder ticks
void loop() {
  
  //move1(FWD, two_rot);            //move the robot wheels
  //print_data();                   //prints encoder data
  //delay(wait_time);               //wait to move robot
}

//function prints encoder data to serial monitor
void print_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}


/*
   The move1(int dir, int amt) function will move the robot forward or reverse by amount in input.Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1(int dir, int amt) {
  digitalWrite(ltDirPin, dir);
  digitalWrite(rtDirPin, dir);
  for (int x = 0; x < amt; x++) {  // Makes pulses for wheel rotations
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
}

/*
  The goToAngle function takes in an angle in degrees (positive values are left, negative are right),
  then spins the robot to that angle.
*/
void goToAngle(int angle) {
  
  double ticksPerDegree = 5.75;
  int ticksToDrive = (int)(angle*ticksPerDegree);

  //digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  //digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperRight.setSpeed(300);//set right motor speed
  stepperLeft.setSpeed(300);//set left motor speed
  stepperRight.moveTo(ticksToDrive);//set distance for right wheel to move
  stepperLeft.moveTo(-ticksToDrive);//set distance for left wheel to move
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  
}

/*
  The goToGoal function takes in coordinates x and y in inches,
  the drives to that location by first turning to the correct angle
  and then driving forward.
*/
void goToGoal(int x, int y) {
  digitalWrite(redLED, LOW);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED
  double thetaD = atan2(y,x);
  goToAngle(thetaD*180/3.14159265358);
  int distance = sqrt((x*x)+(y*y));
  forward(distance);
 
}

/*
  The moveSquare function takes in a side length in inches,
  then drives the robot in a square with sides of that length.
*/
void moveSquare(int side) {
  digitalWrite(redLED, HIGH);//turn on red LED
 // digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED
  for(int i = 0; i<4; i++){
    forward(side);
    goToAngle(90);
  }
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
  The forward function takes in a distance to drive in inches,
  then drives forward that distance.
*/
void forward(int distance) {
  int ticksPerInch = 76;
  int ticksToDrive = distance*ticksPerInch;
  stepperRight.move(ticksToDrive);//move one full rotation forward relative to current position
  stepperLeft.move(ticksToDrive);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  
}
