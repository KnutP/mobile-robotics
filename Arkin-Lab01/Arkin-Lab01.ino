/*
***********************************
  Arkin-Lab01.ino
  Knut Peterson 12/13/19
  Garrett Jacobs 12/13/19

  This program will introduce the basic movement functions of the robot Arkin.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary
  goToAngle - given an angle in degrees use odomery to turn the robot
  goToGoal - given an x and y position in feet, use the goToAngle() function and trigonometry to move to a goal positoin
  moveSquare - given the side length in feet, move the robot in a square with the given side length

*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <math.h>

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
#define enableLED 13        //stepper enabled LED
#define redLED 13           //red LED for displaying states
#define grnLED 12         //green LED for displaying states
#define ylwLED 11        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define pauseTime 2500 //time before robot moves

void setup()
{
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED


  stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  //Serial.begin(9600); //start serial communication at 9600 baud rate for debugging

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

}

void loop()
{
  goToAngle(90);
  delay(5000);

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

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*
  The pivot method takes in an angle in degrees (positive values are left, negative are right),
  then pivots the robot to that angle by running either the left or right motor forward.
*/
void pivot(int angle) {

  double ticksPerDegree = 11.32; // Measured the number of ticks in 90 degrees, then divided to find the constant
  int ticksToDrive = (int)(abs(angle)*ticksPerDegree);

  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero

  if(angle > 0){
    stepperRight.moveTo(ticksToDrive);//set distance to move
    stepperRight.setSpeed(1000);//set right motor speed
    stepperRight.runSpeedToPosition();//move right motor
    runToStop();//run until the robot reaches the target
  }else{    
    stepperLeft.moveTo(ticksToDrive);//set distance to move
    stepperLeft.setSpeed(1000);//set left motor speed
    stepperLeft.runSpeedToPosition();//move left motor
    runToStop();//run until the robot reaches the target
  }
  
}

/*
  The spin function takes in an angle in degrees (positive values are left, negative are right),
  then spins the robot to that angle by running one motor forward and the other backward.
*/
void spin(int angle) {

  double ticksPerDegree = 5.75;
  int ticksToDrive = (int)(angle*ticksPerDegree);

  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  
  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperRight.moveTo(ticksToDrive);//set distance for right wheel to move
  stepperLeft.moveTo(-ticksToDrive);//set distance for left wheel to move
  stepperRight.setSpeed(300);//set right motor speed
  stepperLeft.setSpeed(300);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  
}

/*
  The turn function takes in speeds and distances(pulses) to drive for each motor,
  then drives in a curve using the speed differential where the outer wheel drives at the faster speed
  and the inner wheel drives at the slower speed, resulting in a turn.
*/
void turn(double leftSpeed, double rightSpeed, int leftPulses, int rightPulses) {
    stepperLeft.setCurrentPosition(0);//set left wheel position to zero
    stepperRight.setCurrentPosition(0);//set right wheel position to zero
    long positions[2];
    positions[0] = leftPulses;
    positions[1] = rightPulses;
    stepperLeft.setSpeed(leftSpeed);
    stepperRight.setSpeed(rightSpeed);
    stepperLeft.setMaxSpeed(leftSpeed);
    stepperRight.setMaxSpeed(rightSpeed);
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

}
/*
  The forward function takes in a distance to drive in inches,
  then drives forward that distance.
*/
void forward(int distance) {
  int ticksPerInch = 76;
  int ticksToDrive = distance*ticksPerInch;
  
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperRight.moveTo(ticksToDrive);//move one full rotation forward relative to current position
  stepperLeft.moveTo(ticksToDrive);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  
}
/*
  The reverse function takes in a distance to drive in inches,
  then drives backward that distance.
*/
void reverse(int distance) {
  forward(-distance);
}
/*
  The stop function stops the robot's motors.
*/
void stop() {
  stepperRight.stop();//stop right motor
  stepperLeft.stop();//stop left motor
}

/*
  The moveCircle function takes in a diameter in inches and a direction integer (1 for left, -1 for right).
  The robot will then drive in a circle of the given diameter in the given direction.
*/
void moveCircle(int diam, int dir) {

  digitalWrite(redLED, HIGH); //turn on red LED
  digitalWrite(grnLED, LOW); //turn off green LED
  
  double robotWidth = 8.5; // inches
  double cWheel = 10.75; // inches
  double innerR = (diam/2) - (robotWidth/2);
  double outerR = (diam/2) + (robotWidth/2);

  double angleDifferential = 1.16; // Constant to account for slippage and friction
  
  double arcOuter = angleDifferential*2*3.14159265358*outerR; // solve for distance each wheel needs to travel
  double arcInner = angleDifferential*2*3.14159265358*innerR;
  double innerPulses = arcInner*(1/cWheel)*(800); //solve for pulses each motor needs to go
  double outerPulses = arcOuter*(1/cWheel)*(800); //800 pulses per rotation
  
  int outerSpeed = 2000; // pulses per second (pick a reference speed to base the other one off of)
  int driveTime = outerPulses/outerSpeed; // seconds
  int innerSpeed = innerPulses/driveTime; // pulses per second

  int tickDifferential = 280; // This values evened out the ticks traveled by the inner and outer wheels

  if(dir == 1){
    turn(outerSpeed, innerSpeed, outerPulses, innerPulses+tickDifferential);
  }else{
    turn(innerSpeed, outerSpeed, innerPulses+tickDifferential, outerPulses);
  }
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam, int dir){
  digitalWrite(redLED, HIGH); // turn on red LED
  digitalWrite(grnLED, LOW); // turn off green LED
  digitalWrite(ylwLED, HIGH); // turn on yellow LED

  moveCircle(diam, dir); // circle to the left
  delay(250);
  moveCircle(diam, -dir); // circle to the right
}

/*
  The goToAngle function takes in an angle in degrees (positive values are left, negative are right),
  then spins the robot to that angle.
*/
void goToAngle(int angle) {
  
  double ticksPerDegree = 5.62;
  int ticksToDrive = (int)(angle*ticksPerDegree);

  digitalWrite(grnLED, HIGH);//turn on green LED

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

/*
  The goToGoal function takes in coordinates x and y in inches,
  the drives to that location by first turning to the correct angle
  and then driving forward.
*/
void goToGoal(int x, int y) {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  
  double thetaD = atan2(y,x); // calculates angle from desired coordinates
  goToAngle(thetaD*180/3.14159265358); // turns to the angle
  
  int distance = sqrt((x*x)+(y*y)); // calculate distance from desired coordinates
  forward(distance); // drives distance
 
}

/*
  The moveSquare function takes in a side length in inches,
  then drives the robot in a square with sides of that length.
*/
void moveSquare(int side) {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED

  // iterates through all 4 sides of the square
  for(int i = 0; i<4; i++){
    forward(side);
    goToAngle(90);
  }
}
