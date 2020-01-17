/*
***********************************
  Arkin-Lab03.ino
  Knut Peterson 12/13/19
  Garrett Jacobs 12/13/19

  This program will introduce subsumption architecture for the robot Arkin.
  The layers are randomWander, shyKid, aggressiveKid, smartWander, and goalHoming.
  randomWander - the robot moves around randomly.
  shyKid - the robot remains in place until an obstacle enters its line of sight, then it moves away.
  smartWander - the robot moves around randomly and avoids objects.
  goalHoming - the robot moves to specified coordinates while avoiding obstacles along the way.

*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>//include timer interrupt library
#include <math.h>

//define pin numbers
const int rtStepPin = 50; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 51;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 52; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin
const int stepTime = 300; //delay time between high and low on step pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define enableLED 13        //stepper enabled LED
#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define pauseTime 2500 //time before robot moves

double motorSpeed = -400;
enum State { center, right, left, obstacle, wander};
State state = wander;

double lError = 0;
double rError = 0;

double lPrevError = 0;
double rPrevError = 0;

double currentTime = 0;
double lastTime = 0;

double drEdt = 0;
double dlEdt = 0;

double lastMillis = 0;

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


  stepperRight.setMaxSpeed(2500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(2500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
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
  
  Serial.begin(9600);

}

void loop(){

  rightWallFollow();
//  Serial.println(irRead(3));// Back = 1, left = 3, right = 2, front = 0
}


void wallFollow(){
  updateState();
  switch(state)
  {
      case center: centerWallFollow();
          break;
      case right: rightWallFollow(); 
          break;
      case left : leftWallFollow();
          break;
      case obstacle : collide();
          break;
      case wander: randomWander();
          break;
  }

 

}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
*/
void updateState() {

  double rDist = irRead(2);
  double lDist = irRead(3);
  double bDist = irRead(1);
  
  if (rDist > 15 && lDist > 15 && bDist > 15 && millis()-lastMillis > 4000) { //no sensors triggered
     state = wander;
  }
  else if (  rDist < 15 && lDist > 15 ) { // only right sensor triggered
    lastMillis = millis();
    state = right;
  }
  else if ( rDist > 15 && lDist < 15 ) { // only left sensor triggered
    lastMillis = millis();
    state = left;
  }
  else if ( rDist > 15 && lDist > 15 && bDist < 4){ // only back sensor triggered
    state = obstacle;
  }
  else if ( rDist < 15 && lDist < 15 ) { // left and right sensor triggered
    state = center;
  }
  
}

/* Follow a wall on the left side of the robot using PD control
 */
void leftWallFollow(){

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  
  updateError(); // TAKE OUT FOR FULL PROGRAM

  double kp = 30.69; // nice
  double kd = 10.0;


  if(irRead(3) > 8){
    kp *= 5;
  }

  if(irRead(1) < 4){
    goToAngle(90);
  }

  double rightSpeed = (motorSpeed + kp*lError + kd*dlEdt);
  double leftSpeed = motorSpeed /*+ kp*rError*/;
  Serial.println(rightSpeed);
  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

  double beginMillis = millis();
  while(millis()<beginMillis + 50){
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor 
  }
}

/* Follow a wall on the right side of the robot using PD control
 */
void rightWallFollow(){

  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, HIGH);
  
  updateError(); // TAKE OUT FOR FULL PROGRAM

  double kp = 30.69; // nice
  double kd = 10.0;

  Serial.println(drEdt);
  if(irRead(2) > 8){
    kp *= 5;
  }

  if(irRead(1) < 4){
    Serial.println(irRead(1));
      goToAngle(-90);
  }

  double leftSpeed = (motorSpeed + kp*rError + kd*drEdt);
  double rightSpeed = motorSpeed /*+ kp*rError*/;

  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed

  double beginMillis = millis();
  while(millis()<beginMillis + 50){
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor 
  }
}


/* Keep the robot centered between two walls using P control
 */
void centerWallFollow(){

  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  
  updateError(); // TAKE OUT FOR FULL PROGRAM

  double leftSpeed = 0;
  double rightSpeed = 0;
  double deltaError = lError - rError;
  double deltadEdt = dlEdt - drEdt;

  double kp = 20.69; // nice
  double kd = 10.0;

  if(irRead(1) < 4){
      goToAngle(-90);
  }

  if(deltaError>0){
      leftSpeed = (motorSpeed - kp*deltaError - kd*deltadEdt);
      rightSpeed = motorSpeed /*+ kp*rError*/;
  }
  else{
  rightSpeed = (motorSpeed + kp*deltaError + kd*deltadEdt);
  leftSpeed = motorSpeed /*+ kp*rError*/;
  }

  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed
  stepperRight.setMaxSpeed(abs(rightSpeed));//set right motor speed
  stepperLeft.setMaxSpeed(abs(leftSpeed));//set left motor speed

  double beginMillis = millis();
  while(millis()<beginMillis + 50){
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor 
  }
  
}

/*
 * 
 */
void collide(){
    if(irRead(1) < 4){
    goToAngle(90);
  }
}

/*
 * 
 */
void updateError(){

  double rDist = irRead(2);
  double lDist = irRead(3);

  if(rDist>6){
    rError = 6-rDist;
  }
  else if(rDist<6){
    rError = 6-rDist;
  }

  if(lDist>6){
    lError = 6-lDist;
  }
  else if(lDist<6){
    lError = 6-lDist;
  }

  updateDerivatives();
  
}

/*
 * 
 */
double updateDerivatives(){
  currentTime = millis();

  drEdt = (rError-rPrevError)/(currentTime - lastTime);
  dlEdt = (lError-lPrevError)/(currentTime - lastTime);
  
  rPrevError = rError;
  lPrevError = lError;
  lastTime = currentTime;
}



/* irRead is a helper function that reads the irSensor value at the given pin,
 *  converts the distance to inches, and returns the value.
 */
double irRead(int pin){
  int value = 0;
  for (int i=0; i <30; i++){
    value = value + analogRead(pin);
  }
  value = value/30;

  if(pin == 0){
    return 8452.3*pow(value,-1.235); // front sensor
  } else if(pin == 1){
    return 8916.1*pow(value, -1.274); // back sensor
  } else if(pin == 3){
    return 9992.4*pow(value, -1.29); // left sensor
  } else if(pin == 2){
    return 10910*pow(value, -1.311); // right sensor
  } else{
    return 0;
  }

}


/* randomWander randomly decides to either drive or turn, then executes the
 *  manuever using a random value.
 */
void randomWander(){
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, LOW);

  if (random(0,10) > 5){
        // drive a random distance
        delay(500);
        forward(random(-6,6));
    }
    else {
      // turn to a random angle
      delay(500);
      pivot(random(-90, 90));
    }
}

/* reduceAngle is a helper function that takes in an angle in radians
 *  and if it is larger than a full circle, it divides by 2*pi and returns the value.
 */
double reduceAngle(double angle){
  if(angle > 2*3.14159265358){
    return angle/(2*3.14159265358);
  } else {
    return angle;
  }
}

void shyKid(){
  // if no objects within 15 inches, do nothing
  if(irRead(0) > 15 && irRead(1) > 15 && irRead(2) > 15 && irRead(3) > 15){
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, LOW);
  }else{
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, HIGH);

    // read the 4 ir sensors
    double fDist = irRead(0);
    double bDist = irRead(1);
    double lDist = irRead(3);
    double rDist = irRead(2);

    double leftSpeed = 0;
    double rightSpeed = 0;

    // if objects on both front and back, spin 90 degrees and move away
    if(fDist < 12 && bDist < 12 && lDist > 12 && rDist > 12){
      spin(90);
      forward(12);
    }
    
    // if no objects within 12 inches, do nothing
    else if(fDist < 12 && bDist < 12 && lDist < 12 && rDist < 12){
      double leftSpeed = 0;
      double rightSpeed = 0;
    }
    
    // if an object is behind, add the side force vectors
    else if(bDist<12){
      leftSpeed = 3000*(-1/fDist + 1/lDist + 1/bDist);
      rightSpeed = 3000*(-1/fDist + 1/rDist + 1/bDist);
    }
    
    // default to subtracting the side force vectos
    else{
      leftSpeed = 3000*(-1/fDist - 1/lDist + 1/bDist);
      rightSpeed = 3000*(-1/fDist - 1/rDist + 1/bDist);
    }
   
    stepperRight.setSpeed(rightSpeed);//set right motor speed
    stepperLeft.setSpeed(leftSpeed);//set left motor speed

    double beginMillis = millis();
    while(millis()<beginMillis + 50){
      stepperRight.runSpeed();//move right motor
      stepperLeft.runSpeed();//move left motor 
    }
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
  
//  digitalWrite(redLED, LOW);//turn off red LED
//  digitalWrite(grnLED, HIGH);//turn on green LED
//  digitalWrite(ylwLED, LOW);//turn off yellow LED

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
  The goToAngle function takes in an angle in degrees (positive values are left, negative are right),
  then spins the robot to that angle.
*/
void goToAngle(int angle) {
  
  double ticksPerDegree = 5.62;
  int ticksToDrive = (int)(angle*ticksPerDegree);

//  digitalWrite(grnLED, HIGH);//turn on green LED

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
