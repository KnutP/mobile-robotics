/*
***********************************
  Arkin_Lab02.ino
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

void loop()
{

//  smartWander();
  goalHoming(72, 0);
  delay(10000);
}

/* randomWander drives the robot around randomly
 */
void randomWander(){
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, HIGH);

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

/* smartWander randomly moves around while avoiding obstacles
 */
void smartWander(){
  
  // while the robot sees something, run the shyKid behavior to avoid the obstacle
  while(irRead(0) < 15 || irRead(1) < 15 || irRead(2) < 15 || irRead(3) < 15){
    shyKid()
  }

  // if the robot does not see anything, then randomly wander
  if(irRead(0) > 15 && irRead(1) > 15 && irRead(2) > 15 && irRead(3) > 15){
    randomWander();
  }
}

/* The goalHoming method moves the robot toward a goal while avoiding obstacles.
 */
void goalHoming(double goalX, double goalY){

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  
  double currentX = 0;
  double currentY = 0;
  double currentAngle = 0;
  
  double distanceError = 0.0931; // to account for overshoot from moving forward in 1 inch increments

  boolean atGoal = false;

  // while not at goal, run the goal homing
  while( !atGoal ){
    // check if we're at the goal within a range of +- 1 inch
    atGoal = ( (goalX+1) > currentX && currentX > (goalX-1) ) && ( (goalY+1) > currentY && currentY > (goalY-1) );

    // calculate angle to turn toward goal
    // deltaX and deltaY are the x and y distances between the robot and the goal
    double deltaX = goalX-currentX;
    double deltaY = goalY-currentY;
    double goalAngle = atan2(deltaY, deltaX); // calculates angle from desired coordinates
    
    // delta angle is the angle we need to turn so we face the goal based on where we are currently pointing
    double deltaAngle = goalAngle - currentAngle;
    goToAngle(deltaAngle*180/3.14159265358);
    currentAngle += deltaAngle;

    // if the current angle is >360 degrees, reduces the angle to below 360 degrees
    currentAngle = reduceAngle(currentAngle);

    // while no obstacle, drive toward goal
    // after 10 iterations (count), break out of the the loop so the robot readjusts to turn to the goal
    int count = 0;
    while(irRead(0) > 5 && !atGoal && count < 10 ){
      deltaX = goalX-currentX;
      deltaY = goalY-currentY;
    
      forward(1); // drives forward 1 inch
      currentX = currentX + cos(currentAngle) - distanceError;
      currentY = currentY + sin(currentAngle) - distanceError;

      // check if we're at the goal yet
      atGoal = ( (goalX+1) > currentX && currentX > (goalX-1) ) && ( (goalY+1) > currentY && currentY > (goalY-1) );

      count++;

    }


    // if we see an object, turn left 90 degrees, then drive until we can't see it anymore
    if(irRead(0) < 6){
      goToAngle(85);
      currentAngle += 3.14159265358/2;
      currentAngle = reduceAngle(currentAngle);
      
      while((irRead(2) < 15 || irRead(3) < 15) && !atGoal){
        forward(1);
        currentX = currentX + cos(currentAngle) - distanceError;
        currentY = currentY + sin(currentAngle) - distanceError;
  
        atGoal = ( (goalX+1) > currentX && currentX > (goalX-1) ) && ( (goalY+1) > currentY && currentY > (goalY-1) );

      }
      // once we no longer see the object, move forward to clear the robot body from contacting the object,
      // turn back, and drive forward to let the sensor see the object again
      if( !atGoal ){
        forward(4);
        currentX = currentX + 4*cos(currentAngle) - distanceError;
        currentY = currentY + 4*sin(currentAngle) - distanceError;
        goToAngle(-95);
        currentAngle += -3.14159265358/2;
        currentAngle = reduceAngle(currentAngle);
        forward(12);
        currentX = currentX + 12*cos(currentAngle) - distanceError;
        currentY = currentY + 12*sin(currentAngle) - distanceError;
      }

      // drive forward until we don't see the object anymore
      while((irRead(2) < 15 || irRead(3) < 15) && !atGoal){
        forward(1);
        currentX = currentX + cos(currentAngle) - distanceError;
        currentY = currentY + sin(currentAngle) - distanceError;

        // check if we're within 1 inch of the goal
        atGoal = ( (goalX+1) > currentX && currentX > (goalX-1) ) && ( (goalY+1) > currentY && currentY > (goalY-1) );
      }
    }

    
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

/* irRead is a helper function that reads the irSensor value at the given pin,
 *  converts the distance to inches, and returns the value.
 */
double irRead(int pin){
  int value = 0;
  for (int i=0; i <30; i++){
    value = value + analogRead(pin);
  }
  value = value/30;
  double value_front = 8452.3*pow(value,-1.235);
  double value_left = 9992.4*pow(value, -1.29);
  double value_right = 10910*pow(value, -1.311);
  double value_back = 8916.1*pow(value, -1.274);

  if(pin == 0){
    return value_front;
  } else if(pin == 1){
    return value_back;
  } else if(pin == 3){
    return value_left;
  } else if(pin == 2){
    return value_right;
  } else{
    return 0;
  }

}

void aggressiveKid(){
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);

  // if the front ir sensor sees something, stop
  if(irRead(0)<4){
    
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, LOW);
    stop();
  }
  // otherwise, drive forward
  else{
    stepperRight.run();//move one full rotation forward relative to current position
    stepperLeft.run();//move one full rotation forward relative to current position
    stepperRight.setSpeed(1000);//set right motor speed
    stepperLeft.setSpeed(1000);//set left motor speed
    stepperRight.runSpeed();//move right motor
    stepperLeft.runSpeed();//move left motor
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
    
    // if an object is in front, subtract the side force vectors
    else if(fDist < 12){
      leftSpeed = 3000*(-1/fDist - 1/lDist + 1/bDist);
      rightSpeed = 3000*(-1/fDist - 1/rDist + 1/bDist);
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
