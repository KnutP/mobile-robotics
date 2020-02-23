/* Final_Project_GUI
*  Authors: Knut Peterson, Garrett Jacobs
*  Modified: 2/22/2020
*  This program provides the GUI and decision making for Topological Path Following,
*  Metric Path Planning, Localization, and Mapping.
*/

import processing.serial.*;
Serial myPort;

import controlP5.*;
ControlP5 cp5;

// Occupancy Grid
//int gridLayout[][] =
//    {
//      {11, 15, 13,  3},
//      {10, 15, 15, 10},
//      { 8,  5,  5,  2},
//      {14, 15, 15, 14}
//    };

// Topological map
int gridLayout[][] =
    {
      { 9,  1,  7, 11},
      {14,  8,  5,  2},
      { 9,  6,  9,  6},
      {14, 13,  6, 15}
    };

// Localization Map
//int gridLayout[][] =
//    {
//      { 9,  5,  1,  7},
//      {10, 15, 10, 15},
//      {10, 15, 10, 15},
//      {14, 15, 14, 15}
//    };

// Default Map for Mapmaking
//int gridLayout[][] =
//    {
//      {15, 15, 15, 15},
//      {15, 15, 15, 15},
//      {15, 15, 15, 15},
//      {15, 15, 15, 15}
//    };

// constants for drawing the map
int gridXOffset = 250;
int gridYOffset = 100;
int boxWidth = 50;

// positions for drawing the robot and
int robotX = -1;
int robotY  = -1;
int robot2X = -10;
int robot2Y  = -10;

// global variables referenced throughout the program
int goalX = -1;
int goalY = -1;
int lastDirection = 1;
String input;
String lastVal = "";
int manhattanNumber=0;
int[][] wavefrontGrid = new int[4][4];
String val;
PShape[][] map = new PShape[4][4]; 
PFont font;
String[] irData = {"9", "9", "9", "9"};
int iterationCount = 0;

// array initialization for localization
int possibleCoords1[][] =
    {
      { -1,  -1,  -1, -1},
      {-1,  -1,  -1,  -1},
    };
int possibleCoords2[][] =
    {
      { -1,  -1,  -1, -1},
      {-1,  -1,  -1,  -1},
    };


void setup() {
 
  size(700, 800); //make our canvas 700 x 800 pixels big
  
  font = loadFont("Bahnschrift-32.vlw");
  textFont(font, 32);
  
  
  cp5 = new ControlP5(this);
  
  // Text input for topological commands
  cp5.addTextfield("textInput").setPosition(400, 450).setSize(200, 40).setAutoClear(false);
  cp5.addBang("topologicalSubmit").setPosition(400, 500).setSize(80, 40); 
  
  // Text input for goal position
  cp5.addTextfield("textInputGoal").setPosition(100, 450).setSize(200, 40).setAutoClear(false);
  cp5.addBang("SubmitGoal").setPosition(100, 500).setSize(80, 40); 
  
  // Text input for robot position
  cp5.addTextfield("textInputPosition").setPosition(100, 550).setSize(200, 40).setAutoClear(false);
  cp5.addBang("SubmitPosition").setPosition(100, 600).setSize(80, 40); 
  
  // Text input for starting localization
  cp5.addBang("StartLocalization").setPosition(515, 625).setSize(80, 40);
  
  // Text input for mapmaking
  cp5.addBang("StartMapmaking").setPosition(515, 675).setSize(80, 40); 
  
  
  //  initialize serial port and set the baud rate to 9600
  //myPort = new Serial(this, "COM7", 9600);
  //myPort.bufferUntil('\n'); 
  
  
  // make the grid for the map
  for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++) {
     
     stroke(0);
     map[i][j] = createShape( RECT, gridXOffset+(i*50),gridYOffset+(j*50),50,50);
   }
 } 
}


/* The draw() method is the main loop for the program. It updates the GUI display,
*  registers mouse clicks, and issues commands to the robot.
*/
void draw() {
  
  background(255); // clear the screen
  
  displayMap(map);
  drawRobot();
  drawStartRobot();
  
  // draw text
  fill(0);
  textFont(font, 32);
  text("Metric", 100, 430);
  text("Topological", 400, 430);
  
  textFont(font, 20);
  text("Goal:", 25, 475);
  text("Robot:", 25, 575);
  text("Localize:", 400, 650);
  text("Make Map:", 400, 700);
    
    
  // update topological map based on clicks
  if (mousePressed == true && getMouseLocation()[1] != -1){
    
    int location [] = {0, 0};
    location = getMouseLocation();
    int i = location[0];
    int j = location[1];
       
     if(gridLayout[i][j] >= 15){
       gridLayout[i][j] = 0;
     } else{
       gridLayout[i][j] += 1;
     }
     
     println(i);
     println(j);
     println(gridLayout[i][j]);
     
     mousePressed = false;
    
  }
  
  // update map based on clicks and and display
  for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++){
     drawLineFromClick(gridLayout[i][j], i, j);
   }
 }
  
  // display wavefront grid numbers
 for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++){
     
     stroke(0);
     text(wavefrontGrid[i][j], gridXOffset+10+(i*50),gridYOffset+40+(j*50));
   }
 } 
 
 // if we have a robot position and a goal, then go to the goal
 followPath();
  
}

/********* Map Making *********/

/* StartMapmaking() is called whenever the mapmaking button is pressed.
*  The function gathers sensor data from the robot and updates the map on the GUI
*  once it completes. The robot moves from square to square and maps the entirety
*  of the world. Note: There is a bug where the robot stops communicating and
*  freezes after the first three iterations of making the map. We still have no
*  idea where the bug is or why it occurs.
*/
void StartMapmaking(){
  robotX = 3;
  robotY = 3;
  
  while(robotX != 0 || robotY != 3){
    
    // get the sensor readings from the robot and save the obstacle number
    gridLayout[robotY][robotX] = getObstacleNumFromRobot();
    
    int options[]={99,99,99,99};
    int obstacleNum = gridLayout[robotY][robotX];
    
    // update list of possible locations to drive to next
    if(canMoveNorth(obstacleNum)){
      println("can move north 1");
      if(robotY-1 >0){
        println("stuff ahead");
        if(gridLayout[robotY-1][robotX] == 15){
          options[0] = 1;
          println("can move north");
        } 
      } 
    }
    if(canMoveSouth(obstacleNum)){
      if(robotY+1 < 4){
        if(gridLayout[robotY+1][robotX] == 15){
          options[1] = 1;
          println("can move south");
        } 
      } 
    }
    if(canMoveEast(obstacleNum)){
       if(robotX+1 < 4){
        if(gridLayout[robotY][robotX+1] == 15){
          options[2] = 1;
          println("can move east");
        } 
      }
    }
    if(canMoveWest(obstacleNum)){
       if(robotX-1 > 0){
        if(gridLayout[robotY][robotX-1] == 15){
          options[3] = 1;
          println("can move west");
        } 
      }
    }
    

    // if we can move east and we haven't been to the square already, go there
    if(options[2]!=99){
      turnToEast();

      driveForward();
      println("moved east");
      robotX += 1;
      
      delay(1000);
      turnToNorth();
      
    } 
    // if we can move west and we haven't been to the square already, go there
    else if(options[3]!=99){
      turnToWest();
      
      driveForward();
      println("moved west");
      robotX += -1;
      
      delay(1000);
      turnToNorth();
    } 
    // if we can move north and we haven't been to the square already, go there
    else if(options[0]!=99){
      turnToNorth();
      delay(1000);
      
      driveForward();
      println("moved north");
      robotY += -1;
    }
    // if we can move south and we haven't been to the square already, go there
    else if(options[1]!=99){
      turnToSouth();
      
      driveForward();
      println("moved south");
      robotY += 1;
      
      delay(1000);
      turnToNorth();
    }
    else if(canMoveEast(obstacleNum)){
      turnToEast();
      
      driveForward();
      println("moved east");
      robotX += -1;
      
      delay(1000);
      turnToNorth();
    } 
    else if(canMoveWest(obstacleNum)){
      turnToWest();
      
      driveForward();
      println("moved west");
      robotX += -1;
      
      delay(1000);
      turnToNorth();
    } 
  
  }
 
}


/* The following 5 methods send a command to the robot to do
*  what the method is named (drive forward, turn to North, etc.)
*/
void driveForward(){
  myPort.write('4');
  myPort.write('\n');
}

void turnToNorth(){
  myPort.write('5');
  myPort.write('\n');
  myPort.write('9');
  myPort.write('\n');
}

void turnToEast(){
  myPort.write('7');
  myPort.write('\n');
  myPort.write('9');
  myPort.write('\n');
}

void turnToWest(){
  myPort.write('8');
  myPort.write('\n');
  myPort.write('9');
  myPort.write('\n');
}

void turnToSouth(){
  myPort.write('6');
  myPort.write('\n');
  myPort.write('9');
  myPort.write('\n');
}


/********* Metric Path Planning *********/

/* followPath() uses the position of the robot and the goal to move
 * the robot toward the goal without moving through walls based one
 * the topological map.
*/
void followPath(){

  if((robotX != -1 && robotY != -1 && goalX != -1 && goalY != -1) && (robotX != goalX || robotY != goalY)){
    int options[]={99,99,99,99};
    int smallestNum = 99;
    int directionToDrive = 99;
    int obstacleNum = gridLayout[robotY][robotX];
   
    // check which ways we are able to drive
    if(canMoveNorth(obstacleNum)){
      options[0] = wavefrontGrid[robotX][robotY-1];
    }
    if(canMoveSouth(obstacleNum)){
      options[1] = wavefrontGrid[robotX][robotY+1];
    }
    if(canMoveEast(obstacleNum)){
      options[2] = wavefrontGrid[robotX+1][robotY];
    }
    if(canMoveWest(obstacleNum)){
      options[3] = wavefrontGrid[robotX-1][robotY];
    }
    
    // pick the best direction to drive based on the wavefront number
    for(int i =0; i < 4; i++){
      
      if(options[i] < smallestNum){
        smallestNum = options[i]; 
        directionToDrive = i;
      }
      
      // prioritize turning
      if(options[i] == smallestNum){
        if(lastDirection != i){
          smallestNum = options[i];
          directionToDrive = i;
        }
      }
      
    }
    
    lastDirection = directionToDrive;
    
    // write commands to the robot and update wavefront grid
    if(directionToDrive==0){
      // north
      myPort.write('5');
      println("moved north");
      wavefrontGrid[robotX][robotY] = 99;
      robotY += -1;
    }
    else if(directionToDrive==1){
      // south
      //myPort.write('6');
      println("moved south");
      wavefrontGrid[robotX][robotY] = 99;
      robotY += 1;
    }
    else if(directionToDrive==2){
      // east
      myPort.write('7');
      println("moved east");
      wavefrontGrid[robotX][robotY] = 99;
      robotX += 1;
    }
    else if(directionToDrive==3){
      // west
      myPort.write('8');
      println("moved west");
      wavefrontGrid[robotX][robotY] = 99;
      robotX += -1;
    }
    
    myPort.write('\n');
  
  }
  
  // tell the robot to execute all the commands once the path is fully planned
  if((robotX == goalX && robotY == goalY) && robotX != -1){
    myPort.write('9');
    myPort.write('\n');
  }
  
}


/* The following 4 methods each check if the robot can move in a certain
*  cardinal direction based on the topological obstacle number of the
*  square it is currently in.
*/
boolean canMoveNorth(int obstacleNum){
  if(obstacleNum==0 || obstacleNum==2 || obstacleNum==4 || obstacleNum==6 || obstacleNum ==8 || obstacleNum==10 || obstacleNum==12 || obstacleNum==14){
  return true;
  }
  return false;
}

boolean canMoveSouth(int obstacleNum){
  if(obstacleNum==0 || obstacleNum==1 || obstacleNum==2 || obstacleNum==3 || obstacleNum ==8 || obstacleNum==9 || obstacleNum==10 || obstacleNum==11){
  return true;
  }
  return false;
}

boolean canMoveEast(int obstacleNum){
  if(obstacleNum==0 || obstacleNum==1 || obstacleNum==4 || obstacleNum==5 || obstacleNum ==8 || obstacleNum==9 || obstacleNum==12 || obstacleNum==13){
  return true;
  }
  return false;
}
boolean canMoveWest(int obstacleNum){
  if(obstacleNum==0 || obstacleNum==1 || obstacleNum==2 || obstacleNum==3 || obstacleNum ==4 || obstacleNum==5 || obstacleNum==6 || obstacleNum==7){
  return true;
  }
  return false;
}


/* The SubmitGoal method is called whenever the goal submission button is
*  pressed. Then the current goal is updated and the wavefront propogation
*  is created based on the coordinates entered into the text field.
*/
void SubmitGoal() {
  // read the input from the textfield
  input = cp5.get(Textfield.class,"textInputGoal").getText();
  print("the following goal was submitted :");
  println(" textInputGoal = " + input);
  
  // read input and convert to int
  goalX = input.charAt(0) - '0';
  goalY = input.charAt(1) - '0';
  
  createWavefront();
}


/* The SubmitPosition method is called whenever the robot position submission
*  button is pressed. Then the robot position is updated based on the coordinates
*  entered into the text field.
*/
void SubmitPosition() {
  // read the input from the textfield
  input = cp5.get(Textfield.class,"textInputPosition").getText();
  print("the following goal was submitted :");
  println(" textInputPosition = " + input);
  
  // read the input and convert to int
  robotX = input.charAt(0) - '0';
  robotY = input.charAt(1) - '0';
}


/********* Localization *********/

/* The StartLocalization method is called whenever the localization button
*  is pressed. 
*/
void StartLocalization() {
  println("beginning localization");
  
  int obstacleNum = getObstacleNumFromRobot();  
  findPositionOptions(obstacleNum);
  
  boolean localized = false;
  int iteration = 0;
  
  while(!localized){
    
    turnToNorth();
    driveForward();
    println("moved north");
    
    // update possible locations the robot could be at
    possibleCoords1[0][iteration+1] = possibleCoords1[0][iteration]-1;
    possibleCoords1[1][iteration+1] = possibleCoords1[1][iteration];
    possibleCoords2[0][iteration+1] = possibleCoords2[0][iteration]-1;
    possibleCoords2[1][iteration+1] = possibleCoords2[1][iteration];
    
    delay(3000);
    obstacleNum = getObstacleNumFromRobot();
    
    // check 1st position option
    int i1 = possibleCoords1[0][iteration+1];
    int j1 = possibleCoords1[1][iteration+1];
    int i2 = possibleCoords2[0][iteration+1];
    int j2 = possibleCoords2[1][iteration+1];    
    
    // if the obstacle number doesn't match, this option isn't viable
    if(obstacleNum != gridLayout[i1][j1]){
      robotY = i2;
      robotX = j2;
      robot2Y = possibleCoords2[0][0];
      robot2X = possibleCoords2[1][0];
      localized = true;
    }
    
    // if the obstacle number doesn't match, this option isn't viable
    if(obstacleNum != gridLayout[i2][j2]){
      robotY = i1;
      robotX = j1;
      robot2Y = possibleCoords1[0][0];
      robot2X = possibleCoords1[1][0];
      localized = true;
    }

    iteration++;
  }
  
  println("localized");
  
}


/* The getObstacleNumFromRobot method sends the robot a command to send sensor
* data, then reads the data and uses the getObstacleNumber helper method to
* determine the obstacle number at the robot's current location.
*/
int getObstacleNumFromRobot(){
  
  // request ir sensor data from the robot
  myPort.write('1');
  myPort.write('\n');
  
  boolean keepReading = true;

  int i = 0;
  delay(5000);
  println("getting obstacle num");
  
  while(keepReading){
    // read data from robot
    val = myPort.readStringUntil('\n');
    val = trim(val);
    
    if (val != null) {
      println(val);
      
      if(i < 4){
        irData[i] = val;
        i++;
      }
      if(i >= 4){
        // keep reading from the serial until we have 4 values
        keepReading = false;
      }
      
   }
    
  }
  iterationCount++;
  int obstacleNum = getObstacleNumber();

  return obstacleNum;
  
}

/* The findPositionOptions method compares the obstacle number obtained
*  from the robot's IR readings to the encoded map of the world, and
*  then determines all the possible locations the robot could be.
*/
void findPositionOptions(int obstacleNum){
  for(int i =0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      
       if (gridLayout[i][j] == obstacleNum){
         println("found option");
         
        if(possibleCoords1[0][0] > -1 && possibleCoords1[1][0] > -1){
          possibleCoords2[0][0] = i;
          possibleCoords2[1][0] =  j;
        }
        else{
          possibleCoords1[0][0] = i;
          possibleCoords1[1][0] =  j;
        }
        
      }
      
    }
  }  
  
}


/* The getObstacleNumber method checks the most recent set of IR
* data from the robot and determines the toplogical obstacle number
* of the square based on that data. It then returns the number.
*/
int getObstacleNumber(){
  int obstacleNum = 15;
  boolean frontOpen = false;
  boolean backOpen = false;
  boolean leftOpen = false;
  boolean rightOpen = false;
    
  if(!irData[0].equals("9")){
    frontOpen = true;
  }
  if(!irData[1].equals("9")){
    backOpen = true;
  }
  if(!irData[2].equals("9")){
    rightOpen = true;
  }
  if(!irData[3].equals("9")){
    leftOpen = true;
  }
  
  if(frontOpen && backOpen && leftOpen && rightOpen){
    obstacleNum = 0;
  }
  else if(!frontOpen && backOpen && leftOpen && rightOpen){
    obstacleNum = 1;
  }
  else if(frontOpen && backOpen && leftOpen && !rightOpen){
    obstacleNum = 2;
  }
  else if(!frontOpen && backOpen && leftOpen && !rightOpen){
    obstacleNum = 3;
  }
  else if(frontOpen && !backOpen && leftOpen && rightOpen){
    obstacleNum = 4;
  }
  else if(!frontOpen && !backOpen && leftOpen && rightOpen){
    obstacleNum = 5;
  }
  else if(frontOpen && !backOpen && leftOpen && !rightOpen){
    obstacleNum = 6;
  }
  else if(!frontOpen && !backOpen && leftOpen && !rightOpen){
    obstacleNum = 7;
  }
  else if(frontOpen && backOpen && !leftOpen && rightOpen){
    obstacleNum = 8;
  }
  else if(!frontOpen && backOpen && !leftOpen && rightOpen){
    obstacleNum = 9;
  }
  else if(frontOpen && backOpen && !leftOpen && !rightOpen){
    obstacleNum = 10;
  }
  else if(!frontOpen && backOpen && !leftOpen && !rightOpen){
    obstacleNum = 11;
  }
  else if(frontOpen && !backOpen && !leftOpen && rightOpen){
    obstacleNum = 12;
  }
  else if(!frontOpen && !backOpen && !leftOpen && rightOpen){
    obstacleNum = 13;
  }
  else if(frontOpen && !backOpen && !leftOpen && !rightOpen){
    obstacleNum = 14;
  }
  
  return obstacleNum;
}


/* The createWavefront method calculates the wavefront number
* using manhattan distance for each square on the map based on
* the robot's current goal.
*/
void createWavefront(){
 
  for (int i = 0; i < 4; i++){
     for(int j = 0; j < 4; j++){
       
       manhattanNumber = abs(i-goalX)+abs(j-goalY);
       wavefrontGrid[i][j] = manhattanNumber;
       manhattanNumber=0;
       
     }
  }
 
}


/********* Topological Path Following *********/

/* The topologicalSubmit method is called whenever the button for
*  topological path following is pressed. It then reads the input
*  from the textbox and sends the corresponding commands to the robot.
*/
void topologicalSubmit() {
  
  // read in text input
  input = cp5.get(Textfield.class,"textInput").getText();
  print("the following text was submitted :");
  println(" textInput = " + input);
  
  char currentChar;
  
  // determine command to send to the robot
  // s=1,l=2,r=3,t=4
  for(int i = 0; i < input.length(); i++){
    currentChar = input.charAt(i);
   
    if (currentChar == 'S'){ // Start
    myPort.write('1');
    }
    else if (currentChar == 'L'){ // Left
    myPort.write('2');
    }
    else if (currentChar == 'R'){ // Right
    myPort.write('3');
    }
    else { // Terminate
    myPort.write('4');
    }
    
    myPort.write(currentChar);
    println(currentChar);
    
  }
}

/********* GUI Draw Methods *********/

/* The drawRobot method draws a circle on the screen to represent the robot during
* localization, mapping, and path planning.
*/
void drawRobot(){
  circle(gridXOffset+(robotX*50)+boxWidth/2,gridYOffset+(robotY*50)+boxWidth/2, 30);
}


/* The drawStartRobot method is used by the localization behavior to draw a circle
* on the map where the robot started in its localization.
*/
void drawStartRobot(){
  circle(gridXOffset+(robot2X*50)+boxWidth/2,gridYOffset+(robot2Y*50)+boxWidth/2, 30);
}


/* The drawLineFromClick method uses the number of times a map square has been
* clicked to determine the corresponding topolagical map number and draw the
* appropriate walls.
*/
void drawLineFromClick(int clickNumber, int row, int column){
  
  switch(clickNumber) {
    case 0:
      break;
    case 1:
    drawHorizontalLine(row, column);
      break;
    case 2:
    drawVerticalLine(row, column+1);
      break;
    case 3:
    drawHorizontalLine(row, column);
    drawVerticalLine(row, column+1);
      break;
    case 4:
    drawHorizontalLine(row+1, column);
      break;
    case 5:
    drawHorizontalLine(row, column);
    drawHorizontalLine(row+1, column);
      break;
    case 6:
    drawHorizontalLine(row+1, column);
    drawVerticalLine(row, column+1);
      break;
    case 7:
    drawHorizontalLine(row, column);
    drawHorizontalLine(row+1, column);
    drawVerticalLine(row, column+1);
      break;
    case 8:
    drawVerticalLine(row, column);
      break;
    case 9:
    drawHorizontalLine(row, column);
    drawVerticalLine(row, column);
      break;
    case 10:
    drawVerticalLine(row, column);
    drawVerticalLine(row, column+1);
      break;
    case 11:
    drawVerticalLine(row, column);
    drawVerticalLine(row, column+1);
    drawHorizontalLine(row, column);
      break;
    case 12:
    drawVerticalLine(row, column);
    drawHorizontalLine(row+1, column);
      break;
    case 13:
    drawHorizontalLine(row, column);
    drawHorizontalLine(row+1, column);
    drawVerticalLine(row, column);
      break;
    case 14:
    drawVerticalLine(row, column+1);
    drawHorizontalLine(row+1, column);
    drawVerticalLine(row, column);
      break;
    case 15:
    drawVerticalLine(row, column+1);
    drawHorizontalLine(row+1, column);
    drawVerticalLine(row, column);
    drawHorizontalLine(row, column);
      break;
    
  }
  
}


/* The drawHorizontalLine method draws a line on the GUI at the 
* given row and column of the map.
*/
void drawHorizontalLine(int row, int col){
  strokeWeight(4);
  line((col*boxWidth)+gridXOffset, (row*boxWidth)+gridYOffset, ((col+1)*boxWidth)+gridXOffset, ((row)*boxWidth)+gridYOffset);
}


/* The drawVerticalLine method draws a line on the GUI at the 
* given row and column of the map.
*/
void drawVerticalLine(int row, int col){
  strokeWeight(4);
  line((col*boxWidth)+gridXOffset, (row*boxWidth)+gridYOffset, ((col)*boxWidth)+gridXOffset, ((row+1)*boxWidth)+gridYOffset);
}


/* The displayMap method takes in a 2D array of rectangles and
* draws them all on the window.
*/
void displayMap( PShape[][] map) {

  for ( int i= 0; i< 4; i++) {
    for ( int j=0; j< 4; j++) {
      shape(map[i][j]);
    }
  }
  
}


/* The getMouselocation determines which map square the mouse
* is currently in and returns the row and column of the square.
* If the mouse is not in the map, it returns [-1, -1].
*/
int[] getMouseLocation(){
  int x = mouseX;
  int y = mouseY;
  int location [] = {-1, -1};
  
  for ( int i= 1; i<= 4*50; i+=boxWidth) {
    for ( int j=1; j<= 4*50; j+=boxWidth) {
      
      if((x > gridXOffset+i && x < gridXOffset+(i+boxWidth))  &&  (y > gridYOffset+j && y < gridYOffset+(j+boxWidth) )){
          
          location[0] = j/boxWidth;
          location[1] = i/boxWidth;
          
      }
      
    }
  }
  
  return location;
  
}
