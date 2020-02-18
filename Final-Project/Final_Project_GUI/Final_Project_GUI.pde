import processing.serial.*;
 Serial myPort;
 String val;
boolean firstContact = false;
PShape[][] map = new PShape[4][4]; 
PFont font;
//int[][] gridLayout = new int[4][4];

int gridLayout[][] =
    {
      // Occupancy Grid
      {11, 15, 13,  3},
      {10, 15, 15, 10},
      { 8,  5,  5,  2},
      {14, 15, 15, 14}
    };
    
//int gridLayout[][] =
//    {
//      // Topological
//      { 9,  1,  7, 11},
//      {14,  8,  5,  2},
//      { 9,  6,  9,  6},
//      {14, 13,  6, 15}
//    };
    


int gridXOffset = 250;
int gridYOffset = 100;
int robotX = -1;
int robotY  = -1;
int goalX = -1;
int goalY = -1;

int boxWidth = 50;
String input;
String lastText = "";
int manhattanNumber=0;
int[][] wavefrontGrid = new int[4][4];


import controlP5.*;
ControlP5 cp5;

String textValue = "";

Textfield myTextfield;


void setup() {
 
  
  font = loadFont("Bahnschrift-32.vlw");
  textFont(font, 32);
  
  
  cp5 = new ControlP5(this);
  cp5.addTextfield("textInput").setPosition(400, 450).setSize(200, 40).setAutoClear(false);
  cp5.addBang("Submit").setPosition(400, 500).setSize(80, 40); 
  
  cp5.addTextfield("textInputGoal").setPosition(100, 450).setSize(200, 40).setAutoClear(false);
  cp5.addBang("SubmitGoal").setPosition(100, 500).setSize(80, 40); 
  
  cp5.addTextfield("textInputPosition").setPosition(100, 550).setSize(200, 40).setAutoClear(false);
  cp5.addBang("SubmitPosition").setPosition(100, 600).setSize(80, 40); 
  
  
  size(700, 800); //make our canvas 200 x 200 pixels big
  //  initialize your serial port and set the baud rate to 9600
  myPort = new Serial(this, "COM7", 9600);
  myPort.bufferUntil('\n'); 
  
  for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++){
     
     stroke(0);
     map[i][j] = createShape( RECT, gridXOffset+(i*50),gridYOffset+(j*50),50,50);
   }
 } 
  
}

void draw() {
  
  background(255);
  
  displayMap(map);
  
  fill(0);
  text("Sensor Data", 100, 400);
  text("Commands", 400, 400);
  
  drawRobot();
  
  
  //val = myPort.readStringUntil('\n');
  //make sure our data isn't empty before continuing
  if (val != null) {
    //trim whitespace and formatting characters (like carriage return)
    val = trim(val);
    text(val, 100, 450);
    println(val);
    lastText = val;
  }
  text(lastText, 100, 450);
    
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
  
  for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++){
     drawLineFromClick(gridLayout[i][j], i, j);
   }
 }
  
  for(int i=0;i<4;i++) {
   for(int j=0;j<4;j++){
     
     stroke(0);
     text(wavefrontGrid[i][j], gridXOffset+10+(i*50),gridYOffset+40+(j*50));
   }
 } 
 
 followPath();
 
  
}


void followPath(){

  if((robotX != -1 && robotY != -1 && goalX != -1 && goalY != -1) && (robotX != goalX || robotY != goalY)){
    int options[]={99,99,99,99};
    int smallestNum = 99;
    int directionToDrive = 99;
    int obstacleNum = gridLayout[robotY][robotX];
   
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
    
    for(int i =0; i < 4; i++){
      if(options[i] <= smallestNum){
        smallestNum = options[i];
        //becasue want to defer to turning if two numbers of equal size 
        directionToDrive = i;
      }
    }
    
    if(directionToDrive==0){
      // north
      myPort.write('5');
      println("moved north");
      robotY += -1;
    }
    else if(directionToDrive==1){
      // south
      myPort.write('6');
      println("moved south");
      robotY += 1;
    }
    else if(directionToDrive==2){
      // east
      myPort.write('7');
      println("moved east");
      robotX += 1;
    }
    else if(directionToDrive==3){
      // west
      myPort.write('8');
      println("moved west");
      robotX += -1;
    }
    myPort.write('\n');
  
  }
  
  //delay(500);
 
  
  
}

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


void Submit() {
  print("the following text was submitted :");
  input = cp5.get(Textfield.class,"textInput").getText();
  println(" textInput = " + input);
  char currentChar;
  // s=1,l=2,r=3,t=4
  for(int i = 0; i < input.length(); i++){
    currentChar = input.charAt(i);
   
    if (currentChar == 'S'){
    myPort.write('1');
    }
    else if (currentChar == 'L'){
    myPort.write('2');
    }
    else if (currentChar == 'R'){
    myPort.write('3');
    }
    else{
    myPort.write('4');
    }
    myPort.write(currentChar);
    println(currentChar);
    
    
  }
  
}

void SubmitGoal() {
  print("the following goal was submitted :");
  input = cp5.get(Textfield.class,"textInputGoal").getText();
  println(" textInputGoal = " + input);
  goalX = input.charAt(0) - '0';
  goalY = input.charAt(1) - '0';
  createWavefront();
  // how to overlap the wavefront numbers with the obsatcles and get that to work together
}

void SubmitPosition() {
  print("the following goal was submitted :");
  input = cp5.get(Textfield.class,"textInputPosition").getText();
  println(" textInputPosition = " + input);
  robotX = input.charAt(0) - '0';
  robotY = input.charAt(1) - '0';
}

void drawRobot(){
  
  circle(gridXOffset+(robotX*50)+boxWidth/2,gridYOffset+(robotY*50)+boxWidth/2, 30);
  
}

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


void drawHorizontalLine(int row, int col){
  strokeWeight(4);
  line((col*boxWidth)+gridXOffset, (row*boxWidth)+gridYOffset, ((col+1)*boxWidth)+gridXOffset, ((row)*boxWidth)+gridYOffset);
}

void drawVerticalLine(int row, int col){
  strokeWeight(4);
  line((col*boxWidth)+gridXOffset, (row*boxWidth)+gridYOffset, ((col)*boxWidth)+gridXOffset, ((row+1)*boxWidth)+gridYOffset);
}


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



void displayMap( PShape[][] map) {

  for ( int i= 0; i< 4; i++) {
    for ( int j=0; j< 4; j++) {
      shape(map[i][j]);
    }
  }
  
}


void createWavefront(){
 
  for (int i = 0; i < 4; i++){
     for(int j = 0; j < 4; j++){
       
       manhattanNumber = abs(i-goalX)+abs(j-goalY);
       wavefrontGrid[i][j] = manhattanNumber;
       manhattanNumber=0;
       
     }
  }
 
}
