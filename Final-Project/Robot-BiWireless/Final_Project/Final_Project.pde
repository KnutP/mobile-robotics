import processing.serial.*;
 Serial myPort;
 String val;
// since we're doing serial handshaking, 
// we need to check if we've heard from the microcontroller
boolean firstContact = false;
PShape[][] map = new PShape[4][4]; 
PFont font;
int[][] gridLayout = new int[4][4];
int gridXOffset = 250;
int gridYOffset = 100;
int boxWidth = 50;
String input;
String lastText = "";

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
  
  
  size(700, 600); //make our canvas 200 x 200 pixels big
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
  
  
  if (mousePressed == true ) 
    {             //if we clicked in the window
      //myPort.write('1');        //send a 1
      println("2");
      mousePressed = false;
    }
    
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
  
  
}


void Submit() {
  print("the following text was submitted :");
  input = cp5.get(Textfield.class,"textInput").getText();
  println(" textInput = " + input);
//  byte topologicalPath [] = {};
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
  //myPort.write(input);
  //println(input);
  
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

/*
void serialEvent( Serial myPort) {
//put the incoming data into a String - 
//the '\n' is our end delimiter indicating the end of a complete packet
val = myPort.readStringUntil('\n');
//make sure our data isn't empty before continuing
if (val != null) {
  //trim whitespace and formatting characters (like carriage return)
  val = trim(val);
  println(val);

  //look for our 'A' string to start the handshake
  //if it's there, clear the buffer, and send a request for data
  if (firstContact == false) {
    if (val.equals("A")) {
      myPort.clear();
      firstContact = true;
      myPort.write("A");
      println("contact");
    }
  }
  else { //if we've already established contact, keep getting and parsing data
    println(val);

    if (mousePressed == true) 
    {                           //if we clicked in the window
      myPort.write('1');        //send a 1
      println("1");
    }

    // when you've parsed the data you have, ask for more:
    myPort.write("A");
    }
  }
}
*/
