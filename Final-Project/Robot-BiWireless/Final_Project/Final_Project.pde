import processing.serial.*;
 Serial myPort;
 String val;
// since we're doing serial handshaking, 
// we need to check if we've heard from the microcontroller
boolean firstContact = false;
PShape[][] map = new PShape[6][6]; 
PFont font;
int[][] gridLayout = new int[6][6];

import controlP5.*;

ControlP5 cp5;

String textValue = "";

Textfield myTextfield;


void setup() {
  
  font = loadFont("Bahnschrift-32.vlw");
  textFont(font, 32);
  
  size(700, 600); //make our canvas 200 x 200 pixels big
  //  initialize your serial port and set the baud rate to 9600
  myPort = new Serial(this, "COM7", 9600);
  myPort.bufferUntil('\n'); 
  
  for(int i=0;i<6;i++) {
   for(int j=0;j<6;j++){
     
     stroke(0);
     map[i][j] = createShape( RECT, 200+(i*50),30+(j*50),50,50);
   }
 } 
  
}

void draw() {
  background(255);
  
  displayMap(map);
  
  fill(0);
  text("Sensor Data", 100, 400);
  text("Commands", 400, 400);
  
    //  if (mousePressed == true ) 
    //{             //if we clicked in the window
    //  myPort.write('1');        //send a 1
    //  println("1");
    //  mousePressed = false;
    //}
    
  if (mousePressed == true){
    
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
  
  for(int i=0;i<6;i++) {
   for(int j=0;j<6;j++){
     drawLineFromClick(gridLayout[i][j], i, j);
   }
 }
  
  
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
    
      break;
    case 5:
    
      break;
    case 6:
    
      break;
    case 7:
    
      break;
    case 8:
    
      break;
    case 9:
    
      break;
    case 10:
    
      break;
    case 11:
    
      break;
    case 12:
    
      break;
    case 13:
    
      break;
    case 14:
    
      break;
    case 15:
    
      break;
    
  }
  
}


void drawHorizontalLine(int row, int col){
  strokeWeight(4);
  line((col*50)+200, (row*50)+30, ((col+1)*50)+200, ((row)*50)+30);
}

void drawVerticalLine(int row, int col){
  strokeWeight(4);
  line((col*50)+200, (row*50)+30, ((col)*50)+200, ((row+1)*50)+30);
}


int[] getMouseLocation(){
  int x = mouseX;
  int y = mouseY;
  int location [] = {0, 0};
  
  for ( int i= 1; i<= 6*50; i+=50) {
    for ( int j=1; j<= 6*50; j+=50) {
      
      if((x > 200+i && x < 200+(i+50))  &&  (y > 30+j && y < 30+(j+50) )){
          
          location[0] = i/50;
          location[1] = j/50;
          
      }
      
    }
  }
  
  return location;
  
}



void displayMap( PShape[][] map) {

  for ( int i= 0; i< 6; i++) {
    for ( int j=0; j< 6; j++) {
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
