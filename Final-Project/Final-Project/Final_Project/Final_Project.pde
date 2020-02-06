import processing.serial.*;
 Serial myPort;
 String val;
// since we're doing serial handshaking, 
// we need to check if we've heard from the microcontroller
boolean firstContact = false;
PShape[][] map = new PShape[6][6]; 
PFont font;

import controlP5.*;

ControlP5 cp5;

String textValue = "";

Textfield myTextfield;


void setup() {
  
  font = loadFont("Bahnschrift-32.vlw");
  textFont(font, 32);
  
  size(700, 600); //make our canvas 200 x 200 pixels big
  //  initialize your serial port and set the baud rate to 9600
  //myPort = new Serial(this, Serial.list()[3], 9600);
  //myPort.bufferUntil('\n'); 
  
   for(int i=0;i<6;i++) {
   for(int j=0;j<6;j++){
     
     stroke(0);
     map[i][j] = createShape( RECT, 200+(i*50),30+(j*50),50,50);
   }
 } 
  
}

void draw() {
  
  displayMap(map);
  
  fill(0);
  text("Sensor Data", 100, 400);
  text("Commands", 400, 400);
  
  
  
}



void displayMap( PShape[][] map) {

  for ( int i= 0; i< 6; i++) {
    for ( int j=0; j< 6; j++) {
      shape(map[i][j]);
    }
  }
  
}


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
