#include <Servo.h> 

String readString;

Servo s0;
Servo s1;
Servo s2;

int a0 = 0;
int a1 = 0;

boolean readToggle = true;
int axis;

/**
* In the serial window type <axis #> and enter, then <angle> and enter to
* send position command do that axis. 
*/

void setup() {
  s0.attach(9);
  s1.attach(8);
  s2.attach(7);
  servoWrite(1,135);
  servoWrite(0,90);
  Serial.begin(9600);
  Serial.println("servo-test-22-dual-input"); // so I can keep track of what is loaded
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) {
    //Serial.println(readString);  //so you can see the captured string 
    int n = readString.toInt();  //convert readString into a number
  
    if (readToggle) {
      // Axis
      axis = n;
      if (n > 3) {
        Serial.print("Re-enter axis:");
        readToggle = !readToggle;
      } else {
        Serial.print("Axis: ");
      }
    } else {
      // Position
      Serial.print("Writing Angle: ");
      servoWrite(axis, n);
    }
    Serial.println(n);
    readToggle = !readToggle;
    readString="";
  } 
}

void servoWrite(int axis, int pos) {
  switch (axis) {
    case 0:
      s0.write(pos);
      break;
    case 1:
      s1.write(pos);
      s2.write(pos);
      break;
  }
}
