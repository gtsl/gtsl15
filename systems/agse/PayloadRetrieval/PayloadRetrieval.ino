#include <Servo.h> 

String readString;

Servo s1;
Servo s2;
Servo s3;
Servo s4;
Servo s5;
Servo s6;
Servo s7; //25-160


//int a0 = 0;
//int a1 = 0;


boolean readToggle = true;
int axis;
/*  joint_1: 1
    joint_2: 2
    joint_3: 3 
    Joint_4: 4  */

void setup() {
  s1.attach(3); //joint 1
  s2.attach(4); //joint 2 
  s3.attach(5);  //joint 3
  
  s4.attach(6);   //joint 4
  s5.attach(7);   //joint 5
  s6.attach(8);   //spinner
  s7.attach(10);   //manipulator  
  
  //servoWrite(1,135);
  //servoWrite(0,90);
  Serial.begin(9600);
  Serial.println("PLIS Test"); 
}

void loop() {
  
  /*Serial Testing*/
  
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
      if (n > 50) {
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
 
  /* Payload Retrieval */
 
   //servoWrite(0, 90);
 
}

void servoWrite(int axis, int pos) {
  switch (axis) {
    //joint 1
    case 1:
      s1.write(pos);
      break;
    //joint 2 / joint 3
    case 23:
      s2.write(pos);
      s3.write(pos);
      break;
    //z_axis
    case 4:
      s4.write(pos);

      break;
    case 5:
      s5.write(pos);
      break;
    case 6:
      s6.write(pos);
      break;
    case 7:
      s7.write(pos);
      break;
  }
}
