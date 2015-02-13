
// gt-usli-2015 flight software ver 0.0
// dependent on adafruit sensor lib

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <ITG3200.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "tables.h"


// Generic catch-all implementation.
template <typename T_ty> struct TypeInfo { static const char * name; };
template <typename T_ty> const char * TypeInfo<T_ty>::name = "unknown";

// Handy macro to make querying stuff easier.
#define TYPE_NAME(var) TypeInfo< typeof(var) >::name

// Handy macro to make defining stuff easier.
#define MAKE_TYPE_INFO(type)  template <> const char * TypeInfo<type>::name = #type;

// Type-specific implementations.
MAKE_TYPE_INFO( int )
MAKE_TYPE_INFO( float )
MAKE_TYPE_INFO( short )

enum states
{
  INIT,             /* Initalize avionics, error check and report */
  LAUNCH_STDBY,     /* Unarmed on launchpad */
  LAUNCH_RDY,       /* Armed on launchpad */
  LIFTOFF,          /* Powered flight until clear of launch rail */
  PWR_ASC,          /* Powered flight */
  UPWR_ASC,         /* Ascent after motor burnout */
  RCV_DROG,         /* Descent with drogue deployed */
  RCV_PLD,          /* Descent after payload jettison */
  RCV_MAIN,         /* Descent with main parachute deployed */
  LANDED            /* On ground */
};

/* Boot into INIT state */
enum states state = INIT; 

/* Hardware connections */
Servo servos[3];
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const int chipSelect = 4;

/* Variables */
int i;
File file;
int delta_t = 24; // ms
unsigned int long time_0;
unsigned int long time;
int alt_m;
double epsilon;
int theta;

void setup(void) 
{
  Serial.begin(9600);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }*/
  //Serial.println("hello0");

  /* Initialise ADXL345 accelerometer */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  //Serial.println("hello1");

  accel.setRange(ADXL345_RANGE_16_G);  /* Set accelerometer range */
  
  /* Initialize SD Card */
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // Open file to write to, file length must be under 8 chars
  file = SD.open("data1.txt", FILE_WRITE);
  
  /* Initialize servo motors */
  int pins[3] = {21,22,23};
  for(i = 0; i < 3; i++) {
    servos[i].attach(pins[i]);
  }
  setServos(0);

}

void loop() 
{ 
  switch (state) {
  case INIT:
    state_init();
    break;
  case LAUNCH_STDBY:
    state_launch_stdby();
    break;
  case LIFTOFF:
    state_liftoff();
    break;
  case PWR_ASC:
    state_pwr_asc();
    break;
   }
}

/* State routines */

void state_init(void)
{
//  time += 0.5f;
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");  
  
  state = LAUNCH_STDBY;
}

void state_launch_stdby(void)
{
  sensors_event_t event;
  accel.getEvent(&event);
  writeAccelerometer(event, 0, 0, 0);
  /* Loop until acceleration passes 2g */
  /* MAKE SURE TO USE VERTICLAL COMPONEENT OF ACCELEROMETER */
  while(abs(event.acceleration.y) < 18) {
     /* Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
      Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
      Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");  
      */
    delay(500);
    accel.getEvent(&event);
  }
  writeAccelerometer(event, 0, 0, 0);
  /* Acceleration passed threshold, liftoff */
  state = LIFTOFF;
}

void state_liftoff() 
{
  Serial.println("liftoff!!");
  time_0 = millis();
  /* Start the clock */
  /* Delay for rocke to clear rail */
  delay(370);
  /* Extend servos to nominal position */
  setServos(35);
  state = PWR_ASC;
}
 
void state_pwr_asc()
{
  while((millis()-time_0)<35000) { // loop for time of flight + extra
  
    unsigned int long initial = millis();
    
    time = millis() - time_0;
    int ndx = time/delta_t;
    if(ndx > 500) {
      ndx = 500;
    }
    
    int ideal_h = table_h[ndx];
    //Serial.println(ideal_h);
    /* Read altimeter in ft, convert to meters */
    if(Serial1.available()) {
       static char input[16];
       static uint8_t i;
       char c = Serial1.read ();
       if ( c != '\r' && i < 15 ) // assuming "Carriage Return" is chosen in the Serial monitor as the line ending character
         input[i++] = c;
       else
       {
         input[i] = '\0';
         i = 0;
         int alt_ft = atoi( input );
         alt_m =alt_ft*.304;
       }  
    }
    /* control with alt_m and ideal_h */
    epsilon = alt_m - ideal_h;      //calculation of eps
    theta=get_theta(epsilon);       //get theta from Control.h
    setServos(theta);
    
    sensors_event_t event;
    accel.getEvent(&event);
    
    writeAccelerometer(event, alt_m, theta, time);
    int dly = delta_t*3 - (millis()-initial);
    dly = max(dly, 1);
    delay(dly);
  }
  setServos(0);
}

/* Subroutines */

void setServos(int theta)
{
  // 0->160 , 110->50
  if(theta > 90) {
    theta = 90;
  } else if(theta < 0) {
    theta = 0;
  }
  int pos = 160-theta;
  for(i = 0; i < 3; i++) {
    servos[i].write(pos);
  }
}


int get_theta(double eps){
  int theta;
  if (abs(eps)<=0.5){
      theta = 35;}
  else if (eps>=0){
      theta = 90;}
  else if (eps<=0){
      theta = 0;}
  return theta;
}

void writeAccelerometer(sensors_event_t event, int alt, int pin_angle, unsigned int long t)
{
  char buf[0x100];
  snprintf(buf, sizeof(buf), ", %f, %f, %f, %f, %f\n", event.acceleration.x, 
                              event.acceleration.y, event.acceleration.z, alt, pin_angle);
                              
//  Serial.println(toWrite);
  //Serial.print(millis());
  //Serial.println(buf);
  //Serial.print("time: ");
  // if the file is available, write to it:
  if (file) {
    //file.print(millis());
    file.print(t);
    //Serial.println(t);
    file.print(" ");
    file.println(buf);
    // saves what was written to the file
    file.flush();
    // print to the serial port too:
    //Serial.println("Writing to SD");
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
