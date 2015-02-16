

/ gt-usli-2015 flight software ver 0.0
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
//100ms
enum states
{
    INIT,             /* Initalize avionics, error check and report */
    LAUNCH_STDBY,     /* Unarmed on launchpad */
    LAUNCH_RDY,       /* Armed on launchpad */
    LIFTOFF,          /* Powered flight until clear of launch rail */
    PWR_ASC,          /* Powered flight */
    DESCENT          /* Descent with drogue deployed */
};

/* Boot into INIT state */
enum states state = INIT;

/* Hardware connections */
Servo servos[3];
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // Why is 12345 specified
const int chipSelect = 4; // Figure out what this does
const int ledPin = 20; // This is a random value, change to the right one!

/* Constants */
const int LAUNCH_RAIL_TIME_MS = 100;
const int MILLIS = 1000; // Milliseconds in a second, for converting from hertz to millis
const int LAUNCH_RDY_SPEED = MILLIS / 1000; // Loop at 1000 Hz
const int PWR_ASC_SPEED = MILLIS / 20; // Loop at 20 Hz

/* Variables */
int i;
int loop_time_ms = MILLIS / 20; // Filler value, gets overwritten asap
unsigned long init_time;
unsigned long init_launch_time;
int currAltitude, prev_altitude;

sensors_event_t accelEvent;
unsigned long lastLedBlink = 0;
bool ledOn = 0;



boolean active_control = false;
File file;

void setup()
{
}

void loop()
{
    init_time = millis();

    switch (state) {
    case INIT:
        state_init();
        break;
    case LAUNCH_STDBY: 
        state_launch_stdby();
        break;
    case LAUNCH_RDY:
        state_launch_rdy();
        break;
    case LIFTOFF:
        state_liftoff();
        break;
    case PWR_ASC:
        state_pwr_asc();
        break;
    }

    long delay_time = loop_time_ms - (millis() - init_time);
    if (delay_time > 0) delay(delay_time);
    else Serial.println("Trying to loop too quickly.");
}

void state_init()
{
    Serial.begin(9600);

    /* Initialise ADXL345 accelerometer */
    if(!accel.begin())
    {
        Serial.println("No ADXL345 detected ... Check your wiring!");
    } else {
        Serial.println("ADXL345 initialized.");
        accel.setRange(ADXL345_RANGE_16_G);
        active_control = true; // Only use active control if accelerometer is found
    }

    /* Initialize SD Card */
    pinMode(10, OUTPUT); // Necessary even if chip select pin isn't used.

    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
    } else {
        Serial.println("SD Card initialized.");
        file = SD.open("data.txt", FILE_WRITE); // File name <= 8 chars.
    }

    /* Initialize servo motors */
    int pins[3] = {14,15,20};
    for (i = 0; i < 3; i++)
        servos[i].attach(pins[i]);
}

void state_launch_stdby()
{
    if (millis() - lastLedBlink > MILLIS / 10) // Blink at 10 Mhz
    {
        ledOn = !ledOn;
        digitalWrite(ledPin, ledOn);
    }
    /* STATE CHANGE */
    if (true) // If arm_button_pressed();
    {
        digitalWrite(ledPin, HIGH);
        loop_time_ms = LAUNCH_RDY_SPEED; // Loop at max speed (1000 Mhz)
        state = LAUNCH_RDY;
    }
}

void state_launch_rdy()
{
    /* Update accel data only, don't worry about logging */
    accel.getEvent(&accelEvent);

    /* STATE CHANGE */
    if (accelEvent.acceleration.y > 3) {
        loop_time_ms = PWR_ASC_SPEED;
        init_launch_time = millis();
        state = PWR_ASC;
    }
}

void state_liftoff()
{
    updateAndLog();

    /* STATE CHANGE */
    if (millis() - init_launch_time > LAUNCH_RAIL_TIME_MS) state = PWR_ASC;
}

void state_pwr_asc()
{
    updateAndLog();

    if (active_control)
    {
        // Still work to be done here!
        updateAltitude();
        int ndx = init_launch_time / loop_time_ms;
        int ideal_h = table_h[ndx];
        int ideal_v = table_v[ndx];
    }
    
    /* STATE CHANGE */
    //return true if last altitude > current altitude (ie we are past apogee)
    int alt = 0; // This is temporary!
    if (prev_altitude > alt) state = DESCENT;
}

void state_descent()
{
    updateAndLog();
}

/* Subroutines */

/*
 * Get a servo theta value given an altitude and 
 * velocity.
 */
int lookup(double altitude, double deltaVel)
{
    // Check if in range first
    // return lookup[altitude][deltaVel];
    return 0;
}

void setServos(int theta)
{
    // 0->160 , 110->50
    int pos = 160-theta;
    for(i = 0; i < 3; i++)
      servos[i].write(pos);
}

/*
 * Updates sensor data and logs to SD card.
 */
void updateAndLog()
{
    accel.getEvent(&accelEvent);
    char buf[100]; // Is 100 enough?
    sprintf(buf, "Time: %ul, ALT: %d, ACCEL(x,y,z), %f, %f, %f\n", 
        millis() - init_launch_time, currAltitude, accelEvent.acceleration.x, 
        accelEvent.acceleration.y, accelEvent.acceleration.z);
}

/* 
 * Saves string to SD card, returns true if worked
 * else return false.
 */
boolean logSD(char* str)
{
    if (file)
    {
        file.println(str);
        file.flush();
        return true;
    } else return false;
}

/*
 * Returns an integer altitude when called.
 * TODO: Test how quickly this can be called!
 */
int updateAltitude()
{
    if (Serial1.available())
    {
        static char input[16];
        static uint8_t i;
        char c = Serial1.read();
        if (c!- '\r' && i < 15) // CR is chosen in serial monitor as line end
            input[i++] = c;
        else
        {
            input[i] = '\0';
            i = 0;
            int alt_ft = atoi(input);
            alt_m = alt_ft * .304;
        }
    }
    previousAltitude = currAltitude;
    currAltitude = alt_m
}
