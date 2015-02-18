/* gt-usli-2015 flight software ver 0.3 */
/*
 * About:
 * Flight code for the teeensy in the rocket body.
 * There is a function for every state that is 
 * called in a loop with a speed based on loop_time_ms.
 * Update methods update global variables that are 
 *     relied upon in different parts of the code.
 * Log methods log data that is updated using the 
 *     update methods.
 */
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
    DESCENT           /* Descent with drogue deployed */
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
int loop_time_ms = 0;
unsigned long init_time;
unsigned long init_launch_time;
int curr_altitude, prev_altitude;

sensors_event_t accelEvent;
unsigned long lastLedBlink = 0;
bool ledOn = 0;
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
}

/*
 * Initialize accelerometer, SD card and servo motors.
 * Report errors to serial
 * (Altimeter doesn't need to be initialized, may need
 * to put in error check for it)
 */
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

    state = LAUNCH_STDBY;
    loop_time_ms = 0;
}

/*
 * Standby state, blink LED and wait for arm switch
 * to be triggered. Run as fast as possible because
 * we are polling the arm button.
 */
void state_launch_stdby()
{
    if (millis() - lastLedBlink > MILLIS / 10) // Blink at 10 Mhz
    {
        ledOn = !ledOn;
        digitalWrite(ledPin, ledOn);
    }
    /* STATE CHANGE */
    if (true) // If arm_button_pressed
    {
        digitalWrite(ledPin, HIGH);
        loop_time_ms = LAUNCH_RDY_SPEED; // Loop at max speed (1000 Mhz)
        state = LAUNCH_RDY;
    }
}

/*
 * Loop as fast as we can get accelerometer readings.
 * When we detect an upwards acceleration (maybe filter
 * a bit?) go into liftoff state.
 */
void state_launch_rdy()
{
    /* Update accel data only, don't worry about logging */
    accel.getEvent(&accelEvent);

    /* STATE CHANGE */
    if (abs(accelEvent.acceleration.y) > 18) {
        loop_time_ms = PWR_ASC_SPEED;
        init_launch_time = millis();
        state = PWR_ASC;
    }
}

/*
 * Update and log until we clear the launch rail
 * based on amount of time since launch started.
 */
void state_liftoff()
{
    updateLogAll();

    /* STATE CHANGE */
    if (millis() - init_launch_time > LAUNCH_RAIL_TIME_MS) {
        /* Extend servos to nominal position */
        setServos(35);
        state = PWR_ASC;
    }
}

/*
 * Active control state, update and log, then based
 * on that information, calculate our new servo 
 * positions. Exit when a current altitude is
 * less than a previous altitude (reached apogee).
 */
void state_pwr_asc()
{
    updateLogAll();

    int ndx = init_launch_time / loop_time_ms;
    if (ndx > 500) ndx = 500;
    int ideal_h = table_h[ndx];

    int epsilon = curr_altitude - ideal_h;
    setServos(getTheta(epsilon));
    
    /* STATE CHANGE */
    //return true if last altitude > current altitude (ie we are past apogee)
    if (prev_altitude > curr_altitude) {
        state = DESCENT;
        setServos(0);
    }
}

/*
 * Update and log sensor values only.
 */
void state_descent()
{
    updateLogAll();
}

/* Subroutines */

void setServos(int theta)
{
    // 0->160 , 110->50
    if (theta > 90)
        theta = 90;
    else if (theta < 0)
        theta = 0;
    int pos = 160 - theta;
    for (i = 0; i < 3; i++)
        servos[i].write(pos);
}

/*
 * Update accelerometer and altimeter and log all information.
 */
void updateLogAll()
{
    accel.getEvent(&accelEvent);
    updateAltitude();
    logAll();
}

/*
 * Log information from time, accelerometer event, and whatever
 * is in the currAltitude variable.
 */
void logAll()
{
    char buf[100];
    sprintf(buf, "TIME: %ul, ACCEL(x,y,z), %f, %f, %f, ALT: %f", 
        millis() - init_launch_time, accelEvent.acceleration.x, 
        accelEvent.acceleration.y, accelEvent.acceleration.z, 
        curr_altitude);
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
 * Updates previousAltitude and currAltitude to
 * reflect the latest info from the altimeter.
 * TODO: Test how quickly this can be called!
 */
void updateAltitude()
{
    if (Serial1.available())
    {
        static char input[16];
        static uint8_t i;
        char c = Serial1.read();
        if (c != '\r' && i < 15) // CR is chosen in serial monitor as line end
            input[i++] = c;
        else
        {
            input[i] = '\0';
            i = 0;
            int alt_ft = atoi(input);
            int alt_m = alt_ft * .304;
            prev_altitude = curr_altitude;
            curr_altitude = alt_m;
        }
    }
}

/*
 * Returns a theta value for the servos given a
 * difference between current altitude and ideal
 * altitude.
 */
int getTheta (double eps) {
    int theta;
    if (abs(eps) <= 0.5)
        theta = 35;
    else if (eps >= 0)
        theta = 90;
    else if (eps < 0)
        theta = 0;
    return theta;
}
