/* gt-usli-2015 flight software ver 0.3 */
/*
 * About:
 * Flight code for the Teensy in the rocket body.
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
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#include "tables.h"

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
    LAUNCH_RDY,       /* Armed on launchpad */
    LIFTOFF,          /* Powered flight until clear of launch rail */
    PWR_ASC,          /* Powered flight */
    DESCENT           /* Descent with drogue deployed */
};

/* Boot into LAUNCH_RDY state */
enum states state = LAUNCH_RDY;

/* Hardware connections */
Servo servos[3];
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const int chip_select = 4;
const int led_pin = 17;

/* Constants */
const int LAUNCH_RAIL_TIME_MS = 100;
const int MILLIS = 1000; // Milliseconds in a second, for converting from hertz to millis
const int LAUNCH_RDY_SPEED = MILLIS / 1000; // Loop at 1000 Hz (< 3200 Hz limit)
const int PWR_ASC_SPEED = MILLIS / 10; // Loop at 10 Hz (< 20 Hz limit)
const int LAUNCH_DETECT_THRESHOLD = 18; // m/s^2, 2gs
const int LAUNCH_DETECT_COUNTER_THRESHOLD = 7; // Number of times LAUNCH_DETECT_THRESHOLD must be read
const int APOGEE_DETECT_COUNTER_THRESHOLD = 20;

/* Variables */
int loop_time_ms = LAUNCH_RDY_SPEED;
unsigned long last_time;
unsigned long init_launch_time;
int curr_altitude, prev_altitude;
char alt_input[16];
int alt_input_index = 0;
int launch_detect_counter = 0;
int launch_time;
int apogee_detect_counter = 0;
boolean alt_updated = false;

sensors_event_t accel_event;
unsigned long last_led_blink = 0;
bool last_on = 0;
File file;


/*
 * Initialize accelerometer, SD card and servo motors.
 * Report errors to serial
 * (Altimeter doesn't need to be initialized, may need
 * to put in error check for it)
 */
void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600); // Altimeter Serial
    
    while (!Serial) {
      ;
    }
    
    digitalWrite(led_pin, HIGH);
    /* Initialise ADXL345 accelerometer */
    //if(!accel.begin())
    //{
    //    Serial.println("No ADXL345 detected ... Check your wiring!");
    //} else {
    //    Serial.println("ADXL345 initialized.");
    //    accel.setRange(ADXL345_RANGE_16_G);
    //}

    /* Initialize SD Card */
    pinMode(10, OUTPUT); // Necessary even if chip select pin isn't used.

    if (!SD.begin(chip_select)) {
        Serial.println("Card failed, or not present");
    } else {
        Serial.println("SD Card initialized.");
        file = SD.open("data.txt", FILE_WRITE); // File name <= 8 chars.
    }

    /* Initialize servo motors */
    int pins[3] = {21, 22, 23};
    for (int i = 0; i < 3; i++)
        servos[i].attach(pins[i]);

    state = LAUNCH_RDY;
    loop_time_ms = 0;
}

void loop()
{
    if (millis() - last_time > loop_time_ms) {
        switch (state) {
        case LAUNCH_RDY:
            //state_launch_rdy();
            log_all();
            break;
        case LIFTOFF:
            state_liftoff();
            break;
        case PWR_ASC:
            state_pwr_asc();
            break;
        case DESCENT:
            state_descent();
            break;
        }
        last_time = millis();
    }
    alt_updated = update_altitude(); // This is called every loop to make sure we don't lose data
}

/*
 * Loop as fast as we can get accelerometer readings.
 * When we detect an upwards acceleration, go into 
 * liftoff state.
 */
void state_launch_rdy()
{
    /* Update accel data only, don't worry about logging */
    accel.getEvent(&accel_event);

    /* Acceleration must pass threshold 7 times to go for launch */
    if (abs(accel_event.acceleration.y) > LAUNCH_DETECT_THRESHOLD) {
      if (!launch_detect_counter) {
        /* First time passing threshold, record time */
        launch_time = millis();
      }
      launch_detect_counter++;
  } else launch_detect_counter = 0;

    /* STATE CHANGE */
    if (launch_detect_counter >= LAUNCH_DETECT_COUNTER_THRESHOLD) {
        loop_time_ms = PWR_ASC_SPEED;
        init_launch_time = launch_time;
        state = LIFTOFF;
    }
}

/*
 * Update and log until we clear the launch rail
 * based on amount of time since launch started.
 */
void state_liftoff()
{
    accel.getEvent(&accel_event);
    log_all();

    /* STATE CHANGE */
    if (millis() - init_launch_time > LAUNCH_RAIL_TIME_MS) {
        /* Extend servos to nominal position */
        set_servos(35);
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
    accel.getEvent(&accel_event);
    log_all();

    if (alt_updated) {
        alt_updated = false;
        int ndx = init_launch_time / loop_time_ms;
        if (ndx > 500) ndx = 500;
        //int ideal_h = table_h[ndx];
        int ideal_h = 0;
    
        int epsilon = curr_altitude - ideal_h;
        set_servos(get_theta(epsilon));
    
        /* STATE CHANGE */
        /*
         * Change only if prev_altitude > curr_altitude for at least 10 times,
         * this is to reduce the chance of a false reading
         */
        if (prev_altitude > curr_altitude) {
            apogee_detect_counter++;
            if (apogee_detect_counter >= APOGEE_DETECT_COUNTER_THRESHOLD) {
                state = DESCENT;
                set_servos(0);
            }
        } else apogee_detect_counter = 0;
    }
}

/*
 * Update and log sensor values only.
 */
void state_descent()
{
    accel.getEvent(&accel_event);
    log_all();
}

/* Subroutines */

void set_servos(int theta)
{
    // 0->160 , 110->50
    if (theta > 90)
        theta = 90;
    else if (theta < 0)
        theta = 0;
    int pos = 160 - theta;
    for (int i = 0; i < 3; i++)
        servos[i].write(pos);
}

/*
 * Log information from time, accelerometer event, and whatever
 * is in the currAltitude variable.
 */
void log_all()
{
    if (file) {
        String data_str = "T: ";
        data_str += millis() - init_launch_time;
        data_str += ", Ac: ";
        //data_str += accel_event.acceleration.x;
        data_str += 10;
        data_str += ", ";
        //data_str += accel_event.acceleration.y;
        data_str += 20;
        data_str += ", ";
        //data_str += accel_event.acceleration.z;
        data_str += 30;
        data_str += ", Al: ";
        data_str += curr_altitude;
        file.println(data_str);
        file.flush();
    }
}

/*
 * Updates previous_altitude and curr_altitude to
 * reflect the latest info from the altimeter. Return true
 * whenever altitude was updated.
 * THIS MUST BE CALLED AS FAST AS POSSIBLE
 */
boolean update_altitude()
{
    if (Serial1.available()) {
        char c = Serial1.read();
        if (c != '\r' && alt_input_index < 15)
            alt_input[alt_input_index++] = c;
        else
        {
            alt_input[alt_input_index] = '\0';
            alt_input_index = 0;
            int alt_ft = atoi(alt_input);
            int alt_m = alt_ft * .304;
            prev_altitude = curr_altitude;
            curr_altitude = alt_m;
            //Serial.println(alt_ft);
            //Serial.println(curr_altitude);
            Serial.println(curr_altitude);
            return true;
        }
    }
    return false;
}

/*
 * Returns a theta value for the servos given a
 * difference between current altitude and ideal
 * altitude.
 */
int get_theta(double eps)
{
    int theta;
    if (abs(eps) <= 0.5)
        theta = 35;
    else if (eps >= 0)
        theta = 90;
    else if (eps < 0)
        theta = 0;
    return theta;
}
