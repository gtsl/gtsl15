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
#include "control_matrix.h"

// Set to 1 for ground testing, set to 0 for flight
#define GROUND_TEST 0

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
    UNPWR_ASC,        /* Unpowered flight, active control */
    DESCENT           /* Descent with drogue deployed */
};

/* Boot into LAUNCH_RDY state */
enum states state = LAUNCH_RDY;

/* Hardware connections */
Servo servos[3];
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const int chip_select = 10;
const int led_pin = 17;

/* Constants */
const int LAUNCH_RAIL_TIME_MS = 100;
const int MILLIS = 1000; // Milliseconds in a second, for converting from hertz to millis
const int LAUNCH_RDY_SPEED = MILLIS / 1000; // Loop at 1000 Hz (< 3200 Hz limit)
const int PWR_ASC_SPEED = MILLIS / 20; // Loop at 10 Hz (< 20 Hz limit)
const int LAUNCH_DETECT_THRESHOLD = 18; // m/s^2, 2gs
const int LAUNCH_DETECT_COUNTER_THRESHOLD = 7; // Number of times LAUNCH_DETECT_THRESHOLD must be read
const int MOTOR_BURNOUT_DETECT_COUNTER_THRESHOLD = 10;
const int APOGEE_DETECT_COUNTER_THRESHOLD = 20;
const int THETA_MAX = 80;

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
int motor_burnout_counter = 0;
boolean alt_updated = false;
unsigned long last_log = 0;

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
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);

    /* Initialise ADXL345 accelerometer */
    // if(!accel.begin())
    // {
    //     Serial.println("No ADXL345 detected ... Check your wiring!");
    //     digitalWrite(led_pin, LOW);
    // } else {
    //     Serial.println("ADXL345 initialized.");
    //     accel.setRange(ADXL345_RANGE_16_G);
    // }

    /* Initialize SD Card */
    pinMode(chip_select, OUTPUT); // Necessary even if chip select pin isn't used.

    if (!SD.begin(chip_select)) {
        Serial.println("Card failed, or not present");
        digitalWrite(led_pin, LOW);
    } else {
        Serial.println("SD Card initialized.");
        file = SD.open("data.txt", FILE_WRITE); // File name <= 8 chars.
    }

    /* Initialize servo motors */
    int pins[3] = {21, 22, 23};
    for (int i = 0; i < 3; i++)
        servos[i].attach(pins[i]);

    for (int i = 0; i < 3; i++)
        servos[i].write(0);

        digitalWrite(led_pin, LOW);

    int pos;
    for (;;) {
        for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees
        {                                  // in steps of 1 degree
            for (int i = 0; i < 3; i++)
                servos[i].write(pos);

            delay(15);                       // waits 15ms for the servo to reach the position
        }
        for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees
        {
            for (int i = 0; i < 3; i++)
                servos[i].write(pos);
            delay(15);
        }
    }
    state = LAUNCH_RDY;
    loop_time_ms = 0;
}

void loop()
{
    if (millis() - last_time > loop_time_ms) {
        switch (state) {
        case LAUNCH_RDY:
            state_launch_rdy();
            break;
        case LIFTOFF:
            state_liftoff();
            break;
        case PWR_ASC:
            state_pwr_asc();
            break;
        case UNPWR_ASC:
            state_unpwr_asc();
            break;
        case DESCENT:
            state_descent();
            break;
        }
        last_time = millis();
    }
    #if GROUND_TEST
    // GET ACCEL AND ALT, STORE IN "BUFFER"

    // SEND PIN ANGLE uint8_t OVER SERIAL

    #endif
    if (Serial.available()) {
        digitalWrite(led_pin, LOW);
    }
    Serial.println("hello");
    alt_updated = update_altitude(); // This is called every loop to make sure we don't lose data
}

/*
 * Loop as fast as we can get accelerometer readings.
 * When we detect an upwards acceleration, go into
 * liftoff state.
 */
void state_launch_rdy()
{
    /* Update accel data */
    update_accel();

    /* Log just in case we don't change state or something weird */
    if (millis() - last_log >= 50) {
        log_all();
        last_log = millis();
    }

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
        digitalWrite(led_pin, LOW);
    }
}

/*
 * Update and log until we clear the launch rail
 * based on amount of time since launch started.
 */
void state_liftoff()
{
    update_accel();
    log_all();

    /* STATE CHANGE */
    if (millis() - init_launch_time > LAUNCH_RAIL_TIME_MS) {
        /* Extend servos to nominal position */
        set_servos(35); // Needs to be changes?
        state = PWR_ASC;
    }
}

/*
 * Non-active control state, update flight time
 * and log. Exit when acceleration is downward (positive)
 * to mark motor burnout.
 */
void state_pwr_asc()
{
    update_accel();
    log_all();

    if (alt_updated) {
        alt_updated = false;
        int flight_time = millis() - init_launch_time;

        /*
         * STATE CHANCE CHECK
         * Change only if accel.y for at least 10 times,
         * this is to reduce the chance of a false reading
         */
        if (accel_event.acceleration.y > 0) {
            motor_burnout_counter++;
        } else {
            motor_burnout_counter = 0;
        }
        if (motor_burnout_counter >= MOTOR_BURNOUT_DETECT_COUNTER_THRESHOLD) {
            state = UNPWR_ASC;
        }
    }
}

/*
 * Active control state, update and log, then based
 * on that information, calculate our new servo
 * positions. Exit when a current altitude is
 * less than a previous altitude (reached apogee).
 */
void state_unpwr_asc()
{
    update_accel();
    log_all();

    if (alt_updated) {
        alt_updated = false;
        int flight_time = millis() - init_launch_time;

        /* Get xvec_est = {h, hdot} from kalman filter */
        int xvec_est[2];
        kf(curr_altitude, accel_event.acceleration.y, xvec_est);

        /* Use {h, hdor} to get servo angle */
        int servo_angle = get_theta(xvec_est[0], xvec_est[1]);
        set_servos(servo_angle);

        /*
         * STATE CHANGE CHECK
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
    update_accel();
    log_all();
}

/* Subroutines */

/*
 * Kalman filter estimates altitude and vertical velocity from measured
 * altitude and acceleration in the vertical direction. Function includes
 * error handing for probable failure in either measurement.
 */
void kf(int h_meas, float a_meas, int *xvec_est)
{

}

void set_servos(int theta)
{
    if (theta > THETA_MAX)
        theta = THETA_MAX;
    else if (theta < 0)
        theta = 0;
    int pos = 25 + theta;
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
        data_str += accel_event.acceleration.x;
        data_str += ", ";
        data_str += accel_event.acceleration.y;
        data_str += ", ";
        data_str += accel_event.acceleration.z;
        data_str += ", Al: ";
        data_str += curr_altitude;
        data_str += ", St: ";
        data_str += state;

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
    #if GROUND_TEST
    // Update altitude from serial source
    #else
    // Update altitude from altimeter
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
            return true;
        }
    }
    #endif
    return false;
}

void update_accel()
{
    #if GROUND_TEST
    // Update accelerometer from serial source
    #else
    // Update accelerometer from sensor
    accel.getEvent(&accel_event);
    #endif
}

void ground_test_transmit()
{
    #if GROUND_TEST
    // Send servo_angle to Simulink
    #endif
}

/*
 * Returns a theta value for the servos given an
 * estimated altitude and vertical speed.
 */
int get_theta(int h, int hdot)
{
    int theta = 0;
    if (h > 2999) h = 2999;
    if (h < 0) h = 0;
    if (hdot > 499) hdot = 499;
    if (hdot < 0) hdot = 0;
    theta = control_matrix[h][hdot];
    return theta;
}
