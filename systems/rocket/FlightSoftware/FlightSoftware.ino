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

// KALMAN FILTER VARIABLES
// double ** P_Kalman;// = new double*[2];
// double ** R_Kalman;// = new double*[2];
// double ** Q_Kalman;// = new double*[2];
double P_Kalman[2][2];// = new double*[2];
double R_Kalman[2][2];// = new double*[2];
double Q_Kalman[2][2];// = new double*[2];
double ** A;// = new double*[2];
double ** Atr;// = new double*[2];
double ** B;// = new double*[2];
double ** x_minus;// = new double*[2];
double ** x_est;// = new double*[2];
double ** P_minus;// = new double*[2];
double ** K;// = new double*[2];
double ** temp1;// = new double*[2];
double ** temp2;// = new double*[2];
double ** temp3;// = new double*[2];
double ** temp4;// = new double*[2];
double ** temp5;// = new double*[2];
double ** temp6;// = new double*[2];
double ** temp7;// = new double*[2];
double ** temp8;// = new double*[2];
double ** inv_temp;// = new double*[2];

for (int i=0;i<2;i++) {
    P_Kalman[i] = new double[2];
	R_Kalman[i] = new double[2];
	Q_Kalman[i] = new double[2];
	A[i] = new double[2];
	Atr[i] = new double[2];
	B[i] = new double[1];
	x_minus[i] = new double[1];
	x_est[i] = new double[1];
	P_minus[i] = new double[2];
	K[i] = new double[2];
	temp1[i] = new double[2];
	temp2[i] = new double[2];
	temp3[i] = new double[2];
	temp4[i] = new double[2];
	temp5[i] = new double[1];
	temp6[i] = new double[1];
	temp7[i] = new double[1];
	temp8[i] = new double[1];
	inv_temp[i] = new double*[2];
}

R_Kalman[0][0] = 80.0*80.0;
R_Kalman[1][1] = 140.0*140.0;
R_Kalman[0][1] = 0.0;
R_Kalman[1][0] = 0.0;
Q_Kalman[0][0] = 0.0;
Q_Kalman[1][1] = 30.0*30.0;
Q_Kalman[0][1] = 0.0;
Q_Kalman[1][0] = 0.0;
P_Kalman[0][0] = 1.0*1.0;
P_Kalman[1][1] = 1.0*1.0;
P_Kalman[0][1] = 0.0;
P_Kalman[1][0] = 0.0;
// END KALMAN FILTER VARIABLES


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
//KALMAN FILTER FUNCTION
void kf(int h_meas, int h_meas_prev, float a_meas, double **xvec_est,
    double timestep)
{
	// Recalculate A & B matrices
	A[0][0] = 1.0;
	A[1][1] = 1.0;
	A[0][1] = timestep;
	A[1][0] = 0.0;
	B[0][0] = 0.0;
	B[1][0] = timestep;

	double v_meas = (h_meas - h_meas_prev)/timestep;

	/* KALMAN ALGORITHM:
	x_minus = A*x_est(:,i) + B*u(i+1);
    P_minus = A*P_Kalman*A' + Q;
    K = P_minus*inv(P_minus + R);
    x_est(:,i+1) = x_minus + K*([h_act(i+1);v_act(i+1)] - x_minus);
    P_Kalman = (eye(2) - K)*P_minus;*/

	matMultiplication(2, 2, A, 2, 1, xvec_est, temp5);
	matScaleMultiplication(2, 1, B, a_meas, temp6);
	matSum(2, 1, temp5, temp6, x_minus);

	matMultiplication(2, 2, A, 2, 2, P_Kalman, temp1);
	transpose(2, 2, A, Atr);
	matMultiplication(2, 2, temp1, 2, 2, Atr, temp2);
	matSum(2, 2, temp2, Q_Kalman, P_minus);

	matSum(2, 2, P_minus, R_Kalman, temp3);
	mat2Inverse(temp3, inv_temp);
	matMultiplication(2, 2, P_minus, 2, 2, inv_temp, K);

	temp7[0][0] = h_meas;
	temp7[1][0] = v_meas;
	matScaleMultiplication(2, 1, x_minus, -1, temp8);
	matSum(2, 1, temp7, temp8, temp5);
	matMultiplication(2, 2, K, 2, 1, temp5, temp6);
	xvec_est = matSum(2, 1, temp6, x_minus, xvec_est);

	temp3[0][0] = 1.0;
	temp3[0][1] = 0.0;
	temp3[1][0] = 0.0;
	temp3[1][1] = 1.0;
	matScaleMultiplication(2, 2, K, -1, temp4);
	matSum(2, 2, temp3, temp4, temp1);
	matMultiplication(2, 2, temp1, 2, 2, P_minus, P_Kalman);

}
//END KALMAN FILTER FUNCTION


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

// KALMAN FILTER LINEAR ALGEBRA
void matScaleMultiplication(int row, int column, double **mat, double scale,
    double** result)
{
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            result[i][j] = mat[i][j] * scale;
        }
    }

}

void matMultiplication(int row1, int column1, double **mat1, int row2,
    int column2, double **mat2, double** result) {

    if (column1 != row2) {
        cout << "Error: matrix dimension mismatch! \n";
    } else {
        double sum;
        for (int i = 0; i < row1; i++) {
            for (int j = 0; j < column2; j++) {
                sum = 0;
                for (int k = 0; k < column1; k++) {
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }
    }
}

void matSum(int row,  int column, double **mat1, double **mat2,
    double** result)
{

    for(int i=0; i<row; i++) {
        for(int j=0; j<column; j++) {
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void transpose(int row_in, int column_in, double **mat_in, double **mat_out)
{
    double temp;
    for (int j = 0; j<column_in; j++) {
        for (int i = 0; i<row_in; i++) {
            temp = mat_in[i][j];
            mat_out[j][i] = temp;
        }
    }
}

void mat2Inverse(double **a, double **ainv)
{

	a_ = a[0][0];
	b_ = a[0][1];
	c_ = a[1][0];
	d_ = a[1][1];

	ainv[0][0] = d_/(a_*d_* - b_*c_*);
	ainv[0][1] = -b_/(a_*d_* - b_*c_*);
	ainv[1][0] = -c_/(a_*d_* - b_*c_*);
	ainv[1][1] = a_/(a_*d_* - b_*c_*);

}
// END KALMAN FILTER LINEAR ALGEBRA
