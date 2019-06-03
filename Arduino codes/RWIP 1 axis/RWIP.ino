#include <DualMC33926MotorShield.h>

#include <helper_3dmath.h>
//#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_9Axis_MotionApps41.h>

//230400 baud

//#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20_mod_1.h>

//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotioApps20.h"n
#include "Wire.h"
#include "PinChangeInt.h"

DualMC33926MotorShield md;


////////////////////////////////////////////
//            PIN Assignments             //
////////////////////////////////////////////

#define pin_A_PMW           9
#define pin_A_dir           7
#define pin_A_brake         5
#define pin_A_tach          A2

#define pin_B_PMW           10
#define pin_B_dir           8
#define pin_B_brake         6
#define pin_B_tach          3

#define pin_IMU_interrupt   2
#define pin_IMU_SDA         A4
#define pin_IMU_SCL         A5

////////////////////////////////////////////
//         Variables for Interrupt        //
////////////////////////////////////////////

#define tach_rev_res          211                  // Encoder ticks per revolution

volatile unsigned int   A_tach_rev_tick = 0;      // Encoder ticks
volatile long           A_tach_rev = 0;           // Encoder revolutions
volatile unsigned long  A_time_now = 0;           // Time when one revolution happens [us]
volatile unsigned long  A_time_prev = 0;          // Time of previous revolution [us]
volatile unsigned long  A_time_diff = 0;          // Time of one revolution [us]

volatile unsigned int   B_tach_rev_tick = 0;      // Encoder ticks
volatile long           B_tach_rev = 0;           // Encoder revolutions
volatile unsigned long  B_time_now = 0;           // Time when one revolution occures [us]
volatile unsigned long  B_time_prev = 0;          // Time of previous revolution [us]
volatile unsigned long  B_time_diff = 0;          // Time of one revolution [us]



////////////////////////////////////////////
//    Variables for Theta Calculations    //
////////////////////////////////////////////

float   theta_time = 0;         // Time of Theta calculation
float   cycle_time = 0;         // Time since last Theta calculation

float   theta_A = 0;            // Angular displacement of Pendulum body, theta_A, in Y direction
float   theta_A_prev = 0;       // Stored value of previous theta_A
float   dTheta_A = 0;           // Angular velocity of Pendulum body, dTheta_A, in Y direction
float   dTheta_A_prev = 0;      // Stored value of previous dTheta_A
float   theta_A_zero = -5.22;       // Reference zero for theta_A
float   theta_A_zero_prev = 0;  // Previous reference zero
float   theta_A_error = 0;      // Error value of theta_A

float   theta_B = 0;            // Angular displacement of Pendulum body, theta_B, in X direction
float   theta_B_prev;           // Stored value of previous theta_B
float   dTheta_B = 0;           // Angular velocity of Pendulum body, dTheta_B, in X direction
float   dTheta_B_prev;          // Stored value of previous dTheta_B
float   theta_B_zero = 0;       // Reference zero for theta_B
float   theta_B_zero_prev =0;  // Previous reference zero
float   theta_B_error = 0;      // Error value of theta_B

////////////////////////////////////////////
//          Controller Variables          //
////////////////////////////////////////////

float   Kp1_A = -242;            // K_p for motor A    (for theta_A_error)
float   Kp2_A = -33.58;             // K_d for motor A    (for dTheta_A)
float   Kp3_A = -3;              // K_p,w for motor A  (for A_speed)

//float   Kp1_A = -820;               // K_p for motor A    (for theta_A_error)
//float   Kp2_A = -10;                // K_d for motor A    (for dTheta_A)
//float   Kp3_A = -3;              // K_p,w for motor A  (for A_speed)

float   A_set_speed = 0;        // Controller output for motor A
float   A_set_PWM = 0;          // Setpoint PWM for motor A (from A_set_speed)
float   A_set_PWM_prev = 0;     // Previous PWM setpoint

float   Kp1_B = 200;            // K_p for motor B    (for theta_B_error)
float   Kp2_B = 60;             // K_d for motor B    (for dTheta_B)
float   Kp3_B = 5;              // K_p,w for motor B  (for B_speed)

//float   Kp1_B = 425;               // K_p for motor B    (for theta_B_error)
//float   Kp2_B = 13;                // K_d for motor B    (for dTheta_B)
//float   Kp3_B = 2;              // K_p,w for motor B  (for B_speed)

float   B_set_speed = 0;        // Controller output for motor A
float   B_set_PWM = 0;          // Setpoint PWM for motor A (from A_set_speed)
float   B_set_PWM_prev = 0;     // Previous PWM setpoint

////////////////////////////////////////////
//         Motor Speed Variables          //
////////////////////////////////////////////

float   A_speed = 0;            // Calculated speed for motor A
float   A_speed_prev = 0;       // Previous speed
int     A_dir_set = 0;          // Direction set by motor shield, -1 or 1
int     A_dir = 0;              // Actual direction of motor, -1 or 1

float   B_speed = 0;            // Calculated speed for motor B
float   B_speed_prev = 0;       // Previous speed
int     B_dir_set = 0;          // Direction set by motor shield, -1 or 1
int     B_dir = 0;              // Actual direction of motor, -1 or 1


String package;                 // Serial print
int x = 0;

////////////////////////////////////////////
//             SETUP PROCEDURE            //
////////////////////////////////////////////

void setup() {
  // Serial
  Serial.begin(230400);
  md.init();

  // Pin Modes
  pinMode(pin_A_PMW, OUTPUT);
  pinMode(pin_A_dir, OUTPUT);
  pinMode(pin_A_brake, OUTPUT);
  pinMode(pin_A_tach, INPUT);
  
  pinMode(pin_B_PMW, OUTPUT);
  pinMode(pin_B_dir, OUTPUT);
  pinMode(pin_B_brake, OUTPUT);
  pinMode(pin_B_tach, INPUT);

  pinMode(pin_IMU_interrupt, INPUT);
  pinMode(pin_IMU_SDA, INPUT);
  pinMode(pin_IMU_SCL, INPUT);

  // Interrupts
  PCintPort::attachInterrupt(pin_A_tach, rpm_A_tach, RISING);
  PCintPort::attachInterrupt(pin_B_tach, rpm_B_tach, RISING);
  //attachInterrupt(pin_IMU_interrupt, dmpDataReady, RISING);

  IMU_setup();
}
