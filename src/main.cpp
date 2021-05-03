#include <Arduino.h>

/********************************************************************************
   BB-1 Self balancing robot by Juerd Mispelblom Beyer
   V1

   CONNECTIONS:
   Arduino D3  - Motor Driver PWM 1 Input
   Arduino D4  - Motor Driver DIR 1 Input
   Arduino D9  - Motor Driver PWM 2 Input
   Arduino D10 - Motor Driver DIR 2 Input
   Arduino GND - Motor Driver GND

 ********************************************************************************/
using namespace std;

#include "RobotMotorController.h"
#include "IMU_Controller.h"
#include "CytronMotorDriver.h"
#include "MPU9250.h"

// IMU object
MPU9250 mpu;
// Motor objects
CytronMD motor1(PWM_DIR, 3, 4);  // Left motor    PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 9, 10); // Right motor   PWM 2 = Pin 9, DIR 2 = Pin 10.

unsigned long now = 0;

// Objects that enable the program to run
IMU_Controller IMU;
RobotMotorController Controller;

// Array for storing filtered data from IMU
float pitch, pitch_setpoint;
float yaw, yaw_setpoint;

// States the program can be in
enum program_state {
    IDLE,
    CAL,
    BALANCE,
    STEER
};

// This is the default state the program will start in upon boot
program_state current_state = CAL;

/*  This method changes the program state
    A state machine enables different behaviour to happen once per target state
*/
void switch_program_state(program_state new_state) {
    switch (new_state) {
        case IDLE:
            current_state = IDLE;
            break;

        case CAL:
            current_state = CAL;
            break;

        case BALANCE:
            current_state = BALANCE;
            break;

        case STEER:
            current_state = STEER;
            break;

        default:
            // new_state incorrect, do nothing
            break;
    }
}

void setup() {
    // Start Serial with high baud rate
    Serial.begin(115200);

    IMU.init(&mpu, now);
    Controller.init(motor1, motor2);
}

void loop() {
    // Get array[2] of new filtered pitch & yaw from IMU
    pitch = IMU.get_pitch();
    yaw = IMU.get_yaw();

    switch (current_state) {
        case IDLE:
            break;

        case CAL:
            Serial.println(IMU.calibrate());
            switch_program_state(BALANCE);
            break;

        case BALANCE:
            // Update PID values for only balancing tilt from data measured from IMU
            Controller.run_balance(pitch, pitch_setpoint);
            break;

        case STEER:
            // Update PID values for balancing and steering from data measured from IMU
            Controller.run_balance_steer(pitch, pitch_setpoint, yaw, yaw_setpoint);
            break;

        default:
            break;
    }
}