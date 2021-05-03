#include "RobotMotorController.h"
void RobotMotorController::RobotMotorController() {
}

void RobotMotorController::init(CytronMD &motor1_in, CytronMD &motor2_in) {
    motor1 = motor1_in;
    motor2 = motor2_in;
    PID_pitch = PID_pitch;
    PID_yaw = PID_yaw;
}

/*  This function controls both motors with a single PID function
     *  and keeps the robot upright.
    */
void RobotMotorController::run_balance(float pitch, float pitch_setpoint) {
    // Calculate pitch error
    float current_pitch_error   = (pitch - pitch_setpoint);

    // Calculate required motor speed from error
    float speed = PID_pitch.get_output_signal(current_pitch_error);

    // Write speeds to motors
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
}

/*  This function controls the motors separately, each with its own PID function.
     *  It keeps the robot upright and allows for control of the heading.
    */
void RobotMotorController::run_balance_steer(float pitch, float pitch_setpoint, float yaw, float yaw_setpoint) {
    float current_pitch_error   = (pitch - pitch_setpoint);
    float current_yaw_error     = (yaw - yaw_setpoint);

    // Calculate motor speeds and steer factors
    float motor_speed = PID_pitch.get_output_signal(current_pitch_error);
    float steer_factor = PID_yaw.get_output_signal(current_yaw_error);

    // Calculate individual motor speeds from yaw PID
    float motor_1_speed = motor_speed * (1 / steer_factor);
    float motor_2_speed = motor_speed - motor_1_speed;

    // Power motors
    motor1.setSpeed(motor_1_speed);
    motor2.setSpeed(motor_2_speed);
}