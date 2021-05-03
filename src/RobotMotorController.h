#include <Arduino.h>
#include "CytronMotorDriver.h"
#include "PID_Controller.h"

class RobotMotorController
{

private:
    CytronMD motor1;
    CytronMD motor2;
    PID_Controller PID_pitch;
    PID_Controller PID_yaw;

public:
    RobotMotorController();
    void init(CytronMD &motor1_in, CytronMD &motor2_in);
    void run_balance(float pitch, float pitch_setpoint);
    void run_balance_steer(float pitch, float pitch_setpoint, float yaw, float yaw_setpoint);
    void get_output_tilt();
    void get_output_tilt_heading();
};