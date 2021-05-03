#include <Arduino.h>
#include "CytronMotorDriver.h"

class PID_Controller
{
private:
    float KP;
    float KI;
    float KD;
    float previous_error;

public:
    PID_Controller();
    bool set_PID_values(float P_new, float I_new, float D_new);
    float get_output_signal(float error);
    void get_PID_tilt();
    void get_PID_tilt_heading();
};