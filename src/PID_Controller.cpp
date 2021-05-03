#include <Arduino.h>
#include "PID_Controller.h"

/*  This function sets the values of the P, I and D value for use in the run_tilt() function.
     *  Returns false if one of the input values is not correct.
    */
PID_Controller::PID_Controller() {
}

bool PID_Controller::set_PID_values(float P_new, float I_new, float D_new)
{
    bool success = true;

    if (P_new > 0)
        P = P_new;
    else
        success = false;
    if (I_new > 0)
        I = I_new;
    else
        success = false;
    if (D_new > 0)
        D = D_new;
    else
        success = false;

    return success;
}

float PID_Controller::get_output_signal(float error)
{
    // pitch_integral term
    float pitch_integral  = (pitch_integral + error);

    // pitch_derivative term
    float pitch_derivative  = (error - previous_error);

    float P = (KP * error);
    float I = (KI * pitch_integral);
    float D = (KD * pitch_derivative);

    float output = P + I + D;

    return output;
}