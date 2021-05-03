#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "mpu9250.h"

using namespace std;

class IMU_Controller
{
private:
    // Calibration data
    // MPU object
    MPU9250 *mpu;
    float pitch_acc_measured, pitch_acc_filtered_new, pitch;
    float pitch_acc_filtered_old = 0;
    float pitch_gyro = 0;
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    unsigned long now;
    unsigned long last_measurement;

public:
    IMU_Controller();
    bool init(MPU9250 *mpu_in, unsigned long &central_now);
    bool connect_SD();
    String calibrate();
    bool update_vectors();
    float get_pitch();
    float get_yaw();
    String get_values_string();
    bool save_calibration_data();
};