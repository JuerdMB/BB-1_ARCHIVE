#include "IMU_Controller.h"
// TODO     add yaw to get_pitch_yaw, fix dependencies problem

using namespace std;

IMU_Controller::IMU_Controller()
{
}

bool IMU_Controller::init(MPU9250 *mpu_in, unsigned long &central_now){
    mpu = mpu_in;
    now = central_now;
}

bool IMU_Controller::connect_SD()
{
    // Make connection to an SD card object that is externally provided.
}

String IMU_Controller::calibrate()
{
    mpu->verbose(true);
    delay(5000);
    mpu->calibrateAccelGyro();
    delay(5000);
    mpu->calibrateMag();
    mpu->verbose(false);
    return get_values_string();
}

bool IMU_Controller::save_calibration_data()
{
}

/*  This method returns filtered values for the pitch and yaw as obtained by the IMU
    *   
    */
float IMU_Controller::get_pitch()
{
    now = millis();

    // Get new values from IMU
    update_vectors();

    // Calculate pitch from accelerometers x and z
    pitch_acc_measured = atan2(acc_x, acc_z) / 2 / M_PI;

    // Calculate pitch from integration of angular velocity along y
    float dt = ((now - last_measurement) / 1000.);

    // Complementary filter
    pitch = (pitch + pitch_gyro * dt) * .95 + pitch_acc_measured * .05;

    last_measurement = now;

    // Save current filtered pitch from accelerometer for calculation of next one
    pitch_acc_filtered_old = pitch_acc_filtered_new;

    return pitch;
}

bool IMU_Controller::update_vectors()
{
    mpu->update();
    acc_x = mpu->getAccX();
    acc_y = mpu->getAccY();
    acc_z = mpu->getAccZ();
    gyro_x = mpu->getGyroX();
    gyro_y = mpu->getGyroY();
    gyro_z = mpu->getGyroZ();
}

String IMU_Controller::get_values_string()
{
    String values_string = "";
    values_string += "accel bias [g]:";
values_string += (String) (mpu->getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
values_string += (String) (mpu->getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
values_string += (String) (mpu->getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);

values_string += "\ngyro bias [deg/s]: ";
values_string += (String) (mpu->getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
values_string += (String) (mpu->getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
values_string += (String) (mpu->getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);

values_string += "\nmag bias [mG]: ";
values_string += (String)(mpu->getMagBiasX());
values_string += (String)(mpu->getMagBiasY());
values_string += (String)(mpu->getMagBiasZ());

values_string += "\nmag scale []: ";
values_string += (String)(mpu->getMagScaleX());
values_string += (String)(mpu->getMagScaleY());
values_string += (String)(mpu->getMagScaleZ());

return values_string;
}