#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "LIS3MDL.h"
#include "MicrostrainComm.h"
#include "ukf.h"
/**
 * Manage the magnetometer 
 * - power management with correct timing
 * - initialise sensor with passed frequency and sensitivity
 * - fetch data in the background in a non-blocking via via loop()
 * - convert sensor data to [gauss]
 */ 
class Magnetometer {
    public:
        Magnetometer() {};

        // call setup with the intended of the sensor
        // it  has only predefined frequencies 
        bool setup(dataRate_t freq, range_t dataRange);

        // this handles the datastream from the sensor, call as often as possible, requires only minimal CPU
        // returns true if new value is available
        bool loop();

        // returns true if read can be called.
        // is reset afterwards, i.e. it returns true only once, the next call will return false again.
        bool isDataAvailable() ;

        // call if isDataAvailable indicates new data.
        // fetches data in [gauss]
        void read(double &x, double &y, double &z);

        // true if setup was successful. Stat calling loop() now.
        bool isInitialised() { return initialised; };

        // returns measurement about the frequency
        Measurement& getMeasuremt() { return dataStreamClock;};

        void startHardIronCalibration();
        void startNorthCalibration();

        // loop this during the calibration process, and merge with acceleration data from IMU
        // Acceleration data is used to compute the angle between earth magnetic field and gravity
        void calibrateLoop(double accX, double accY, double accZ, double max_x, double mag_y, double mag_z);

        // returns the result of the hard iron calibration
        Matrix& getHardIronBase() { return hard_iron_base; };

        // returns the result of the North calibration
        Matrix& getIMUMagB0() { return imu_mag_b0; };


    private:
        void requestData();
        void fetchData();

        LIS3MDL device;
        uint32_t dataRequestTime_us = 0;
        bool dataRequested = false;
        double mag_x,mag_y,mag_z = 0;
        bool dataIsAvailable = false;
        bool initialised = false;
        Measurement dataStreamClock;

        enum {
            STATE_UKF_RUNNING = 0,
            STATE_MAGNETO_BIAS_IDENTIFICATION = 1,
            STATE_NORTH_VECTOR_IDENTIFICATION = 2
        } state = STATE_UKF_RUNNING;

        // RLS algorithm
        double  RLS_lambda = 0.999; /* Forgetting factor */
        Matrix RLS_theta{4,1};          /* The variables we want to indentify */
        Matrix RLS_P{4,4};              /* Inverse of correction estimation */
        Matrix RLS_in{4,1};             /* Input data */
        Matrix RLS_out{1,1};            /* Output data */
        Matrix RLS_gain{4,1};           /* RLS gain */
        uint32_t calibrationStart_ms = 0;

        /* Magnetic vector constant (align with local magnetic vector) */
        double  imu_mag_b0_preset[3] = {cos(0), sin(0), 0.000000};
        Matrix imu_mag_b0{3, 1, imu_mag_b0_preset};

        /* The hard-magnet bias */
        double  hard_iron_base_preset[3] = {-20.0, 2.0, -14.0};
        Matrix hard_iron_base{3, 1, hard_iron_base_preset};
};
#endif
