#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "LIS3MDL.h"
#include "ukf.h" // to include matrix.h, UKF defines MAX_MATRIX_SIZE
#include "utils.h"

/**
 * Manage the magnetometer 
 * - power management 
 * - initialise sensor with frequency and sensitivity
 * - fetch data in the background in a non-blocking via via loop()
 * - do hard iron calibration
 * - do north calibration on the basis of acceleration data coming from an IMU
 * - convert sensor data to [uT]
 * - turn the coord system to be aligned with the IMUs coord system
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

        // returns the result of the North calibration, e.g. the difference between the magnetic field vs the gravity field. 
        Matrix& getNorthVector() { return north_vector; };

        bool newCalibDataAvailable();
        
    private:

        LIS3MDL device;
        uint32_t dataRequestTime_us = 0;
        bool dataRequested = false;
        double mag_x,mag_y,mag_z = 0;
        bool dataIsAvailable = false;
        bool initialised = false;
        Measurement dataStreamClock;

        enum {
            PROCESSING = 0,
            CALIBRATE_HARD_IRON = 1,
            CALIBRATE_NORTH_VECTOR = 2
        } state = PROCESSING;

        // RLS algorithm
        double  RLS_lambda = 0.999; /* Forgetting factor */
        Matrix RLS_theta{4,1};          /* The variables we want to indentify */
        Matrix RLS_P{4,4};              /* Inverse of correction estimation */
        Matrix RLS_in{4,1};             /* Input data */
        Matrix RLS_out{1,1};            /* Output data */
        Matrix RLS_gain{4,1};           /* RLS gain */
        uint32_t hardIronCalibStart_ms = 0;       // time when the hard iron calibration started

        /* Magnetic vector constant (align with local magnetic vector) */
        double  north_vector_preset[3] = {cos(0), sin(0), 0.000000};
        Matrix north_vector{3, 1, north_vector_preset};

        /* The hard-magnet bias */
        double  hard_iron_base_preset[3] = {-20.0, 2.0, -14.0};
        Matrix hard_iron_base{3, 1, hard_iron_base_preset};

        // true, if new calibration results are available
        bool calib_north_vector_done = false;
        bool calib_hard_iron_done = true;
};
#endif
