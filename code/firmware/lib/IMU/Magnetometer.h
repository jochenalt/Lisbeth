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
        void loop();

        // returns true if read can be called.
        // is reset afterwards, i.e. it returns true only once, the next call will return false again.
        bool isDataAvailable() ;

        // call if isDataAvailable indicates new data.
        // fetches data in [gauss]
        void read(double &x, double &y, double &z);

        bool isInitialised() { return initialised; };
        Measurement& getMeasuremt() { return dataStreamClock;};

        void startHardIronCalibration();
        void startNorthCalibration();
       
        void calibrateLoop(double accX, double accY, double accZ, double max_x, double mag_y, double mag_z);

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
        uint32_t RLS_u32iterData = 0;   /* To track how much data we take */

        /* Magnetic vector constant (align with local magnetic vector) */
        double  IMU_MAG_B0_data[3] = {cos(0), sin(0), 0.000000};
        Matrix IMU_MAG_B0{3, 1, IMU_MAG_B0_data};

        /* The hard-magnet bias */
        double  HARD_IRON_BIAS_data[3] = {-20.0, 2.0, -14.0};
        Matrix HARD_IRON_BIAS{3, 1, HARD_IRON_BIAS_data};
};
#endif
