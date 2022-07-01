#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "LIS3MDL.h"

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
        void setup(dataRate_t freq, range_t dataRange);

        // this handles the datastream from the sensor, call as often as possible, requires only minimal CPU
        void loop();

        // returns true if read can be called.
        // is reset afterwards, i.e. it returns true only once, the next call will return false again.
        bool isDataAvailable() ;

        // call if isDataAvailable indicates new data.
        // fetches data in [gauss]
        void read(double &x, double &y, double &z);

    private:
        void requestData();
        void fetchData();

        LIS3MDL device;
        uint32_t dataRequestTime_us = 0;
        bool dataRequested = false;
        double mag_x,mag_y,mag_z = 0;
        bool dataIsAvailable = false;
};
#endif
