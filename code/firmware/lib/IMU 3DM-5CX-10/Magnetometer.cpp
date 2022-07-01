#include "Magnetometer.h"

static bool newDataIsAvailable = false;
void newDataAvailable() {
  newDataIsAvailable = true;
}

void Magnetometer::setup(dataRate_t freq, range_t dataRange) {
    device.setup(freq, dataRange);

    dataRequestTime_us = 0;
    attachInterrupt(digitalPinToInterrupt(3), newDataAvailable, RISING);
};

void Magnetometer::requestData() {
    device.requestData();
}

void Magnetometer::fetchData() {
    device.read(mag_x,mag_y,mag_z);
}

bool Magnetometer::isDataAvailable() {
    bool saveFlag = dataIsAvailable;
    dataIsAvailable = false;
    if (saveFlag) 
        return true;
    return false;
}
void Magnetometer::read(double &x, double &y, double &z) {
    x = mag_x;
    y = mag_y;
    z = mag_z;
}

void Magnetometer::loop() {
    // if interrupt fired 
    if (newDataIsAvailable) {
        requestData();
        dataRequestTime_us = micros();
        dataRequested = true;
        newDataIsAvailable = false;
    }

    // Wait until we have 6 bytes in the hardwarebuffer, then read the data
    if (dataRequested && (device.isDataAvailable())) {
        dataRequested = false;
        fetchData();
        dataIsAvailable = true;
    } else {
        // timeout. After 5ms of waiting, something went wrong, ignore the last request
        // and dont wait for a reply anymore
        if (micros() - dataRequestTime_us > 5000) {
            dataRequested = false;
        }
    }
}
