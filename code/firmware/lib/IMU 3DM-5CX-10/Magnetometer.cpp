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

    // 50us after the data request, the data should be available
    if (dataRequested && (micros() - dataRequestTime_us > 50)) {
        dataRequested = false;
        fetchData();
        dataIsAvailable = true;
    }
}
