#include "Magnetometer.h"
#include "TimePassedBy.h"
#include "MicrostrainComm.h"
static volatile bool newDataIsAvailable = false;
void newDataAvailableInterrupt() {
  newDataIsAvailable = true;

}

bool Magnetometer::setup(dataRate_t freq, range_t dataRange) {

    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), newDataAvailableInterrupt, RISING);

    bool ok = device.setup(freq, dataRange);

    if (!ok)
        return false;
    dataRequestTime_us = 0;
    initialised = true;
    Serial.println("MAG: setup");
    newDataIsAvailable = true; // first the first read
    return true;
};

void Magnetometer::requestData() {
    device.requestData();
}

void Magnetometer::fetchData() {
    device.readResponse(mag_x,mag_y,mag_z);
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
    if (initialised) {     
        // if interrupt fired         
        if (newDataIsAvailable) {
            requestData();
            dataStreamClock.tick();
            dataRequestTime_us = micros();
            dataRequested = true;
            newDataIsAvailable = false;
        }

        // Wait until we have 6 bytes in the hardwarebuffer, then read the data
        if (dataRequested) {
            if (device.isDataAvailable()) {
                dataRequested = false;
                fetchData();

                 static TimePassedBy printTimer (1000);
                if (printTimer.isDue()) {
   
                Serial.print("MAG");

                Serial.print("x=");
                Serial.print(mag_x);
                Serial.print("y=");
                Serial.print(mag_y);
                Serial.print("z=");
                Serial.println(mag_z);
                Serial.println(dataStreamClock.getAvrFreq());
                }

                


                dataIsAvailable = true;
            } else {
                // Timeout. After 5ms of waiting, something went wrong, ignore the last request
                // and dont wait for a reply anymore
                if (micros() - dataRequestTime_us > 5000) {
                    dataRequested = false;
                }
            }
        }
    }
}
