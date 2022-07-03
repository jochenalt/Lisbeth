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

    // RLS
    RLS_theta.setZero();
    RLS_P.setIdentity();
    RLS_P = RLS_P * 1000;

    state = STATE_UKF_RUNNING;

    startHardIronCalibration();

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

bool Magnetometer::loop() {
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
                return true;
            } else {
                // Timeout. After 5ms of waiting, something went wrong, ignore the last request
                // and dont wait for a reply anymore
                if (micros() - dataRequestTime_us > 5000) {
                    dataRequested = false;
                }
            }
        }
    }
    return false;
}


void Magnetometer::startHardIronCalibration() {
    Serial.println("start moving the sensor in the shape of an 8");
    state = STATE_MAGNETO_BIAS_IDENTIFICATION;
    calibrationStart_ms = millis();
}

void Magnetometer::startNorthCalibration() {
    state = STATE_NORTH_VECTOR_IDENTIFICATION;
}

void Magnetometer::calibrateLoop(double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z) {

    if (state == STATE_MAGNETO_BIAS_IDENTIFICATION) {
        RLS_in[0][0] =  mag_x;
        RLS_in[1][0] =  mag_y;
        RLS_in[2][0] =  mag_z;
        RLS_in[3][0] =  1;
        RLS_out[0][0] = (mag_x*mag_x) + (mag_y*mag_y) + (mag_z*mag_z);
                
        float err = (RLS_out - (RLS_in.Transpose() * RLS_theta))[0][0];
        RLS_gain  = RLS_P*RLS_in / (RLS_lambda + RLS_in.Transpose()*RLS_P*RLS_in)[0][0];
        RLS_P     = (RLS_P - RLS_gain*RLS_in.Transpose()*RLS_P)/RLS_lambda;
        RLS_theta = RLS_theta + err*RLS_gain;
                
        Matrix P_check{RLS_P.getDiagonalEntries()};
        double error = (P_check.Transpose()*P_check)[0][0];

        if (error < 1e-4) {
            /* The data collection is finished, go back to state UKF running */
            // state = STATE_NORTH_VECTOR_IDENTIFICATION;
            state = STATE_UKF_RUNNING;
                    
            /* Reconstruct the matrix compensation solution */
            hard_iron_base[0][0] = RLS_theta[0][0] / 2.0;
            hard_iron_base[1][0] = RLS_theta[1][0] / 2.0;
            hard_iron_base[2][0] = RLS_theta[2][0] / 2.0;
    
            Serial.println("Calibration finished, the hard-iron bias identified:");
            println("%f %f %f\r\n", hard_iron_base[0][0], hard_iron_base[1][0], hard_iron_base[2][0]);
        }

        static TimePassedBy printTimer (500);
        if (printTimer.isDue()) {
                println( "Hard iron calibration %.3f %.3f %.3f (P = %f > 0.001!)", RLS_theta[0][0] / 2.0, RLS_theta[1][0] / 2.0, RLS_theta[2][0] / 2.0, error);
        }
        if (millis() - calibrationStart_ms > 10000) {
            /* We take the data too long but the error still large, terminate without updating the hard-iron bias */
            state = STATE_UKF_RUNNING;
                    
            Serial.println("Calibration timeout, the hard-iron bias won't be updated\r\n");
        }           
    }
    else if (state == STATE_NORTH_VECTOR_IDENTIFICATION) {
        float Ax = acc_x;
        float Ay = acc_y;
        float Az = acc_z;
        float Bx = mag_x - hard_iron_base[0][0];
        float By = mag_y - hard_iron_base[1][0];
        float Bz = mag_z - hard_iron_base[2][0];
                
        /* Normalizing the acceleration vector & projecting the gravitational vector (gravity is negative acceleration) */
        double _normG = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));
        Ax = Ax / _normG;
        Ay = Ay / _normG;
        Az = Az / _normG;
                
        /* Normalizing the magnetic vector */
        _normG = sqrt((Bx * Bx) + (By * By) + (Bz * Bz));
        Bx = Bx / _normG;
        By = By / _normG;
        Bz = Bz / _normG;
        
        /* Projecting the magnetic vector into plane orthogonal to the gravitational vector */
        float pitch = asin(-Ax);
        float roll = asin(Ay/cos(pitch));
        float m_tilt_x =  Bx*cos(pitch)             + By*sin(roll)*sin(pitch)   + Bz*cos(roll)*sin(pitch);
        float m_tilt_y =                            + By*cos(roll)              - Bz*sin(roll);
        // ignore yaw (otherwise would have been )
        /* float m_tilt_z = -Bx*sin(pitch)             + By*sin(roll)*cos(pitch)   + Bz*cos(roll)*cos(pitch); */
                
        float mag_dec = atan2(m_tilt_y, m_tilt_x);
        imu_mag_b0[0][0] = cos(mag_dec);
        imu_mag_b0[1][0] = sin(mag_dec);
        imu_mag_b0[2][0] = 0;
                
        Serial.println("North identification finished, the north vector identified:");
        println("%.3f %.3f %.3f", imu_mag_b0[0][0], imu_mag_b0[1][0], imu_mag_b0[2][0]);
                
        state  = STATE_UKF_RUNNING;
    }
}