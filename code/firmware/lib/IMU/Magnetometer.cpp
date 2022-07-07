#include "Magnetometer.h"
#include "TimePassedBy.h"
#include "MicrostrainComm.h"
static volatile bool newDataIsAvailable = false;
void newDataAvailableInterrupt() {
  newDataIsAvailable = true;

}

bool Magnetometer::setup(dataRate_t freq, range_t dataRange) {

    Serial.println("MAG: setup");
    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), newDataAvailableInterrupt, RISING);

    bool ok = device.setup(freq, dataRange);

    dataRequestTime_us = 0;
    initialised = true;
    newDataIsAvailable = true; // first the first read

    // RLS
    RLS_theta.setZero();
    RLS_P.setIdentity();
    RLS_P = RLS_P * 1000;

    state = PROCESSING;

    // if sensor could not be found, return false
    if (!ok)
        return false;

    return true;
};


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
            device.requestData();
            dataStreamClock.tick();
            dataRequestTime_us = micros();
            dataRequested = true;
            newDataIsAvailable = false;
        }

        // Wait until we have 6 bytes in the hardwarebuffer, then read the data
        if (dataRequested) {
            if (device.isDataAvailable()) {
                dataRequested = false;
                device.readResponse(mag_x,mag_y,mag_z);

                // read raw data from sensor in [uT] and apply the hard iron compensation
                double tmp_x = mag_x - hard_iron_base[0][0];
                double tmp_y = mag_y - hard_iron_base[1][0];
                double tmp_z = mag_z - hard_iron_base[2][0];

                // Align the coordinate system of the magnetometer with the one from the IMU
                mag_y = - tmp_x;
                mag_x = tmp_y;
                mag_z = tmp_z;
    
                
                /*
                 static TimePassedBy printTimer (1000);
                if (printTimer.isDue()) {   
                    Serial.print("MAG");
                    Serial.print("x=");
                    Serial.print(mag_x);
                    Serial.print("y=");
                    Serial.print(mag_y);
                    Serial.print("z=");
                    Serial.print(mag_z);
                    Serial.print(" f=");
                    Serial.println(dataStreamClock.getAvrFreq());
                }
                */
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
    state = CALIBRATE_HARD_IRON;
    hardIronCalibStart_ms = millis();
    calib_hard_iron_done = false;
}

void Magnetometer::startNorthCalibration() {
    state = CALIBRATE_NORTH_VECTOR;
    calib_north_vector_done = false;

}

void Magnetometer::calibrateLoop(double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z) {

    if (state == CALIBRATE_HARD_IRON) {
        // remove the existing hard iron correction to get the raw sensor data
        mag_x += hard_iron_base[0][0];
        mag_y += hard_iron_base[1][0];
        mag_z += hard_iron_base[2][0];

        // use recursive least squares filter to find the best hard iron compensation
        RLS_in[0][0] =  mag_x;
        RLS_in[1][0] =  mag_y;
        RLS_in[2][0] =  mag_z;
        RLS_in[3][0] =  1;
        RLS_out[0][0] = (mag_x*mag_x) + (mag_y*mag_y) + (mag_z*mag_z);
                
        float err = (RLS_out - (RLS_in.Transpose() * RLS_theta))[0][0];
        RLS_gain  =  RLS_P*RLS_in / (RLS_lambda + RLS_in.Transpose()*RLS_P*RLS_in)[0][0];
        RLS_P     = (RLS_P - RLS_gain*RLS_in.Transpose()*RLS_P)/RLS_lambda;
        RLS_theta = RLS_theta + err*RLS_gain;
                
        Matrix P_check{RLS_P.getDiagonalEntries()};
        double error = (P_check.Transpose()*P_check)[0][0];

        const float max_error = 1e-3;
        if (error < max_error) {
            /* The data collection is finished, go back to state UKF running */
            state = PROCESSING;
                    
            /* Reconstruct the matrix compensation solution */
            hard_iron_base[0][0] = RLS_theta[0][0] / 2.0;
            hard_iron_base[1][0] = RLS_theta[1][0] / 2.0;
            hard_iron_base[2][0] = RLS_theta[2][0] / 2.0;
    
            Serial.println("Calibration finished, the hard-iron bias identified:");
            println("%f %f %f\r\n", hard_iron_base[0][0], hard_iron_base[1][0], hard_iron_base[2][0]);
            calib_hard_iron_done = true;
        }

        static TimePassedBy printTimer (500);
        if (printTimer.isDue()) {
                println( "Hard iron calibration %.3f %.3f %.3f (P = %f > %f)", RLS_theta[0][0] / 2.0, RLS_theta[1][0] / 2.0, RLS_theta[2][0] / 2.0, error, max_error);
        }
        if (millis() - hardIronCalibStart_ms > 10000) {
            /* We take the data too long but the error still large, terminate without updating the hard-iron bias */
            state = PROCESSING;
                    
            Serial.println("Calibration timeout, the hard-iron bias won't be updated\r\n");
        }           
    }
    if (state == CALIBRATE_NORTH_VECTOR) {
        float Ax = acc_x;
        float Ay = acc_y;
        float Az = acc_z;
        float Bx = mag_x;
        float By = mag_y;
        float Bz = mag_z;
                
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
        double pitch = asin(-Ax);
        double roll = asin(Ay/cos(pitch));
        double m_tilt_x =  Bx*cos(pitch)             + By*sin(roll)*sin(pitch)   + Bz*cos(roll)*sin(pitch);
        double m_tilt_y =                            + By*cos(roll)              - Bz*sin(roll);
        double m_tilt_z = -Bx*sin(pitch)             + By*sin(roll)*cos(pitch)   + Bz*cos(roll)*cos(pitch); 

        // ignore yaw
        double mag_dec = atan2(m_tilt_y, m_tilt_x);
        north_vector[0][0] = cos(mag_dec);
        north_vector[1][0] = sin(mag_dec);
        north_vector[2][0] = m_tilt_z*0;
                
        println("Calibration finished: North vector =(%.3f,%.3f,%.3f)", north_vector[0][0], north_vector[1][0], north_vector[2][0]);
                
        state  = PROCESSING;
        calib_north_vector_done = true;
    }
}

bool Magnetometer::newCalibDataAvailable() {
    if (calib_north_vector_done || calib_hard_iron_done) {
        calib_north_vector_done = false;
        calib_hard_iron_done = false;
        return true;
    }
    return false;
}
