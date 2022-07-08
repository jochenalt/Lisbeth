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

                // save  raw data from sensor in [uT] and in sensor's frame
                double sensor_x, sensor_y, sensor_z;
                device.readResponse(sensor_x,sensor_y,sensor_z);

                // Align the coordinate system of the magnetometer with the one from the IMU
                // z is going up
                raw_mag_x =  -sensor_y;
                raw_mag_y =   sensor_x;
                raw_mag_z =   sensor_z;

                // compensate the hard iron
                mag_x = raw_mag_x - hard_iron_base[0][0];
                mag_y = raw_mag_y - hard_iron_base[1][0];
                mag_z = raw_mag_z - hard_iron_base[2][0];

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

void Magnetometer::calibrateLoop(double acc_x, double acc_y, double acc_z) {

    if (state == CALIBRATE_HARD_IRON) {


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
        float Bx = mag_x - hard_iron_base[0][0];
        float By = mag_y - hard_iron_base[1][0];
        float Bz = mag_z - hard_iron_base[2][0];
                
        /* Normalizing the acceleration vector & projecting the gravitational vector (gravity is negative acceleration) */
        double normG = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));
        Ax = Ax / normG;
        Ay = Ay / normG;
        Az = Az / normG;
                
        /* Normalizing the magnetic vector */
        normG = sqrt((Bx * Bx) + (By * By) + (Bz * Bz));
        Bx = Bx / normG;
        By = By / normG;
        Bz = Bz / normG;
        
        /* Projecting the magnetic vector into plane orthogonal to the gravitational vector */
        // double roll = asin(Ay);
        // double pitch = asin(Ax/cos(roll));

        float pitch = asin(-Ax);
        float roll = asin(Ay/cos(pitch));
        double m_tilt_x =  Bx*cos(pitch)             + By*sin(roll)*sin(pitch)   + Bz*cos(roll)*sin(pitch);
        double m_tilt_y =                            + By*cos(roll)              - Bz*sin(roll);
        // double m_tilt_z = -Bx*sin(pitch)             + By*sin(roll)*cos(pitch)   + Bz*cos(roll)*cos(pitch); 

        // ignore yaw
        double mag_dec = atan2(m_tilt_y, m_tilt_x);
        north_vector[0][0] = cos(mag_dec);
        north_vector[1][0] = sin(mag_dec);
        north_vector[2][0] = 0;
        println("Acc =(%.3f,%.3f,%.3f)", Ax, Ay, Az);
        println("B =(%.3f,%.3f,%.3f)", Bx, By, Bz);
        
        println("pitch %.3f roll %.3f m_tilt_x %.3f, m_tilt_y %.3f mag_dec %.3f", pitch/(2.0*3.1415)*360.0, roll/(2.0*3.1415)*360.0, m_tilt_x, m_tilt_y, mag_dec/(2.0*3.1415)*360.0);
                
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
