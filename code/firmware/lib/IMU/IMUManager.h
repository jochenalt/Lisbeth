#ifndef _IMU_MANAGER_H_
#define _IMU_MANAGER_H_

// make room for 3 data packets from IMU
#define SERIAL7_RX_BUFFER_SIZE 160

#include "utils.h"
#include <Watchdog.h>
#include <Arduino.h>

// communication protocol to Microstrain IMU
#include <MicrostrainComm.h>
#include <ukf.h>
#include <Magnetometer.h>

// the IMU can be enabled by this PIN
// LOW = power on
// HIGH = power off
#define PIN_IMU_ENABLE 32
#define IMU_ENABLE_PIN_LEVEL LOW
#define IMU_DISABLE_PIN_LEVEL HIGH

// Pin for turning on/off 3.3V power supply with Magnetometer and IMU
// this pin is connected with the 3.3V voltage regulator SHUTDOWN pin
#define PIN_IMU_POWER 6
#define IMU_POWER_ON_PIN_LEVEL HIGH
#define IMU_POWER_OFF_PIN_LEVEL LOW

/** Manage everything round the IMU.
 *  o Take care of the power management with right timings
 *  o Carry out the communication to the IMU
 *  o Filter the results with an unscented kalman filter
 */

struct IMUConfigDataType {
  double hardIron[3] = { 0,0,0};
  double northVector[3]= { 1,0,0};
  void print() {
    println("hardIron = (%.3f,%.3f,%.3f)", hardIron[0],hardIron[1],hardIron[2]);
    println("north    = (%.3f,%.3f,%.3f)", northVector[0],northVector[1],northVector[2]);
  }
  void setup() {
     hardIron[0] =0;
     hardIron[1] =0;
     hardIron[2] =0;
     northVector[0] = 1;
     northVector[1] = 0;
     northVector[2] = 0;
  }
}  ; 


class IMUManager {
  public:
    // the IMU is a bit sensitive to timing when it comes to power on/off.
    // So we take care that it is properly turned off for 500ms, then turned on, and after 1000ms the communciation starts 
    // we also give the Kalman filter 100ms to warmup before delivering data
    enum ImuStateType  { IMU_UNPOWERED = 0, IMU_WARMING_UP = 1, IMU_PREPARE_POWER_UP=2, IMU_POWERED_UP = 3, IMU_SETUP = 4, IMU_COOLING_DOWN = 5};
    const uint32_t warmup_duration_ms = 2000;
    const uint32_t prepare_power_up_ms = 1000;
    const uint32_t warmup_filter_ms = 100;

    IMUManager() {};

    void setup(HardwareSerial* serial,uint16_t sampleFreq, IMUConfigDataType& config);

    // to be called as often as possible, fetches sensor data and filters it
    void loop();

    // get the latest RPY
    void getPoseRPY(double &r, double &p, double &y);
    void getPoseQuat(double &w, double &x, double &y, double &z);
    void getAngualarRate(double &x, double &y, double &z);
    void getLinearAcceleration(double &x, double &y, double &z);

    void powerUp() { cmdPowerUp = true;}
    void powerDown() { cmdPowerDown = true;}

    // returns true if IMU is power and setup and no error happened  
    bool isUpAndRunning() { return ((imuState == IMU_SETUP) && device.isInitialised() && (filterWarmedUp)); };
    void setLogging(bool ok) { logisOn = ok;  }
    bool isLogging() { return logisOn; }

    float getAvrFrequency();
    void startHardIronCalibration();
    void startNorthCalibration();  
    bool hardIronCalibrationDone();

    // get the current calibration data in a format convinient for storing in EEPROM
    void getCalibrationData(IMUConfigDataType& calib);

    // set calibration data coming from EEPROM  
    void setCalibrationData(IMUConfigDataType& calib);

    // returns true if new calibration is available
    bool newCalibrationData();
  private:
    void updatePowerState();

    bool cmdPowerDown = false;              // set if power off command was initiated
    bool cmdPowerUp = false;                // set of power on command was initiated
    ImuStateType imuState = IMU_UNPOWERED;  // current power state 
    uint32_t lastPhaseStart_ms = 0;         // start time [ms] of prevous phase
    bool filterWarmedUp = false;
    bool logisOn = false;                   // be verbose or shut up

    double RPY[3] = {0,0,0};                // result of Kalman filter [rad]
    double RPY_deg[3] = {0,0,0};            // same but in degrees
    double ang_rate[3] = {0,0,0};           // angular rate
    double lin_acc[3] = {0,0,0};           // linear acceleration, i.e. acceleration without gravity vector in the sensor frame

    double RPY_prev[3] = {0,0,0};           // last result to compute dRPY

    double pose_quat[4] = {0,0,0,0};        // original outcome from Kalman is in quatssame but in quats
    double sampleFreq;                      // frequency of the IMU data stream 

    MicrostrainIMU  device;                 // communication interface to any Parker Lord Microstrain IMU
    Magnetometer mag;                       // communication to LIS3MDL magnetometer
    UnscentedKalmanFilter filter;           // magic filter
    HardwareSerial* serial = NULL;
};

#endif