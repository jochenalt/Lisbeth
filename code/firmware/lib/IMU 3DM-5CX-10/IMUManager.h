#ifndef _IMU_MANAGER_H_
#define _IMU_MANAGER_H_
#include "utils.h"

#include <Arduino.h>
// communication protocol to Microstrain IMU
#include <MicrostrainComm.h>
#include <ukf.h>

// the IMU can be enabled  by this PIN
// LOW = power on
// HIGH = power off
#define PIN_IMU_POWER 9

/** Manage everything round the IMU.
 *  o Take care of the power management with right timings
 *  o Carry out the communication to the IMU
 *  o Filter the results with an unscented kalman filter
 */
class IMUManager {
  public:
    // the IMU is a bit sensitive to timing when it comes to power on/off.
    // So we take care that it is properly turned off for 500ms, then turned on, and after 1000ms the communciation starts 
    // we also give the Kalman filter 100ms to warmup before delivering data
    enum ImuStateType  { IMU_UNPOWERED = 0, IMU_WARMING_UP = 1, IMU_PREPARE_POWER_UP=2, IMU_POWERED_UP = 3, IMU_SETUP = 4, IMU_COOLING_DOWN = 5};
    const uint32_t warmup_duration_ms = 1000;
    const uint32_t prepare_power_up_ms = 500;
    const uint32_t warmup_filter_ms = 100;

    IMUManager() {};

    void setup(uint16_t sampleFreq);

    void loop();
    void powerUp() { cmdPowerUp = true;}
    void powerDown() { cmdPowerDown = true;}

    // returns true if IMU is power and setup and no error happened  
    bool isUpAndRunning() { return ((imuState == IMU_SETUP) && device.isInitialised() && (filterWarmedUp)); };
    void setLogging(bool ok) { logisOn = ok;  }
    bool isLogging() { return logisOn; }

    float getAvrFrequency();

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
    double quaternion[4] = {0,0,0,0};       // same but in quats
    uint16_t sampleFreq;                    // frequency of the IMU data stream 

    MicrostrainIMU  device;                 // communication interface to any Parker Lord Microstrain IMU
    UnscentedKalmanFilter filter;           // magic filter
};

#endif