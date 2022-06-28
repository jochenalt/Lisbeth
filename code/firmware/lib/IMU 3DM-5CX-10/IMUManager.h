#ifndef _IMU_MANAGER_H_
#define _IMU_MANAGER_H_
#include "utils.h"

#include <Arduino.h>
// communication protocol to Microstrain IMU
#include <MicrostrainCommProtocol.h>

// Extended Kalman filter
#include <ekf.h>

// the IMU can be powered by this PIN
// LOW = power on
// The IMU takes 500ms to power up
#define PIN_IMU_POWER 9


/** Manage the lifecycle of the IMU with  all the timing contraints:
 * After powering up, we need to wait 500ms before we can send any commands
 */
class IMUManager {
  public:
    enum ImuStateType  { IMU_UNPOWERED = 0, IMU_WARMING_UP = 1, IMU_POWERED_UP = 2, IMU_SETUP = 3, IMU_COOLING_DOWN = 4};
    const uint32_t warmup_duration_ms = 2000;

    IMUManager() {};

    void setup();

    void loop();
    void powerUp() { cmdPowerUp = true;}
    void powerDown() { cmdPowerDown = true;}

    // returns true if IMU is power and setup and no error happened  
    bool isUpAndRunning() { return ((imuState == IMU_SETUP) && device.isInitialised()); };
    void setLogging(bool ok) { log = ok;  }
    bool isLogging() { return log; }

    float getAvrFrequency();

  private:
    void updatePowerState();

    bool cmdPowerDown = false;
    bool cmdPowerUp = false;
    ImuStateType imuState = IMU_UNPOWERED;
    uint32_t warmingStart_ms = 0;
    bool log = false;

    // interface to Lord Microstrain 33DM-GX3-25
    MicrostrainIMU  device;
};

#endif