#ifndef _IMU_MANAGER_H_
#define _IMU_MANAGER_H_

#include <Arduino.h>
#include <imu.h>
#include <utils.h>


// the IMU can be powered by this PIN
// LOW = power on
// The IMU takes 500ms to power up
#define PIN_IMU_POWER 9

// interface to Lord Microstrain 33DM-GX3-25
IMU imu;

/** Manage the lifecycle of the IMU, especially all the timing contraints
 */
class IMUManager {
  public:
    enum ImuStateType  { IMU_UNPOWERED = 0, IMU_WARMING_UP = 1, IMU_POWERED_UP = 2, IMU_SETUP = 3, IMU_COOLING_DOWN = 4};
    const uint32_t warmup_duration_ms = 2000;

    IMUManager() {};

    void loop() {
      switch (imuState) {
        case IMU_UNPOWERED:
          if (cmdPowerUp) {
            digitalWrite(PIN_IMU_POWER, LOW);
            warmingStart_ms = millis();
            imuState = IMU_WARMING_UP;
            cmdPowerUp = false;
          } 
          break;
        case IMU_WARMING_UP:
          if (millis() - warmingStart_ms > warmup_duration_ms) {
            println("IMU warmed up");
            imuState = IMU_POWERED_UP;
          }
          break;
        case IMU_POWERED_UP: {
            slowWatchdog();
            bool ok = imu.setup(&Serial4);
            fastWatchdog();
            if (ok) {
              imuState = IMU_SETUP;
            }
            else {
              digitalWrite(PIN_IMU_POWER, HIGH);
              imuState = IMU_COOLING_DOWN;
              warmingStart_ms = millis();
            }
            break;
          }
          case IMU_SETUP:
            imu.loop();
            if (cmdPowerDown) {
              digitalWrite(PIN_IMU_POWER, HIGH);
              imuState = IMU_COOLING_DOWN;
              warmingStart_ms = millis();
              cmdPowerDown = false;
            }
            break;
          case IMU_COOLING_DOWN:
            if (millis() - warmingStart_ms > warmup_duration_ms) {
              imuState = IMU_UNPOWERED;
            }
            break;
          default:
            println("unkown IMU state %d.", imuState);
            break;
      }
  }
  void powerUp() { cmdPowerUp = true;}
  void powerDown() { cmdPowerDown = true;}

  private:
    bool cmdPowerDown = false;
    bool cmdPowerUp = false;
    ImuStateType imuState = IMU_UNPOWERED;
    uint32_t warmingStart_ms = 0;

};

#endif