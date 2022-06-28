
#include <IMUManager.h>

#include <TimePassedBy.h>
void IMUManager::setup() {
      // IMU's power is controlled by this PIN
      // Initially the IMU is down, start happens in main loop
      pinMode(PIN_IMU_POWER, OUTPUT);
      digitalWrite(PIN_IMU_POWER, HIGH); // turn off IMU
}

void IMUManager::loop() {
      updatePowerState();

      // if IMU is supposed to be verbose print out current values
      if (log && isUpAndRunning()) {
          static TimePassedBy imuTimer (1000);
          if (imuTimer.isDue()) {
            float freq = getAvrFrequency();
            device.printData();
            print("   avr freq : %.2f Hz",freq);
          }
      }
}

float IMUManager::getAvrFrequency() { 
      if (isUpAndRunning()) {
        return device.getMeasuremt().getAvrFreq();
      }
      return 0;
};

void IMUManager::updatePowerState() {
      switch (imuState) {
        case IMU_UNPOWERED:
          if (cmdPowerUp) {
            println("power up IMU");
            digitalWrite(PIN_IMU_POWER, LOW);
            warmingStart_ms = millis();
            imuState = IMU_WARMING_UP;
            cmdPowerUp = false;
          } 
          break;
        case IMU_WARMING_UP:
          if (millis() - warmingStart_ms > warmup_duration_ms) {
            println("warm up IMU");
            imuState = IMU_POWERED_UP;
          }
          break;
        case IMU_POWERED_UP: {
            slowWatchdog();
            println("setup IMU");
            bool ok = device.setup(&Serial4);
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
            device.loop();
            if (cmdPowerDown) {
              digitalWrite(PIN_IMU_POWER, HIGH);
              println("cool down IMU");
              imuState = IMU_COOLING_DOWN;
              warmingStart_ms = millis();
              cmdPowerDown = false;
            }
            break;
          case IMU_COOLING_DOWN:
            if (millis() - warmingStart_ms > warmup_duration_ms) {
              println("IMU is unpowered");
              imuState = IMU_UNPOWERED;
            }
            break;
          default:
            println("unkown IMU state %d.", imuState);
            break;
      }
}