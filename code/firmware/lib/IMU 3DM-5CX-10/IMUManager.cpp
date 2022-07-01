
#include <IMUManager.h>
#include <TimePassedBy.h>
#include <ukf.h>
#include <Magnetometer.h>

void IMUManager::setup(uint16_t targetFreq) {
      // IMU's power is controlled by this PIN
      // Initially the IMU is down, start happens in main loop
      pinMode(PIN_IMU_POWER, OUTPUT);
      digitalWrite(PIN_IMU_POWER, HIGH); // turn off IMU

  sampleFreq  = targetFreq; 

  filter.setup(targetFreq);
}
void IMUManager::loop() {

  // take care that imu is correctly powered up or powered down
  updatePowerState();

  static uint32_t ekftime = 0;

  // if data streaming is on, pump accel and gyro into EKF
  if (isUpAndRunning()) {
      if (device.isNewPackageAvailable()) {
        IMUSensorData sensorData = device.getIMUSensorData();
        
        /* ================== Read the sensor data / simulate the system here ================== */
        float Ax = sensorData.acc_x;
        float Ay = sensorData.acc_y;
        float Az = sensorData.acc_z;
        float p = sensorData.gyro_x;
        float q = sensorData.gyro_y;
        float r = sensorData.gyro_z;
  
        double x,y,z,w;
        uint64_t u64compuTime = micros();
        filter.compute(Ax,Ay,Az,p,q,r, 
#ifdef WITH_MAG       
                        0,0,0,
#endif                        
        x,y, z,w);
        ekftime  = (ekftime + (micros() - u64compuTime))/2;
        quaternion[0] = x;
        quaternion[1] = y;
        quaternion[2] = z;
        quaternion[3] = w;
        
        quaternion2RPY(x, y,z,w, RPY);
        RPY_deg[0] = RPY[0]/(2*3.1415)*360.0;
        RPY_deg[1] = RPY[1]/(2*3.1415)*360.0;
        RPY_deg[2] = RPY[2]/(2*3.1415)*360.0;
      }
  }

  // if IMU is supposed to be verbose print out current values
  if (logisOn && isUpAndRunning()) {
    static TimePassedBy imuTimer (250);
    if (imuTimer.isDue()) {
      float freq = getAvrFrequency();
      device.printData();
      println("   EKF time: %d us",ekftime);

      println("   avr freq : %.2f Hz",freq);
      println("   RPY : %.1f / %.1f / %.1f",RPY_deg[0], RPY_deg[1], RPY_deg[2]);
      println("   Quat : (%.3f, %.4f, %.3f, %.3f)",quaternion[0],quaternion[1], quaternion[2], quaternion[3]);
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
            println("IMU:prepare for power up");
            digitalWrite(PIN_IMU_POWER, HIGH);
            lastPhaseStart_ms = millis();
            imuState = IMU_PREPARE_POWER_UP;
            cmdPowerUp = false;
          } 
          break;
        // ledave power off for 500ms to ensure it is properly down
        // (this is relevant for resets of the uC, which would otherwise turn off power only for 50ms)
        case IMU_PREPARE_POWER_UP:
          if (millis() - lastPhaseStart_ms > prepare_power_up_ms) {
            println("IMU:turn on power");
            digitalWrite(PIN_IMU_POWER, LOW);
            lastPhaseStart_ms = millis();
            imuState = IMU_WARMING_UP;
          }
          break;
        // power is on, wait  200ms before trying to communicate
        case IMU_WARMING_UP:
          if (millis() - lastPhaseStart_ms > warmup_duration_ms) {
            println("IMU:setup");
            imuState = IMU_POWERED_UP;
          }
          break;
        case IMU_POWERED_UP: {
            slowWatchdog();
            bool ok = device.setup(&Serial4, sampleFreq);
            filter.reset();

            fastWatchdog();
            if (ok) {
              imuState = IMU_SETUP;
              filterWarmedUp = false;
              lastPhaseStart_ms = millis();
            }
            else {
              digitalWrite(PIN_IMU_POWER, HIGH);
              imuState = IMU_COOLING_DOWN;
              lastPhaseStart_ms = millis();
            }
            break;
          }
          case IMU_SETUP:
            device.loop();
            if (cmdPowerDown) {
              digitalWrite(PIN_IMU_POWER, HIGH);
              println("IMU:cool down");
              imuState = IMU_COOLING_DOWN;
              lastPhaseStart_ms = millis();
              cmdPowerDown = false;
              filterWarmedUp = false;
            } else {
              if (millis() - lastPhaseStart_ms >warmup_filter_ms)
                filterWarmedUp = true;
            }
            break;
          case IMU_COOLING_DOWN:
            if (millis() - lastPhaseStart_ms > warmup_duration_ms) {
              println("IMU:power turned off");
              imuState = IMU_UNPOWERED;
            }
            break;
          default:
            println("unkown IMU state %d.", imuState);
            break;
      }
}