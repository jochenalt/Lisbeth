
#include <IMUManager.h>
#include <TimePassedBy.h>
#include <ukf.h>

// #include <EKFManager.h>

void IMUManager::setup(uint16_t targetFreq) {
      // IMU's power is controlled by this PIN
      // Initially the IMU is down, start happens in main loop
      pinMode(PIN_IMU_POWER, OUTPUT);
      digitalWrite(PIN_IMU_POWER, HIGH); // turn off IMU

  sampleFreq  = targetFreq; 

  setupKalman();
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
        computeKalman(Ax,Ay,Az,p,q,r, x,y, z,w);
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
            bool ok = device.setup(&Serial4, sampleFreq);
            resetKalman();

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