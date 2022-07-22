

#include <IMUManager.h>
#include <TimePassedBy.h>
#include <ukf.h>
#include <Magnetometer.h>

void IMUManager::setup(HardwareSerial* sn,uint16_t targetFreq, IMUConfigDataType& config) {
  // IMU's enable function is controlled by this PIN
  pinMode(PIN_IMU_ENABLE, OUTPUT);
  digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL); // turn off IMU

  pinMode(PIN_IMU_POWER, OUTPUT);
  digitalWrite(PIN_IMU_POWER, IMU_POWER_ON_PIN_LEVEL); // SHTDN pin = high turns on power supply 3.3V
  
  // Initially the IMU is down, start happens in main loop
  imuState = IMU_UNPOWERED;

  serial = sn;
  sampleFreq  = targetFreq; 

  // initialise unsenced kalman filter
  filter.setup(targetFreq);

  // set the calibration data (hard iron and north)
  setCalibrationData(config);

  // the magnetometer works with maximum precision at 155Hz  
  // its power management is quite robust, does not need the 
  // fancy power management like the IMU
  delay(10);
  mag.setup(DATARATE_155_HZ, RANGE_4_GAUSS);
}

void IMUManager::loop() {
  mag.loop();

  // take care that imu is correctly powered up or powered down
  updatePowerState();

  // in the magnetometer calibration process we need 
  // the acceleration data from the IMU to get the north

  // if data streaming is on, pump accel and gyro into EKF
  if (isUpAndRunning()) {
      if (device.isNewPackageAvailable()) {
        IMUSensorData sensorData = device.getIMUSensorData();
        
        // read accel and gyro data
        double Ax = sensorData.acc_x;
        double Ay = sensorData.acc_y;
        double Az = sensorData.acc_z;

        double p = sensorData.gyro_x;
        double q = sensorData.gyro_y;
        double r = sensorData.gyro_z;

        // read delta velocity vector
        double dvx = sensorData.delta_velocity_x;
        double dvy = sensorData.delta_velocity_y;
        double dvz = sensorData.delta_velocity_z;

        // fetch last data point from magnetometer via I2C (might be the same like last time, since magnetometer runs slower)
        // loop the magnetometer (it runs slower than the IMU, so new_mag_value indicates when a new data point is available)
        mag.loop();

        // read last results from memory, regardless if it is new or not
        double Bx, By, Bz;
        mag.read(Bx, By, Bz);

        // in the magnetometer calibration process we need 
        // the acceleration data from the IMU to get the north
        if (mag.isDataAvailable()) {
          mag.calibrateLoop(Ax, Ay, Az);
        }

        // run unscented kalman filter, and get result in quaternion
        double res_x,res_y,res_z,res_w;
        filter.compute(Ax,Ay,Az,p,q,r, 
                       Bx,By,Bz,
                       res_x,res_y, res_z,res_w);

        // convert it to RPY after saving the prev result
        RPY_prev[0] = RPY[0];
        RPY_prev[1] = RPY[1];
        RPY_prev[2] = RPY[2];
        quaternion2RPY(res_x, res_y,res_z,res_w, RPY);

        // store quaternion
        pose_quat[1] = res_x;
        pose_quat[2] = res_y;
        pose_quat[3] = res_z;
        pose_quat[0] = res_w;

        // compute angular rate
        // since he filter is running warm we do not have to take care of the very first sample, which will give a crazy angular rate
        double dT = 1.0/sampleFreq;
        ang_rate[0] = (RPY[0] - RPY_prev[0]) *dT;
        ang_rate[1] = (RPY[1] - RPY_prev[1]) *dT;
        ang_rate[2] = (RPY[2] - RPY_prev[2]) *dT;

        // guess what this is
        const double g = 9.80665;

        // the IMU delivers delta velocity including the gravity vector, that needs to be removed
        double lin_acc_with_gravity[3] = {dvx/dT,dvy/dT,dvz/dT};
        double rotated_lin_acc[3];

        // rotate the linear acceleration towards the gravity vector
        rotate_by_quat(lin_acc_with_gravity, pose_quat, rotated_lin_acc);

        // now we can substract gravity by this
        rotated_lin_acc[2] -= 1.0;

        // to rotate it back to the IMUs frame, the inverted quaternion is needed
        double pose_quat_inverted[4] = { -pose_quat[0], -pose_quat[1], -pose_quat[2], pose_quat[3]};
        rotate_by_quat(rotated_lin_acc, pose_quat_inverted, lin_acc);

        // return the linear acceleration in [m/s]
        lin_acc[0] *= g;
        lin_acc[1] *= g;
        lin_acc[2] *= g;

        // for logging output compute RPY in [degree] instead of [rad]
        RPY_deg[0] = RPY[0]/(2*M_PI)*360.0;
        RPY_deg[1] = RPY[1]/(2*M_PI)*360.0;
        RPY_deg[2] = RPY[2]/(2*M_PI)*360.0;

        // if IMU is supposed to be verbose print out current values
        if (logisOn && isUpAndRunning()) {
          static TimePassedBy imuTimer (100);
          if (imuTimer.isDue()) {
            float freq = getAvrFrequency();
            device.printData();
            println("   linAcc: %.3f / %.3f / %.3f",lin_acc_with_gravity[0], lin_acc_with_gravity[1], lin_acc_with_gravity[2]);
            println("   lin_acc/g: %.2f / %.2f / %.2f",lin_acc[0], lin_acc[1], lin_acc[2]);

            println("   avr freq : %.2f Hz",freq);
            println("   RPY : %.1f / %.1f / %.1f",RPY_deg[0], RPY_deg[1], RPY_deg[2]);
            println("   Quat xyzw: (%.3f, %.4f, %.3f, %.3f)",pose_quat[0],pose_quat[1], pose_quat[2], pose_quat[3]);
            println("   Mag : (%.3f, %.4f, %.3f)",Bx, By, Bz);
          }
        }
      }
  }
}

// fetch latest pose in RPY 
void IMUManager::getPoseRPY(double &r, double &p, double &y) {
    r = RPY[0];
    p = RPY[1];
    y = RPY[2];
}

// fetch latest pose in RPY 
void IMUManager::getPoseQuat(double &w, double &x, double &y, double &z) {
    w = pose_quat[0];
    x = pose_quat[1];
    y = pose_quat[2];
    z = pose_quat[3];
}

// fetch latest data point in RPY 
void IMUManager::getAngualarRate(double &x, double &y, double &z) {
  x = ang_rate[0];
  y = ang_rate[1];
  z = ang_rate[2];
}

void IMUManager::getLinearAcceleration(double &x, double &y, double &z){
  x = lin_acc[0];
  y = lin_acc[1];
  z = lin_acc[2];
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
            digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL); // disable IMU

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
            digitalWrite(PIN_IMU_ENABLE, IMU_ENABLE_PIN_LEVEL);
            lastPhaseStart_ms = millis();
            imuState = IMU_WARMING_UP;
          }
          break;
        // power is on, wait  200ms before trying to communicate
        case IMU_WARMING_UP:
          if (millis() - lastPhaseStart_ms > warmup_duration_ms) {
            println("IMU:setup");
            digitalWrite(PIN_IMU_ENABLE, IMU_ENABLE_PIN_LEVEL);
            imuState = IMU_POWERED_UP;
          }
          break;
        case IMU_POWERED_UP: {
            slowWatchdog();

            bool ok = device.setup(serial, sampleFreq);
            filter.reset();

            fastWatchdog();
            if (ok) {
              imuState = IMU_SETUP;
              filterWarmedUp = false;
              lastPhaseStart_ms = millis();
            }
            else {
              digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL);
              imuState = IMU_COOLING_DOWN;
              lastPhaseStart_ms = millis();
            }
            break;
          }
          case IMU_SETUP:
            device.loop();

            // the device is very sensitive to EMV, 
            // if something is not right, communication 
            // breaks and isInitialized becomes false
            // in that case restart
            if (!device.isInitialised()) {
              println("IMU:recover ");
              cmdPowerUp = true;
              imuState = IMU_UNPOWERED;
              digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL); // turn off IMU
              println("IMU:cool down");
            } else {
              if (cmdPowerDown) {
                digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL); // turn off IMU
                println("IMU:cool down");
                imuState = IMU_COOLING_DOWN;
                lastPhaseStart_ms = millis();
                cmdPowerDown = false;
                filterWarmedUp = false;
              } else {
                if (millis() - lastPhaseStart_ms >warmup_filter_ms)
                  filterWarmedUp = true;
              }
            }
            break;
          case IMU_COOLING_DOWN:
            if (millis() - lastPhaseStart_ms > warmup_duration_ms) {
              println("IMU:power turned off");
              digitalWrite(PIN_IMU_ENABLE, IMU_DISABLE_PIN_LEVEL); // turn off IMU
              imuState = IMU_UNPOWERED;
            }
            break;
          default:
            println("unkown IMU state %d.", imuState);
            break;
      }
}


void IMUManager::startHardIronCalibration() {
  mag.startHardIronCalibration();
}

void IMUManager::startNorthCalibration(){
  println("IMU: north calibration");
  mag.startNorthCalibration();
}

void IMUManager::getCalibrationData(IMUConfigDataType& calib) {
  calib.hardIron[0] = mag.getHardIronBase()[0][0];
  calib.hardIron[1] = mag.getHardIronBase()[1][0];
  calib.hardIron[2] = mag.getHardIronBase()[2][0];

  calib.northVector[0] = mag.getNorthVector()[0][0];
  calib.northVector[1] = mag.getNorthVector()[1][0];
  calib.northVector[2] = mag.getNorthVector()[2][0];
}

void IMUManager::setCalibrationData(IMUConfigDataType& calib) {
  calib.print();
  mag.getHardIronBase()[0][0] = calib.hardIron[0];
  mag.getHardIronBase()[1][0] = calib.hardIron[1];
  mag.getHardIronBase()[2][0] = calib.hardIron[2];

  mag.getNorthVector()[0][0] = calib.northVector[0];
  mag.getNorthVector()[1][0] = calib.northVector[1];
  mag.getNorthVector()[2][0] = calib.northVector[2];

  filter.setNorthVector(mag.getNorthVector());
}

bool IMUManager::newCalibrationData() {
  bool ok = mag.newCalibDataAvailable();
  if (ok) {
    filter.setNorthVector(mag.getNorthVector());
    return true;
  }
  return false;
}

