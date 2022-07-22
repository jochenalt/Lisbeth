#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include <Arduino.h>
#include <INA226.h>
#include <Utils.h>
#include <TimePassedBy.h>



/** Manage the MOSFET power switch  of the motors.
 *  Takes care that after turning it off, there is enough 
 *  time for the capacitors to unload before turning it on again
 */
class MotorPowerManager {
  public:

  enum MotorPowerType  { MOTOR_UNPOWERED = 0, MOTOR_POWERED_UP = 1, MOTOR_COOLING_DOWN = 2};
  const uint32_t necessary_break_ms = 5000;
  MotorPowerManager(uint8_t pin):power_pin(pin) {};

  bool setup() {
    // the motor MOSFETs are controlled by this PIN
    pinMode(power_pin, OUTPUT);
    digitalWrite(power_pin, LOW);

	  Serial.println("PMS: setup");
    Wire.begin();
    if (!device.begin() )
    {
      Serial.println("could not connect to power sensor.");
      return false;
    }
    device.reset();
    device.setMaxCurrentShunt(10, 0.01);


    return true;
  }

  float getVoltage() {
      float v = device.getBusVoltage();
      return v;
  }

  float getCurrent() {
      float a = device.getCurrent();
      if (a<0)
        a = 0;
      return a;
  }

  float getPower() {
      return getCurrent() * getVoltage();
  }

  void updatePowerState() {
    switch (motorPowerState) {
      case MOTOR_UNPOWERED:
        if (cmdPowerUp) {
            println("PMS: turn motor power on");
            digitalWrite(power_pin, HIGH);
            warmingStart_ms = millis();
            motorPowerState = MOTOR_POWERED_UP;
            cmdPowerUp = false;
        } 
        if ( cmdPowerDown) {
          println("PMS: power is already off");
          cmdPowerDown = false;
        }

        break;
      case MOTOR_POWERED_UP:
          if (cmdPowerDown) {
              println("PMS: turn motor power off");
              digitalWrite(power_pin, LOW);
              motorPowerState = MOTOR_COOLING_DOWN;
              warmingStart_ms = millis();
              cmdPowerDown = false;
          }
          if ( cmdPowerUp) {
            println("PMS: power is already on");
            cmdPowerUp = false;
          }

          break;
      case MOTOR_COOLING_DOWN:
          if (millis() - warmingStart_ms > necessary_break_ms) {
            motorPowerState = MOTOR_UNPOWERED;
          }
          break;
      default:
          println("unkown motor power  state %d.", motorPowerState);
          break;
    }
  }

  void loop() {
      updatePowerState();

      static TimePassedBy imuTimer (1000);
      static bool voltageOrCurrent = false;

      // every 2nd second we fetch voltage
      // ever other second we fetch current
      if (imuTimer.isDue()) {
        if (voltageOrCurrent)
          lastVoltage = getVoltage();
        else
          lastCurrent = getCurrent();
        voltageOrCurrent = !voltageOrCurrent;
      }
  }
  void powerUp() { 
    cmdPowerUp = true;
    Serial.println("PMS: power up command");
  }
  void powerDown() { 
    cmdPowerDown = true;
    Serial.println("PMS: power down command");
  }

  void print() {
    Serial.println("Power Management");
    float v = lastVoltage;
    println("   Voltage:  %0.2fV",v);
    float a = lastCurrent * 1000.0;
    println("   Current:  %0.2fmA",a);
    float w = lastVoltage * lastCurrent / 1000.0;
    println("   Power:    %0.2fW",w);
  }

  private:
    bool cmdPowerDown = false;                        // true, if the command to power down is active
    bool cmdPowerUp = false;                          // true, if the command to power up is active
    MotorPowerType motorPowerState = MOTOR_UNPOWERED; // current state of the motor power pin
    uint32_t warmingStart_ms = 0;     
    uint8_t power_pin = 0;                            // Teensy pin to turn off and on motor power via the enable pin of the driver LTC7001 
    INA226 device = INA226(0x40);                     // Power sensor INA226

    float lastVoltage = 0;
    float lastCurrent = 0;
};

#endif