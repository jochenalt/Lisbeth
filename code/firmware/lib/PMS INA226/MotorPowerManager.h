#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include <Arduino.h>
#include <INA226.h>
#include <Utils.h>



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
	  Serial.println("PMS: setup");
    Wire.begin();
    if (!device.begin() )
    {
      Serial.println("could not connect to power sensor.");
      return false;
    }
    device.reset();
    device.setMaxCurrentShunt(3, 0.1);
    return true;
  }

  void loop() {
    switch (motorPowerState) {
      case MOTOR_UNPOWERED:
        if (cmdPowerUp) {
          digitalWrite(power_pin, HIGH);
          warmingStart_ms = millis();
          motorPowerState = MOTOR_POWERED_UP;
          cmdPowerUp = false;
        } 
        break;
      case MOTOR_POWERED_UP:
          if (cmdPowerDown) {
            digitalWrite(power_pin, LOW);
            motorPowerState = MOTOR_COOLING_DOWN;
            warmingStart_ms = millis();
            cmdPowerDown = false;
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
  void powerUp() { cmdPowerUp = true;}
  void powerDown() { cmdPowerDown = true;}

  void print() {
    Serial.println("Power Management");
    float v = device.getBusVoltage();
    println("   Voltage:  %0.2fV",v);
    float a = device.getCurrent_mA();
    println("   Current:  %0.2fmA",a);
    float w = device.getPower_mW();
    println("   Power:    %0.2fmW",w);
  }

  private:
    bool cmdPowerDown = false;                        // true, if the command to power down is active
    bool cmdPowerUp = false;                          // true, if the command to power up is active
    MotorPowerType motorPowerState = MOTOR_UNPOWERED; // current state of the motor power pin
    uint32_t warmingStart_ms = 0;     
    uint8_t power_pin = 0;            // Teensy pin to turn off and on motor power via the enable pin of the driver LTC7001 
    INA226 device = INA226(0x40);     // Power sensor INA226
};

#endif