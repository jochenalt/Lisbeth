#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include <Arduino.h>



/** Manage the MOSFET power switch  of the motors.
 *  Takes care that after turning it off, there is enough 
 *  time for the capacitors to unload before turning it on again
 */
class PowerManager {
  public:

  enum MotorPowerType  { MOTOR_UNPOWERED = 0, MOTOR_POWERED_UP = 1, MOTOR_COOLING_DOWN = 2};
  const uint32_t necessary_break_ms = 5000;
  PowerManager(uint8_t pin):power_pin(pin) {};

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

  private:
    bool cmdPowerDown = false;
    bool cmdPowerUp = false;
    MotorPowerType motorPowerState = MOTOR_UNPOWERED;
    uint32_t warmingStart_ms = 0;
    uint8_t power_pin = 0;

};

#endif