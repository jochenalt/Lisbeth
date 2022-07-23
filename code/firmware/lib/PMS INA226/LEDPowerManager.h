
#ifndef LED_POWER_MANAGER_H
#define LED_POWER_MANAGER_H

#include <Arduino.h>
#include <Utils.h>
#include <TimePassedBy.h>

#define PIN_PWM_LED 5
#define PWM_LED_ON_LEVEL LOW
#define PWM_LED_OFF_LEVEL HIGH


class LEDPowerManager {
  public:

    LEDPowerManager() {};
    void setup() {
        pinMode(PIN_PWM_LED, OUTPUT);
        digitalWrite(PIN_PWM_LED, PWM_LED_OFF_LEVEL);
        powerOn();
    }

    void loop(uint32_t now_us) {
        // ever ms switch pwm frequency
        if (now_us - last_call_us > 4000) {
            analogWrite(PIN_PWM_LED, pwm_level);            
            pwm_level = (pwm_level+1) % 256;
            last_call_us = now_us; 
        }
    };
    void powerOn() {
        digitalWrite(PIN_PWM_LED, PWM_LED_ON_LEVEL);
    }
    void powerOff() {
        digitalWrite(PIN_PWM_LED, PWM_LED_OFF_LEVEL);
    }

    uint16_t pwm_level = 0;
    uint32_t last_call_us = 0;
};

#endif