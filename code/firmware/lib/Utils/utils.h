#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>
#include <Watchdog.h>

// like sprintf but with a String as output
String strPrintf (const char* format, ...);

// like Serial.print(StrPrintf(format,...))
void print(const char* format, ...);

// like Serial.println(StrPrintf(format,...))
void println(const char* format, ...);

inline void println() {
    Serial.println();
}

// Watchdog (AVR watchdog does not work on Teensy)
extern WDT_T4<WDT1> wdt;

void watchdogWarning();
// set the watchdog timer to 100ms
void fastWatchdog();
void slowWatchdog();

// geometric calculations
void quaternion2RPY(double x, double y, double z, double w , double rpy[]);
void RPY2Quaternion(double RPY[], double &x, double &y, double &z, double &w);
void rotate_by_quat(double value[3], double rotation[4], double result[3]);


class Measurement {
  const uint8_t filter = 16;
  public:
    Measurement() {
        start_time = micros();
        end_time = start_time;
        duration_us = 0;
        duration_avr = 0;
        deviation_avr = 0;
        within_us = 0;
    };
    virtual ~Measurement() {};

    void start() { 
        start_time = micros();
        end_time = start_time;
    }
    void stop()  {
       end_time = micros(); 
       duration_us = end_time - start_time;
       duration_avr = (duration_avr*(filter-1) + duration_us)/filter;
       uint32_t deviation = duration_us > duration_avr?duration_us-duration_avr:duration_avr-duration_us;
       deviation_avr = (deviation_avr*(filter-1) +deviation)/filter;
    }
    float getTime() { return ((float)duration_us)/1000000.0; };
    float getAvrTime() { return ((float)duration_avr)/1000000.0; };
    float getAvrFreq() { 
      if (duration_avr > 0)
        return 1000000.0/duration_avr;
      else
        return 0;
     }
    float getAvrDeviation() { 
      if (deviation_avr > 0)
        return (deviation_avr / duration_avr);
      else
        return 0;
     }

    void tick() {
      end_time = micros();
      duration_us = end_time - start_time;
 
      if (duration_us == 0) {
        within_us++;
      } else {
        double act_duration = ((double)duration_us) /(double) (within_us+1.);
        double deviation = act_duration > duration_avr?act_duration-duration_avr:duration_avr-act_duration;
        deviation_avr = (deviation_avr*(filter-1) + deviation)/filter;
        duration_avr = (duration_avr*(filter-1) + act_duration)/filter;
        start_time = end_time;
        within_us = 0;
      }
    } 

  private:
    uint32_t start_time;
    uint32_t end_time;
    uint32_t duration_us;
    double duration_avr;
    double deviation_avr;
    uint32_t within_us;
};

#endif