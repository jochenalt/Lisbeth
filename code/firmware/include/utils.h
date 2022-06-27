#ifndef _UTILS_H_
#define _UTILS_H_

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

#endif