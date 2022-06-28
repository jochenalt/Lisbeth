#include <Arduino.h>
#include <cstdarg>

#include <Watchdog.h>


String strPrintf (const char* format, ...)
{
	char s[256];
	__gnuc_va_list  args;
		  
	va_start (args, format);
	vsprintf (s, format, args);
	va_end (args);		
    return s;
}

void print(const char* format, ...) {
	char s[256];
	__gnuc_va_list  args;
		  
	va_start (args, format);
	vsprintf (s, format, args);
	va_end (args);		
    Serial.print(s);
}

void println(const char* format, ...) {
	char s[256];
	__gnuc_va_list  args;
		  
	va_start (args, format);
	vsprintf (s, format, args);
	va_end (args);		
    Serial.println(s);
};


// Watchdog (AVR watchdog does not work on Teensy)
WDT_T4<WDT1> wdt;

void watchdogWarning() {
  println("\r\nWatchdog Reset");
}


// set the watchdog timer to 100ms
void fastWatchdog() {
  WDT_timings_t config;
  config.trigger = 0.1; /* [s], trigger is how long before the watchdog callback fires */
  config.timeout = 0.1; /* [s] timeout is how long before not feeding will the watchdog reset */
  config.callback = watchdogWarning;
  wdt.begin(config);
}


// set the watchdog timer to 120s (used for longer things like calibrating motors)
void slowWatchdog() {
  WDT_timings_t config;
  config.trigger = 128.0; /* [s], trigger is how long before the watchdog callback fires */
  config.timeout = 128.0; /* [s] timeout is how long before not feeding will the watchdog reset */
  config.callback = watchdogWarning;
  wdt.begin(config);
}


void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
  res[0] = atan2( r31, r32 );
  res[1] = asin ( r21 );
  res[2] = atan2( r11, r12 );
}

// code stolen from https://stackoverflow.com/questions/11103683/euler-angle-to-quaternion-then-quaternion-to-euler-angle
void quaternion2RPY(double x, double y, double z, double w , double rpy[])
{
      threeaxisrot( -2*(y*z - w*x),
                    w*w - x*x - y*y + z*z,
                    2*(x*z + w*y),
                   -2*(x*y - w*z),
                    w*w + x*x - y*y - z*z,
                    rpy);
}

void RPY2Quaternion(double RPY[], double &x, double &y, double &z, double &w) {
    double c1 = cos(RPY[0] / 2);
    double c2 = cos(RPY[1] / 2);
    double c3 = cos(RPY[2] / 2);
    double s1 = sin(RPY[0] / 2);
    double s2 = sin(RPY[1] / 2);
    double s3 = sin(RPY[2] / 2);
    x = s1 * c2 * c3 + c1 * s2 * s3;
    y = c1 * s2 * c3 - s1 * c2 * s3;
    z = c1 * c2 * s3 + s1 * s2 * c3;
    w = c1 * c2 * c3 - s1 * s2 * s3;
};