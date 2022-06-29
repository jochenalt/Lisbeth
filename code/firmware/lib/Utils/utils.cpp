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

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    rpy[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        rpy[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        rpy[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    rpy[2] = atan2(siny_cosp, cosy_cosp);

}

void RPY2Quaternion(double RPY[], double &x, double &y, double &z, double &w) {
  // Abbreviations for the various angular functions
    double cy = cos(RPY[2] * 0.5);
    double sy = sin(RPY[2] * 0.5);
    double cp = cos(RPY[1] * 0.5);
    double sp = sin(RPY[1] * 0.5);
    double cr = cos(RPY[0] * 0.5);
    double sr = sin(RPY[0] * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
};


