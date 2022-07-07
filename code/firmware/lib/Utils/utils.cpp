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


// set the watchdog timer to 100ms for normal operations
void fastWatchdog() {
  WDT_timings_t config;
  config.trigger = 0.1; /* [s], trigger is how long before the watchdog callback fires */
  config.timeout = 0.1; /* [s] timeout is how long before not feeding will the watchdog reset */
  config.callback = watchdogWarning;
  wdt.begin(config);
}


// set the watchdog timer to 120s (only used for longer things like calibrating things)
void slowWatchdog() {
  WDT_timings_t config;
  config.trigger = 128.0; /* [s], trigger is how long before the watchdog callback fires */
  config.timeout = 128.0; /* [s] timeout is how long before not feeding will the watchdog reset */
  config.callback = watchdogWarning;
  wdt.begin(config);
}

// code stolen from https://stackoverflow.com/questions/11103683/euler-angle-to-quaternion-then-quaternion-to-euler-angle
void quaternion2RPY(double x, double y, double z, double w , double rpy[])
{

    // roll (x-axis rotation)
    double sinr_cosp =     2 * (w * x + y * z);
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


void  cross_product(double v_A[], double  v_B[], double c_P[3]) {
  c_P[0] =   v_A[1] * v_B[2] - v_A[2] * v_B[1];
  c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
  c_P[2] =   v_A[0] * v_B[1] - v_A[1] * v_B[0];
}

double dot_product(double vector_a[3], double vector_b[3]) {
   double product = vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1] + vector_a[2] * vector_b[2];
   return product;
}

void rotate_by_quat(double value[3], double rotation[4], double result[3])
{
    float num12 = rotation[1] + rotation[1];
    float num2 = rotation[2] + rotation[2];
    float num = rotation[3] + rotation[3];
    float num11 = rotation[0] * num12;
    float num10 = rotation[0] * num2;
    float num9 = rotation[0] * num;
    float num8 = rotation[1] * num12;
    float num7 = rotation[1] * num2;
    float num6 = rotation[1] * num;
    float num5 = rotation[2] * num2;
    float num4 = rotation[2] * num;
    float num3 = rotation[3] * num;
    float num15 = ((value[0] * ((1. - num5) - num3)) + (value[1] * (num7 - num9))) + (value[2] * (num6 + num10));
    float num14 = ((value[0] * (num7 + num9)) + (value[1] * ((1. - num8) - num3))) + (value[2] * (num4 - num11));
    float num13 = ((value[0] * (num6 - num10)) + (value[1] * (num4 + num11))) + (value[2] * ((1. - num8) - num5));
    result[0] = num15;
    result[1] = num14;
    result[2] = num13;
}