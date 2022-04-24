#ifndef _UTILS_H_
#define _UTILS_H_

#include <Arduino.h>
// like sprintf but with a String as output
String strPrintf (const char* format, ...);

// like Serial.print(StrPrintf(format,...))
void print(const char* format, ...);

// like Serial.println(StrPrintf(format,...))
void println(const char* format, ...);

inline void println() {
    Serial.println();
}


#endif