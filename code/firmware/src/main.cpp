#include <Arduino.h>
#include <avr/wdt.h>
#include <version.h> 
#include <config.h>


// error codes
#define RETURN_OK 0								// command executed successfully
const uint8_t number_of_error_codes = RETURN_OK+1;

//  baud rate of Serial0 that is used for logging 
#define LOG_BAUD_RATE 115200


void setup() {
  Serial.begin(LOG_BAUD_RATE);
  Serial.print("Firmware Lisbeth V");
  Serial.println(VERSION);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50);                       
  digitalWrite(LED_BUILTIN, LOW);  
  delay(50);                       
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50);                       
  digitalWrite(LED_BUILTIN, LOW);  

 	// read configuration from EEPROM (or initialize if EEPPROM is a virgin)
	setupConfiguration();
  config.counter++;
  writeConfiguration();
  Serial.print("persCounter");
  Serial.print(config.counter);

 	// reset the board when wdt_reset() is not called every 120ms 
  wdt_enable(WATCH_DOG_WAIT);
}

void loop() {
  Serial.println("Loop");

  wdt_reset();
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(500);                       
  digitalWrite(LED_BUILTIN, LOW);   
  delay(200);                       
}