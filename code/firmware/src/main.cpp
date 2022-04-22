#include <Arduino.h>
#include <avr/wdt.h>
#include <version.h> 
#include <config.h>
#include <ODrive.h>
#include <HardwareSerial.h>

//  baud rate of Serial0 that is used for logging 
#define LOG_BAUD_RATE 115200

#define ODRIVE_SERIAL_BAUD_RATE 115200 // default of odrive, is reconfigured during startup to 
// #define ODRIVE_SERIAL_BAUD_RATE 1826086        // this higher baudrate

#define NO_OF_ODRIVES 1
ODrive odrive[NO_OF_ODRIVES];
HardwareSerial* odriveSerial[NO_OF_ODRIVES] = { &Serial1};

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

  for (int i = 0;i<NO_OF_ODRIVES;i++) {
    odrive[0].setup(*odriveSerial[i]);
    odriveSerial[i]->begin(ODRIVE_SERIAL_BAUD_RATE);
    float voltage = odrive[i].getVBusVoltage();
    Serial.print("ODrive[");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(voltage);
    Serial.print("V ");

    uint16_t version_major, version_minor, version_revision;
    odrive[i].getVersion(version_major, version_minor, version_revision);
    Serial.print("FW ");
    Serial.print( version_major);
    Serial.print(".");
    Serial.print( version_minor);
    Serial.print(".");
    Serial.print( version_revision);
    Serial.print(" ");
    Serial.println();
  }
  

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
  float pos = odrive[0].GetPosition(0);
  Serial.print("pos=");
  Serial.print(pos);


}