#include <Arduino.h>
#include <avr/wdt.h>
#include <version.h> 
#include <config.h>
#include <ODrive.h>
#include <HardwareSerial.h>
#include <utils.h>

//  baud rate of Serial0 that is used for logging 
#define LOG_BAUD_RATE 115200

#define ODRIVE_SERIAL_BAUD_RATE 115200                      // default of odrive, is reconfigured during startup to 
// #define ODRIVE_SERIAL_BAUD_RATE 1826086                  // this higher baudrate

#define NO_OF_ODRIVES 1                                     // number of drives we have connected
ODrive odrive[NO_OF_ODRIVES];                               //  ODrive objects, one per ODrive
HardwareSerial* odriveSerial[NO_OF_ODRIVES] = { &Serial1};  // their UART interface

// Command processing
uint32_t now_us = 0;                                        // current time in us, set in loop()
bool commandPending = false;                                // true if command processor is waiting for more input
String command;                                             // current command coming in
uint32_t commandLastChar_us = 0;                             // time when the last character came in 

void setup() {

  Serial.begin(LOG_BAUD_RATE);
  println("Firmware Lisbeth V%d", version);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50);                       
  digitalWrite(LED_BUILTIN, LOW);  
  delay(50);                       
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50);                       
  digitalWrite(LED_BUILTIN, LOW);  

  for (int i = 0;i<NO_OF_ODRIVES;i++) {
      odrive[0].setup(*odriveSerial[i], ODRIVE_SERIAL_BAUD_RATE);
  }

 	// read configuration from EEPROM (or initialize if EEPPROM is a virgin)
	setupConfiguration();

 	// reset the board when wdt_reset() is not called every 120ms 
  wdt_enable(WATCH_DOG_WAIT);
}

// print nice help text and give the status
void printHelp() {
  println("\nFirmware Lisbeth V%d", version);

  for (int i = 0;i<NO_OF_ODRIVES;i++) {
      float voltage = odrive[i].getVBusVoltage();
      float pos = odrive[0].GetPosition(0);
      uint16_t version_major, version_minor, version_revision;
      odrive[i].getVersion(version_major, version_minor, version_revision);
      println("   ODrive[%d] (%d.%d.%d) Voltage=%.2fV", version_major, version_minor, version_revision,i, voltage);
      println("   pos=%.3f", pos);
  }
  println();
};


inline void addCmd(char ch) {
	command += ch;
	commandPending = true;
};

inline void emptyCmd() {
	command = "";
	commandPending = false;
};

void executeCommand() {
	// if the last key is too old, reset the command after 1s (command-timeout)
	if (commandPending && (now_us - commandLastChar_us) > 1000000) {
		emptyCmd();
	}
	
	// check for any input
	if (Serial.available()) {
		// store time of last character, for timeout of command
		commandLastChar_us = now_us;
			
		char inputChar = Serial.read();
		switch (inputChar) {
			case 'h':
				if (command == "")
					printHelp();
				else
					addCmd(inputChar);
				break;
      default:
				addCmd(inputChar);
			} // switch
		} // if (Serial.available())
}

void loop() {
  now_us = micros();
  wdt_reset();

  Serial.print("Loop");
  executeCommand();
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(500);                       
  digitalWrite(LED_BUILTIN, LOW);   
  delay(200);          

}