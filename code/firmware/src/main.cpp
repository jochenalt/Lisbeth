#include <Arduino.h>
#include <version.h> 
#include <config.h>
#include <HardwareSerial.h>
#include <utils.h>

#include <ODrive.h>
#include "PatternBlinker.h"
#include <Watchdog.h>

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

// Teensy Watchdog 
WDT_T4<WDT1> wdt;

// Teensy LED
static uint8_t DefaultBlinkPattern[3] = { 0b11001000,0b00001100,0b10000000}; // nice pattern. Each digit takes 50ms
PatternBlinker blinker;													// initiate pattern blinker

// Error handling
uint8_t NO_ERROR = 0;
uint8_t ODRIVE_SETUP_VOLTAGE_ERROR = 1;
uint8_t ODRIVE_SETUP_FIRMWARE_ERROR = 2;
uint8_t GENERAL_ERROR = 99;
uint8_t error = NO_ERROR;

void watchdogWarning() {
  println("Watchdog");
}

void initWatchdog() {
  WDT_timings_t config;
  config.trigger = 0.05; /* [s], trigger is how long before the watchdog callback fires */
  config.timeout = 0.1; /* [s] timeout is how long before not feeding will the watchdog reset */
  config.callback = watchdogWarning;
  wdt.begin(config);
}

void initODrives() {
    for (int i = 0;i<NO_OF_ODRIVES;i++) {
      odrive[0].setup(*odriveSerial[i], ODRIVE_SERIAL_BAUD_RATE);

      // check sufficient voltage
      float voltage = odrive[i].getVBusVoltage();
      if (voltage < 12) {
        println("\r\nVoltage of %.2fV too low.", voltage);
        error = ODRIVE_SETUP_VOLTAGE_ERROR;
      }

      // check correct firmware
      if (error == NO_ERROR) {
        uint16_t version_major, version_minor, version_revision;
        odrive[i].getVersion(version_major, version_minor, version_revision);
        if (version_major * 100 + version_minor*10 + version_revision != 54) {
          println("\r\nFirmware must be 0.5.4 but is %d.%d.%d.", version_major, version_minor, version_revision);
          error = ODRIVE_SETUP_FIRMWARE_ERROR; 
        }
      }
  }
}

void setup() {

  Serial.begin(LOG_BAUD_RATE);
  print("Firmware Lisbeth V%d ", version);

  pinMode(LED_BUILTIN, OUTPUT);
  blinker.setup(LED_BUILTIN, 50);

  initODrives();

 	// read configuration from EEPROM (or initialize if EEPPROM is a virgin)
	setupConfiguration();

  // set default blink pattern
  blinker.set(DefaultBlinkPattern,sizeof(DefaultBlinkPattern));		// assign pattern

  if (error == NO_ERROR) {
   	println("OK.");

   	// reset the board when wdt_reset() is not called every 100ms 
    initWatchdog();
  } else {
    print("setup error. Resetting in ");
    for (int i= 5;i>=1;i--) {
      digitalWrite(LED_BUILTIN,HIGH);
      if (i > 1) {
        delay(20);
        digitalWrite(LED_BUILTIN,LOW);
      }

      print ("%d ",i);
      delay(980); // let the watchdog fire
    }
    println("0 Reset.");

   	// reset the board when wdt_reset() is not called every 100ms 
    initWatchdog();
    delay(5000); // let the watchdog fire
  }
}

// print nice help text and give the status
void printHelp() {
  println("\r\nFirmware Lisbeth V%d", version);

  for (int i = 0;i<NO_OF_ODRIVES;i++) {
      float voltage = odrive[i].getVBusVoltage();

       uint16_t version_major, version_minor, version_revision;
       odrive[i].getVersion(version_major, version_minor, version_revision);
       println("   ODrive[%d] V%d.%d.%d, %.2fV", version_major, version_minor, version_revision,i, voltage);
      // print dump of ODrive
      String str = odrive[i].getInfoDump();
      str.replace("Hardware", "   HW");
      str.replace("Firmware", "   FW");
      str.replace("Serial number", "   Serial");
      println(str.c_str());

      for (int motor = 0;motor<2;motor++) {
        print("      M%d",motor);
        float pos, vel, ff;
        odrive[i].getFeedback(motor,pos, vel, ff);
        println(" (pos,vel,ff) = (%.2f, %.2f, %.2f)", pos, vel, ff);
    }
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
			case 'r':
				print("reset.");
        delay(5000);
				print("OK.");

				break;

      default:
				addCmd(inputChar);
			} // switch
		} // if (Serial.available())
}

void loop() {
  now_us = micros();
  
  // feed the watchdog
  wdt.feed();
  
  // everybody loves a blinking LED
  blinker.loop(now_us >> 10);

  executeCommand();
}