#include <Arduino.h>
#include <version.h> 
#include <config.h>
#include <HardwareSerial.h>
#include <utils.h>

#include "PatternBlinker.h"
#include <ODrive.h>
#include <IMUManager.h>
#include <MotorPowerManager.h>

//  baud rate of Serial0 that is used for logging 
#define LOG_BAUD_RATE 115200

// Pin for turning on/off the motor power
#define PIN_MOTOR_POWER 30

//--- configure ODrives and motors ---
// ODrive pin 1 goes to Teensy RX
// ODrive pin 2 goes to Teensy TX
// ODrive pin GND goes to Teensy GND 
#define NO_OF_ODRIVES 1                                     // number of drives we have connected
HardwareSerial* odriveSerial[NO_OF_ODRIVES] = { &Serial1};  // their UART interface
// every motor needs to be activated to be used
bool motorActive[NO_OF_ODRIVES][2] { { true, false}};
// names of the odrives (F/H=front,hind, L/R=left/right )
String odriveNames[6] = {"FL", "FL/FR", "FR", "HL", "HL/HR", "HR" };
// names of the motors
String names[12] = {"FL-Hip", "FL-Shoulder", "FL-Knee", "FR-Hip", "FR-Shoulder", "FR-Knee",
                    "HL-Hip", "HL-Shoulder", "HL-Knee", "HR-Hip", "HR-Shoulder", "HR-Knee"};

ODrives odrives;

// Processing of commands coming in via Serial
uint32_t now_us = 0;                                        // current time in us, set in loop()
bool commandPending = false;                                // true if command processor is waiting for more input
String command;                                             // current command coming in
uint32_t commandLastChar_us = 0;                             // time when the last character came in 


// Teensy LED blinker
static uint8_t DefaultBlinkPattern[3] = { 0b11001000,0b00001100,0b10000000}; // nice pattern. Each digit represents 50ms
static uint8_t BlockingBlinkPattern[1] = { 0b11111110};                      // blocking pattern, used for blocking actions like startup of motors 
PatternBlinker blinker;													                           

// Error codes
uint8_t NO_ERROR = 0;
uint8_t ODRIVE_SETUP_VOLTAGE_ERROR = 1;
uint8_t ODRIVE_SETUP_FIRMWARE_ERROR = 2;

uint8_t GENERAL_ERROR = 99;
uint8_t error = NO_ERROR;

// manage the IMU
IMUManager imuMgr;
// manage the power MOSFETs giving power to the motors
MotorPowerManager powerManager(PIN_MOTOR_POWER);

// yield is called randomly by delay, approx. every ms
// we only allow harmless things happening there (e.g. the blinker)
void yield() {
  blinker.loop(millis());
}

void initODrives() {
    for (int i = 0;i<NO_OF_ODRIVES;i++) {
      odrives.addODrive(*odriveSerial[i], names[i*2],names[i*2+1], odriveNames[i]);
      for (int mn = 0;mn < 2;mn++) {
        if (motorActive[i][mn]) {
          odrives[i].activate(mn);
        }
      }
    };

    ODrives::debugComm(config.debugLevel>1);
    ODrives::debugAPI(config.debugLevel>0);

    // set them up
    odrives.setup();

    // do  initial checks
    for (int i = 0;i<NO_OF_ODRIVES;i++) {
      // check baudrate first
      odrives[i].setBaudRate();

      // check sufficient voltage
      float voltage = odrives[i].getVBusVoltage();
      if (voltage < 12) {
        if (voltage < 0.1) {
          println("\r\nODrive is not powered ");
          error = ODRIVE_SETUP_VOLTAGE_ERROR;
        } else {
          println("\r\nVoltage of %.2fV too low.", voltage);
          error = ODRIVE_SETUP_VOLTAGE_ERROR;
        }
      }

      // check correct firmware
      if (error == NO_ERROR) {
        uint16_t version_major, version_minor, version_revision;
        odrives[i].getVersion(version_major, version_minor, version_revision);
        if (version_major * 100 + version_minor*10 + version_revision != 155) {
          println("\r\nFirmware of ODrive[%d]must be 1.5.5 but is %d.%d.%d.", i, version_major, version_minor, version_revision);
          error = ODRIVE_SETUP_FIRMWARE_ERROR; 
        }
      }
  }
}

void setup() {

  Serial.begin(LOG_BAUD_RATE);

  // everybody loves a blinking LED
  pinMode(LED_BUILTIN, OUTPUT);
  blinker.setup(LED_BUILTIN, 50);

 	// read configuration from EEPROM (or initialize if EEPPROM is a virgin)
	Serial.println("EEPROM: setup");
  setupConfiguration();

  // setup power manager
  powerManager.setup();

  // initialise IMU Manager 
	Serial.println("IMU: setup");
  imuMgr.setup(&Serial7, 1000,config.imu);

  // setup IMU for automated start (which will happen during the loop)
  imuMgr.powerUp();


  // setup up all ODrives, motors and encoders
	// Serial.println("setup.ODrives");
  // Serial4.begin(115200);
  // initODrives();

  // set default blink pattern
  blinker.set(DefaultBlinkPattern,sizeof(DefaultBlinkPattern));		// assign pattern

  if (error == NO_ERROR) {
   	// reset the board when wdt_reset() is not called every 100ms 
    fastWatchdog();
  } else {
    /*
    print("setup error. Resetting in ");
    for (int i= 5;i>=1;i--) {
      digitalWrite(LED_BUILTIN,HIGH);
      if (i > 1) {
        delay(20);
        digitalWrite(LED_BUILTIN,LOW);
      }

      print ("%d ",i);
      delay(1000-20); // let the watchdog fire
    }
    println("0 Reset.");

   	// reset the board when wdt_reset() is not called every 100ms 
    fastWatchdog();
    delay(5000); // let the watchdog fire
    */
  }
}

// print nice help text and give the status
void printHelp() {
  println("\r\nFirmware Cerebellar V%d", version);

  if (odrives.isSetup()) {
    for (int i = 0;i<NO_OF_ODRIVES;i++) {
        float voltage = odrives[i].getVBusVoltage();

        uint16_t version_major, version_minor, version_revision;
        odrives[i].getVersion(version_major, version_minor, version_revision);
        println("   ODrive[%d] V%d.%d.%d, %.2fV", i, version_major, version_minor, version_revision, voltage);
        // print dump of ODrive
        String str = odrives[i].getInfoDump();
        str.replace("Hardware", "   HW");
        str.replace("Firmware", "   FW");
        str.replace("Serial number", "   Serial");
        println(str.c_str());

        for (int motor = 0;motor<2;motor++) {
          print("      M%d",motor);
          float pos, vel, ff;
          odrives[i].getFeedback(motor,pos, vel, ff);
          println(" (pos,vel,ff) = (%.2f, %.2f, %.2f)", pos, vel, ff);
      }
      float pos0,vel0,ff0, pos1,vel1,ff1;
      odrives[i].getFeedback(pos0, vel0, ff0, pos1, vel1, ff1);
      println(" (pos,vel,ff) = (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)", pos0, vel0, ff0, pos1, vel1, ff1);
    }
    println("Measurements");
    float freq = 1000000.0/odrives.loopAvrTime_us;
    println("   avr. time for loop : %dus %.2fHz", odrives.loopAvrTime_us,freq);
    println("   avr. delay time    : %dus ", odrives.avrDelayTime_us);
    println("   avr. send time q   : %dus ", odrives.loopSendAvrTime_us);
  }

  println("IMU configuration:");
  config.imu.print();


  println("\r\nUsage:");
  println("   h       - help");
  println("   d<no    - set debug level 0..2");
  println("   c<no>   - calibrate");
  println("   s<no>   - startup");
  println("   S<no>   - shutdown");
  println("   r       - reset");
  println("   s       - startup all");
  println("   S       - shutdown all");
  println("   i       - initialise IMU");
  println("   c       - calibrate hard iron");
  println("   C       - calibrate north");
  println("   l       - log on/off");
  println("   m       - show voltage/current");
  println("   p       - turn motorpower on");
  println("   P       - turn motorpower off");

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
 				if (command == "") {
           print("Reset.");
           delay(5000); // let the watchdog do the reset
        } else 
          addCmd(inputChar);
				break;
      case 's': {
          setup();
          break;
      }
      case 'i': {
          // setup IMU
          imuMgr.powerUp();
          break;
      }
      case 'c': {
          // calibrate IMU
          imuMgr.startHardIronCalibration();
          break;
      }
      case 'C': {
          // calibrate IMU
          imuMgr.startNorthCalibration();
          break;
      }
      case 'I': {
          // setup IMU
          println("power down of IMU.");
        
          imuMgr.powerDown();
          break;
      }
      case 'l': {
          // setup IMU
          println("swap logging");
        
          imuMgr.setLogging(!imuMgr.isLogging());
          break;
      }
      case 'm' :{
        // print power measurement
        powerManager.print();
        break;
      }
      case 'p' :{
        // turn motor power on
        powerManager.powerUp();
        break;
      }
      case 'P' :{
        // turn motor power off
        powerManager.powerDown();
        break;
      }

      case 10:
			case 13:
				if (command.startsWith("c")) {
					unsigned long l = command.substring(1).toInt();
					if (((l >= 0) && (l < odrives.getNumberODrives()*2))) {
            String name = odrives[l/2].getName(l % 2);
            slowWatchdog();
            odrives[l/2].calibrate (l%2);
            fastWatchdog();
					}
					else {
						print("Motor number %lu is out of range", l);
					}
					emptyCmd();
				} else if (command.startsWith("s")) {
          if (command.length() > 1) {
            unsigned long l = command.substring(1).toInt();
            if (((l >= 0) && (l < odrives.getNumberODrives()*2))) {
              String name = odrives[l/2].getName(l % 2);
              slowWatchdog();
              blinker.set(BlockingBlinkPattern, sizeof(BlockingBlinkPattern));
              odrives[l/2].startup (l%2);
              blinker.set(DefaultBlinkPattern, sizeof(DefaultBlinkPattern));
              fastWatchdog();
            }
            else {
              print("Motor number %lu is out of range", l);
            }
          } else {
              slowWatchdog();
              blinker.set(BlockingBlinkPattern, sizeof(BlockingBlinkPattern));
              odrives.startup ();
              blinker.set(DefaultBlinkPattern, sizeof(DefaultBlinkPattern));
              fastWatchdog();
          }
					emptyCmd();
				} else if (command.startsWith("S")) {
          if (command.length() > 1) {
            unsigned long l = command.substring(1).toInt();
            if (((l >= 0) && (l < odrives.getNumberODrives()*2))) {
              String name = odrives[l/2].getName(l % 2);
              println("Stop %s:", name.c_str());
              slowWatchdog();
              odrives[l/2].shutdown (l%2);
              fastWatchdog();
            }
            else {
              print("Motor number %lu is out of range", l);
            }
          } else {
              odrives.shutdown ();
          }
					emptyCmd();
				} else if (command.startsWith("d")) {
            unsigned long l = command.substring(1).toInt();
            if ((l >= 0) && (l < 3)) {
                println("Setting debug Level %ld", l);
                ODrives::debugComm(l>1);
                ODrives::debugAPI(l>0);
                config.debugLevel = l;
                writeConfiguration();
            }
            else {
              print("Debug Level must be 0,1,2, not %ld", l);
            }
					  emptyCmd();
				} else 
        if (command != "") {
          Serial.println("unknown command");
        }

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
  blinker.loop(now_us/1000);

  // react on input from Serial 
  executeCommand();

  // get feedback of all odrives
  // odrives.loop();

  imuMgr.loop();

  // if calibration came to a result, store it in EPPROM
  if (imuMgr.newCalibrationData()) {
    Serial.println("Store calibration in EEPROM");
    imuMgr.getCalibrationData(config.imu);
    config.write(); 
  }

  // wait for any pending commands to be executed
  powerManager.loop();
}