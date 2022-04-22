/**
 * Manages configuration data within EPPROM
 */ 
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <version.h>
#include <EEPROM.h>

// This is the configuration memory block that is stored in EEPROM in a "bank". It contains a write_counter for counting the number of 
// write operations determining the right time when to switch the bank.
struct configuration_type {
	uint16_t write_counter;				// counts the write operation to change the EPPROM bank when overflows

    /** block with application configuration data */ 
	unsigned long counter;

	// initialize all configuration values to factory settings
	void setup() {
		write_counter = 0;
		counter = 1;					// now_us [us] between two images. In normal operations, anything between 1000000/3 fps and 100000/20 fps is allowed
	}
	
	void write() ;
	void read() ;
	void writeByte(uint16_t no_of_byte);
};

extern configuration_type config;

void readConfiguration();
void setupConfiguration();
void writeConfiguration();
void resetConfiguration();

#endif