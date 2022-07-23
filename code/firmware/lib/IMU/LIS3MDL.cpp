
#include <Arduino.h>
#include <LIS3MDL.h>
#include <Wire.h>
#include <math.h>


// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0x1C
#define LIS3MDL_SA1_LOW_ADDRESS   0x1E
#define TEST_REG_ERROR -1
#define LIS3MDL_WHO_ID  0x3D

LIS3MDL::LIS3MDL()
{
}

bool  LIS3MDL::setup(dataRate_t dataRate, range_t dataRange)
{
  Wire1.begin();

  if (testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
  {
      address =  LIS3MDL_SA1_HIGH_ADDRESS ;
  }
  else {
         if (testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
             address =  LIS3MDL_SA1_LOW_ADDRESS ;
         else {
             Serial.println("LIS3MDL sensor not found on I2C. Is the magnetometer connected?");
             return false;
         }
  }

  range = dataRange;
  uint8_t temperature_enable = 0;
  uint8_t self_test_enable = 0;  
  uint8_t perf_mode = LOWPOWERMODE;
  if (dataRate == DATARATE_155_HZ) {
    // set OP to UHP
    perf_mode = ULTRAHIGHMODE;
  }
  if (dataRate == DATARATE_300_HZ) {
    // set OP to HP
    perf_mode = HIGHMODE;
  }
  if (dataRate == DATARATE_560_HZ) {
    // set OP to MP
    perf_mode = MEDIUMMODE;
  }
  if (dataRate == DATARATE_1000_HZ) {
    // set OP to LP
    perf_mode = LOWPOWERMODE;
  }

    writeReg(CTRL_REG1, (temperature_enable << 7)  | (perf_mode << 5) | (dataRate << 1) | self_test_enable);
    writeReg(CTRL_REG2, range << 5);
    writeReg(CTRL_REG3, CONTINUOUSMODE);
    writeReg(CTRL_REG4, perf_mode << 2);

    bool enableX = false;
    bool enableY = false;
    bool enableZ = true;
    bool polarity = false;
    bool latch = false;
    bool enableInt = true;
    uint8_t value = 0x08; // set default bits, see table 36
    value |= enableX << 7;
    value |= enableY << 6;
    value |= enableZ << 5;
    value |= polarity << 2;
    value |= latch << 1;
    value |= enableInt;
    writeReg(INT_CFG, value);

    
  rangeScale = 1; // LSB per gauss
  if (range == RANGE_16_GAUSS)
    rangeScale = 1.0/1711.0;
  if (range == RANGE_12_GAUSS)
    rangeScale = 1.0/2281.0;
  if (range == RANGE_8_GAUSS)
    rangeScale = 1.0/3421.0;
  if (range == RANGE_4_GAUSS)
    rangeScale = 1.0/6842.0;
  
  // convert from [gauss] to [uT]
  rangeScale *= 100.0;
  
  return true; // all good
}

// Writes a mag register
void LIS3MDL::writeReg(uint8_t reg, uint8_t value)
{
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  last_status = Wire1.endTransmission();
}

// Reads a mag register
uint8_t LIS3MDL::readReg(uint8_t reg)
{
  uint8_t value;

  Wire1.beginTransmission(address);
  Wire1.write(reg);
  last_status = Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t)1);
  value = Wire1.read();
  Wire1.endTransmission();

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void LIS3MDL::readSync(double  &mag_x, double &mag_y, double &mag_z )
{
  Wire1.beginTransmission(address);
  // assert MSB to enable subaddress updating
  Wire1.write(OUT_X_L | 0x80);
  Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t)6);

  while (Wire1.available() < 6)
  {
    delay(1);
  }

  readResponse (mag_x,mag_y,mag_z);
}

// request data, dont fetch it yet, this is done in readResponse
void LIS3MDL::requestData()
{
  Wire1.beginTransmission(address);
  // assert MSB to enable subaddress updating
  Wire1.write(OUT_X_L | 0x80);
  Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t)6);
}

// Reads the 3 mag channels and stores them in vector m
bool   LIS3MDL::isDataAvailable() {
  return Wire1.available() >= 6;
}

void  LIS3MDL::readResponse(double  &mag_x, double &mag_y, double &mag_z )
{
  uint8_t xlm = Wire1.read();
  uint8_t xhm = Wire1.read();
  uint8_t ylm = Wire1.read();
  uint8_t yhm = Wire1.read();
  uint8_t zlm = Wire1.read();
  uint8_t zhm = Wire1.read();

  // combine high and low bytes
  mag_x = (int16_t)(xhm << 8 | xlm);
  mag_y = (int16_t)(yhm << 8 | ylm);
  mag_z = (int16_t)(zhm << 8 | zlm);
  
  mag_x *= rangeScale;
  mag_y *= rangeScale;
  mag_z *= rangeScale;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LIS3MDL::testReg(uint8_t address, regAddr reg)
{
  Wire1.beginTransmission(address);
  Wire1.write((uint8_t)reg);
  if (Wire1.endTransmission() != 0)
  {  
    return TEST_REG_ERROR;
  }

  Wire1.requestFrom(address, (uint8_t)1);
  if (Wire1.available())
  {
    return Wire1.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}
