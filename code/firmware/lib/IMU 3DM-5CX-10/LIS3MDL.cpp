#include <LIS3MDL.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(void)
{
  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool LIS3MDL::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LIS3MDL::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LIS3MDL::getTimeout()
{
  return io_timeout;
}

bool LIS3MDL::init()
{
    if (testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        address =  LIS3MDL_SA1_HIGH_ADDRESS ;
      }
      else {
         address =  LIS3MDL_SA1_LOW_ADDRESS ;
    }
  return true;
}

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL::setup(dataRate_t dataRate, range_t dataRange)
{
    init();

    range = dataRange;
    uint8_t temperature_enable = 0;

    uint8_t self_test_enable = 0;  
    uint8_t perf_mode = 0;
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

    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    // writeReg(CTRL_REG1, 0x70);
    writeReg(CTRL_REG1, (temperature_enable << 7) | (dataRate << 1) | (perf_mode << 5) | self_test_enable);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    // writeReg(CTRL_REG2, 0x00 );
    writeReg(CTRL_REG2, range << 5);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeReg(CTRL_REG3, CONTINUOUSMODE);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    // writeReg(CTRL_REG4, 0x0C);
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
    writeReg(LIS3MDL_REG_INT_CFG, value);

    
  rangeScale = 1; // LSB per gauss
  if (range == RANGE_16_GAUSS)
    rangeScale = 1711;
  if (range == RANGE_12_GAUSS)
    rangeScale = 2281;
  if (range == RANGE_8_GAUSS)
    rangeScale = 3421;
  if (range == RANGE_4_GAUSS)
    rangeScale = 6842;
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
void LIS3MDL::requestData()
{
  Wire1.beginTransmission(address);
  // assert MSB to enable subaddress updating
  Wire1.write(OUT_X_L | 0x80);
  Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t)6);
}

// Reads the 3 mag channels and stores them in vector m
void  LIS3MDL::read(double  &mag_x, double &mag_y, double &mag_z )
{

  uint16_t millis_start = millis();
  while (Wire1.available() < 6)
  {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xlm = Wire1.read();
  uint8_t xhm = Wire1.read();
  uint8_t ylm = Wire1.read();
  uint8_t yhm = Wire1.read();
  uint8_t zlm = Wire1.read();
  uint8_t zhm = Wire1.read();

  // combine high and low bytes
  double x = (int16_t)(xhm << 8 | xlm);
  double y = (int16_t)(yhm << 8 | ylm);
  double z = (int16_t)(zhm << 8 | zlm);


  mag_x = x / rangeScale;
  mag_x = y / rangeScale;
  mag_x = z / rangeScale;

}

void LIS3MDL::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
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
