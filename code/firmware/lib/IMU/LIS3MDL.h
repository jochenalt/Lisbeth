#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <Arduino.h>



/** The magnetometer ranges */
typedef enum {
  RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
  RANGE_8_GAUSS = 0b01,  ///< +/- 8g
  RANGE_12_GAUSS = 0b10, ///< +/- 12g
  RANGE_16_GAUSS = 0b11, ///< +/- 16g
} range_t;

/** The magnetometer data rate, includes FAST_ODR bit */
typedef enum {
  DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
  DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
  DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
  DATARATE_5_HZ = 0b0110,     ///<  5 Hz
  DATARATE_10_HZ = 0b1000,    ///<  10 Hz
  DATARATE_20_HZ = 0b1010,    ///<  20 Hz
  DATARATE_40_HZ = 0b1100,    ///<  40 Hz
  DATARATE_80_HZ = 0b1110,    ///<  80 Hz
  DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
  DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
  DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
  DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
} dataRate_t;

/** The magnetometer performance mode */
typedef enum {
  LOWPOWERMODE = 0b00,  ///< Low power mode
  MEDIUMMODE = 0b01,    ///< Medium performance mode
  HIGHMODE = 0b10,      ///< High performance mode
  ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
} performancemode_t;

/** The magnetometer operation mode */
typedef enum {
  CONTINUOUSMODE = 0b00, ///< Continuous conversion
  SINGLEMODE = 0b01,     ///< Single-shot conversion
  POWERDOWNMODE = 0b11,  ///< Powered-down mode
} operationmode_t;


class LIS3MDL
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };
    // register addresses
    enum regAddr
    {
      WHO_AM_I    = 0x0F,

      CTRL_REG1   = 0x20,
      CTRL_REG2   = 0x21,
      CTRL_REG3   = 0x22,
      CTRL_REG4   = 0x23,
      CTRL_REG5   = 0x24,

      STATUS_REG  = 0x27,
      OUT_X_L     = 0x28,
      OUT_X_H     = 0x29,
      OUT_Y_L     = 0x2A,
      OUT_Y_H     = 0x2B,
      OUT_Z_L     = 0x2C,
      OUT_Z_H     = 0x2D,
      TEMP_OUT_L  = 0x2E,
      TEMP_OUT_H  = 0x2F,
      INT_CFG     = 0x30,
      INT_SRC     = 0x31,
      INT_THS_L   = 0x32,
      INT_THS_H   = 0x33,
    };

    vector<int16_t> m; // magnetometer readings

    uint8_t last_status; // status of last I2C transmission

    LIS3MDL();

    bool init();

    bool setup(dataRate_t dataRate,range_t dataRange);

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    // true if the hardware buffer holds the next data point
    bool isDataAvailable();
    void requestData();
    void readSync(double &x, double &y, double &z);

    // read data in [gauss]
    void readResponse(double &x, double &y, double &z);

    void setTimeout(uint16_t timeout);
    uint16_t getTimeout(void);
    bool timeoutOccurred(void);
 
  private:
    uint8_t address;

    uint16_t io_timeout;
    bool did_timeout;
    range_t range;
    double rangeScale;

    int16_t testReg(uint8_t address, regAddr reg);
};

#endif



