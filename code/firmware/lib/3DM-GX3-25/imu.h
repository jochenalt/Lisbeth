#ifndef UART_IMU_H
#define UART_IMU_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define BUFFER_SIZE 256
#define FIELD_BUFFER_SIZE 128
#define MAX_FIELDS 5

struct ImuData
{
  // raw data
  float acc_x;
  float acc_y;
  float acc_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float roll;
  float pitch;
  float yaw;

  float delta_theta_x;
  float delta_theta_y;
  float delta_theta_z;

  float quat_w;
  float quat_q1;
  float quat_q2;
  float quat_q3;

  bool delta_theta_modified = false;
  bool acc_modified = false;
  bool gyro_modified = false;
  bool quat_modified = false;
  bool rpy_modified = false;
};


class Measurement {
  public:
    Measurement() {
        start_time = micros();
        end_time = start_time;
        duration = 0;
        duration_avr = 0;
    };
    virtual ~Measurement() {};

    void start() { 
        start_time = micros();
    }
    void stop()  {
       end_time = micros(); 
       duration = end_time - start_time;
      duration_avr = (duration_avr + duration)/2;
    }
    float getTime() { return ((float)duration)/1000000.0; };
    float getAvrTime() { return ((float)duration)/1000000.0; };
    float getAvrFreq() { 
      if (duration_avr > 0)
        return 1000000.0/(float)duration_avr;
      else
        return 0;
     }
    void tick() {
      stop();
      start();
    } 

  private:
    uint32_t start_time;
    uint32_t end_time;
    uint32_t duration;
    uint32_t duration_avr;
};


struct FieldType {
   uint8_t descr;
   uint8_t len;
   uint8_t payload[FIELD_BUFFER_SIZE];
};  

class CommandData {
  public:
    CommandData() {
      max_timestamp_ms = millis() + 100;
    }
    CommandData(String s) {
      name = s;
      max_timestamp_ms = millis() + 100;
    }
    ~CommandData() {};
  String name;
  // buffer for constructing a command
  uint8_t buffer_cmd [BUFFER_SIZE];
  uint8_t buffer_cmd_len ;

  uint16_t max_timestamp_ms = millis() + 100;

  // buffer of response
  uint8_t buffer_res[BUFFER_SIZE];
  uint8_t buffer_res_len = 0;
  uint8_t buffer_res_idx = 0;         // current index when reading the response

  // header information
  uint8_t payload_len = 0;            // payload of package coming from header = total length - 6
  uint8_t descriptor_set_byte = 0;    // description of this package

  // all fields of a reponse
  FieldType fields[MAX_FIELDS];
  uint8_t no_fields = 0;            // number of fields 
  uint8_t field_idx = 0;            // current index of the considered field
  uint8_t parse_idx = 0;            // current index while parsing a field 

  uint32_t check = 010565;
};


class IMU {
    public:
        IMU() {  is_initialised = false; baud_rate = 115200;};
        virtual ~IMU() { };
        bool setup(HardwareSerial* serial);
        bool isInitialised() { return is_initialised; };
        void loop();

        // returns true, if the IMU sent a new package. 
        // Resets the flag internally, so next call is return false until the next IMU package arrived.
        bool isNewPackageAvailable();
        ImuData& getIMUData() { return imu_data; };
        Measurement& getMeasuremt();

        void printData();
    private:
        void clearBuffer();
        ImuData imu_data;                           // struct of data returned by IMU
        CommandData res;
        bool is_initialised;
        uint32_t baud_rate;
}; 


#endif


