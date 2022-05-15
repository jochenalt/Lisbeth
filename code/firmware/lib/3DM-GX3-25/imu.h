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
        IMU() {   is_initialised = false; baud_rate = 115200;};
        virtual ~IMU() { };
        void setup(HardwareSerial* serial);
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

int imu_init();
void loopImu();
int parse_IMU_data();
void print_imu();

uint16_t get_acc_x_in_D16QN();
uint16_t get_acc_y_in_D16QN();
uint16_t get_acc_z_in_D16QN();

uint16_t get_gyr_x_in_D16QN();
uint16_t get_gyr_y_in_D16QN();
uint16_t get_gyr_z_in_D16QN();

uint16_t get_roll_in_D16QN();
uint16_t get_pitch_in_D16QN();
uint16_t get_yaw_in_D16QN();

uint16_t get_linacc_x_in_D16QN();
uint16_t get_linacc_y_in_D16QN();
uint16_t get_linacc_z_in_D16QN();

#define ACCEL_ANGRATE_ORIENT 0xC8
#define LENGTH_ACCEL_ANGRATE_ORIENT 67

#define ACC_ANG_MAG 0xCB
#define LENGTH_ACC_ANG_MAG 43

#define ACC_STAB (0xD2)
#define LENGTH_ACC_STAB (43)

#define DANG_DVEL_MAG 0xD3
#define LENGTH_DANG_DVEL_MAG 43

#define ACC_ANG_MAG_ROT 0xCC
#define LENGTH_ACC_ANG_MAG_ROT 79

#define CONTINUOUS_MODE_COMMAND 0xC4
#define LENGTH_CONTINUOUS_MODE 4
#define LENGTH_CONTINUOUS_MODE_ECHO 8

#define COMMS_SETTINGS_COMMAND 0xD9
#define LENGTH_COMMS_SETTINGS 11
#define LENGTH_COMMS_SETTINGS_ECHO 10

#define SAMPLING_SETTINGS_COMMAND 0xDB
#define LENGTH_SAMPLING_SETTINGS 20
#define LENGTH_SAMPLING_SETTINGS_ECHO 19

#define DATA_RATE_DEFAULT 100
#define DATA_RATE_MED 500
#define DATA_RATE_HIGH 1000

#define BAUD_RATE_DEFAULT 115200
#define BAUD_RATE_MED 460800
#define BAUD_RATE_HIGH 921600

#define DELTA_ANG_VEL_DT_DEFAULT 0.010
#define DELTA_ANG_VEL_DT_MED 0.002
#define DELTA_ANG_VEL_DT_HIGH 0.001

#define FILTER_WINDOW_SIZE_DEFAULT 15
#define FILTER_WINDOW_SIZE_MIN 1
#define FILTER_WINDOW_SIZE_MAX 32

#define GRAVITY 9.80665
#define INPUT_BUFFER_SIZE 2000

#endif


