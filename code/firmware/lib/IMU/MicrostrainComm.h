/*************************************************************************************************************
 * class for communicating with a LORD Parker Microstrain IMU.
 *
 * Implements the specific communication protocol as specificed in 
 * https://github.com/jochenalt/Lisbeth/blob/main/datasheets/Microstrain%203DM-CV5-IMU/3DM-CV5-10%20IMU%20Data%20Communication%20Protocol%20Manualpdf.pdf
 * 
 * Uses serial communication with hardcoded 921600 baud (because the 1000Hz sample frequency requires that)
 * Assumes that the IMU has a certain startup configuration, so no configuration is done during startup.
 * 
 ************************************************************************************************************/


#ifndef MICROSTRAIN_H_
#define MICROSTRAIN_H_

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>

#define BUFFER_SIZE 256
#define FIELD_BUFFER_SIZE 128
#define MAX_FIELDS 5

struct IMUSensorData
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

  float delta_velocity_x;
  float delta_velocity_y;
  float delta_velocity_z;

  float quat_w;
  float quat_q1;
  float quat_q2;
  float quat_q3;

  bool delta_theta_modified = false;
  bool acc_modified = false;
  bool gyro_modified = false;
  bool quat_modified = false;
  bool rpy_modified = false;
  bool delta_velocity_modified = false;
  bool imu_modified = false;
};


// Type that implements a field in the communication protocol, which is 
// the basic data type in the Microstrain protocol.
struct FieldType {
   uint8_t descr;
   uint8_t len;
   uint8_t payload[FIELD_BUFFER_SIZE];
};  

// data structure with buffers for outgoing commands and incoming data from the IMU
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

  uint32_t max_timestamp_ms = 0;

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


class MicrostrainIMU {
    public:
        MicrostrainIMU(): 
          is_initialised(false) ,
          baud_rate(460800) {}; 
        virtual ~MicrostrainIMU() { };
        bool setup(HardwareSerial* serial, uint16_t sampleFreq);

        bool isInitialised() { return is_initialised; };
        void loop();

        // returns true, if the IMU sent a new package. 
        // Resets the flag internally, so next call is return false until the next IMU package arrived.
        bool isNewPackageAvailable();

        // return the latest data as coming from IMU
        IMUSensorData& getIMUSensorData() { return imu_data; };

        // get some timeing PerfMeaurements
        Measurement& getMeasuremt();

        // direct communication with the IMU
        void sendGetDeviceInformation();
        bool sendPing();
        bool sendSetToIdle();
        bool sendResumeDevice();
        bool sendSetIMUMessageFormat();
        bool sendSaveFormat();
        bool sendSetHeading();
        bool sendResetDevice();
        bool sendEnableDataStream(bool ok);
        bool sendChangeBaudRate(uint32_t baud);

        void createCommand1(uint8_t descriptor_set,uint8_t field_descriptor_byte, uint8_t field_length, uint8_t field_data[]);
        void createCommand2(uint8_t descriptor_set,uint8_t field_descriptor_byte1, uint8_t field_length1, uint8_t field_data1[],uint8_t field_descriptor_byte2, uint8_t field_length2, uint8_t field_data2[]);
        bool expectResponse();
        bool expectAckNackResponse();


        // log last measurement
        void printData();

    private:
        void clearBuffer();
        IMUSensorData imu_data;                           // struct of data returned by IMU
        CommandData res;

        bool is_initialised;
        uint32_t baud_rate;
        uint16_t targetFreq = 1000;
        Measurement dataStreamClock;
        uint32_t last_data_package_ts = 0;                   // time of the last data package arrived
        uint32_t data_package_size = 0;
        uint32_t setup_time_ms = 0;
}; 


#endif


