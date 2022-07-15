#include <Arduino.h>
#include "HardwareSerial.h"
#include "MicrostrainComm.h"

HardwareSerial* serial = &Serial8;


union float_int {
  float f;
  unsigned long ul;
};

static bool assert(bool condition, String name) {
  if (!condition) {
    Serial.print("ASSERT:");
    Serial.println(name);
    return false;
  }
  return true;
}

static float parseFloat(CommandData &res) {
  float_int fi;
  fi.ul = ((res.fields[res.field_idx].payload[res.parse_idx+0] << 24) | 
           (res.fields[res.field_idx].payload[res.parse_idx+1] << 16) | 
           (res.fields[res.field_idx].payload[res.parse_idx+2] << 8) | 
           (res.fields[res.field_idx].payload[res.parse_idx+3]));
  res.parse_idx += 4;
  return fi.f;
}

static uint16_t parseU16(CommandData &res) {
  uint16_t value = ((res.fields[res.field_idx].payload[res.parse_idx] << 8) | 
                    (res.fields[res.field_idx].payload[res.parse_idx+1] & 0xFF));
  res.parse_idx += 2;
  return value;
}

static uint8_t parseU8( CommandData &res) {
  uint8_t value =  res.fields[res.field_idx].payload[res.parse_idx];
  res.parse_idx += 1;
  return value;
}

// parse a fixed length string from a response 
static String parseString(CommandData &res, uint8_t len) {
  String s;
  for (int i = 0;i<len;i++) {
    assert(res.field_idx < res.no_fields,  "parseString field_idx exceeded");
    assert(i<res.fields[res.field_idx].len, "parseString i exceeded");

    s.append((char)res.fields[res.field_idx].payload[res.parse_idx+i]);
  }
  res.parse_idx += len;
  return s;
}


uint16_t generateChecksum(uint8_t buffer[], uint8_t buffer_len) {
  uint8_t checksum_byte1 = 0, checksum_byte2 = 0;
  
  for(int i=0; i<buffer_len; i++)
  {
    checksum_byte1 += buffer[i];
    checksum_byte2 += checksum_byte1;
  }
  uint16_t checksum = ((uint16_t) checksum_byte1 << 8) + (uint16_t) checksum_byte2;

  return checksum;
}

void printBuffer(String name, uint8_t buffer[], uint8_t buffer_len) {
  Serial.print(name);
    
  for (int i = 0;i<buffer_len;i++) {
    Serial.print(" 0x");
    Serial.print(buffer[i], HEX);
  }
  Serial.println();
}


void printResponseBuffer(CommandData& res) {
  printBuffer(res.name, res.buffer_res, res.buffer_res_len);
}


void readPrint() {
  uint32_t t = millis() + 200;
  while (millis() < t) {
    if (serial->available()) {
      Serial.print("!0x");
      Serial.print(serial->read(),HEX);
    }
  }
}

void  printErrorCode(uint8_t error_code) {
  if (error_code == 0) {
    Serial.println("ACK");
  }
  else if (error_code == 1) {
    Serial.println("NACK:Unkown Command");
  }
  else if (error_code == 1) {
    Serial.println("NACK:Invalid checksum");
  }
  else if (error_code == 3) {
    Serial.println("NACK:Invalid param");
  }
  else if (error_code == 4) {
    Serial.println("NACK:Command Failed");
  }
  else if (error_code == 5) {
    Serial.println("NACK:Command Timeout");
  }
}

void MicrostrainIMU::createCommand1(uint8_t descriptor_set, 
                   uint8_t field_descriptor_byte, uint8_t field_length, uint8_t field_data[]) {
  uint8_t buffer[BUFFER_SIZE];
  buffer[0] = 0x75;
  buffer[1] = 0x65;
  buffer[2] = descriptor_set;
  buffer[3] = field_length + 2; // plus field descriptor and field len 
  buffer[4] = field_length + 2; // we have only one field, so package len = field length
  buffer[5] = field_descriptor_byte;
  
  for (int i = 0;i<field_length;i++) 
    buffer [6+i] = field_data[i];
  
  uint16_t checksum = generateChecksum(buffer,6+field_length );
  buffer[6+field_length + 0] = checksum >> 8;
  buffer[6+field_length + 1] = checksum & 0xFF;

  uint16_t buffer_len = 8 + field_length;
  assert(buffer_len < BUFFER_SIZE, "createCommand buffer overflow");

  for (int i = 0;i<buffer_len;i++)
    serial->write(buffer[i]);
}

void MicrostrainIMU::createCommand2(uint8_t descriptor_set, 
                   uint8_t field_descriptor_byte1, uint8_t field_length1, uint8_t field_data1[],
                   uint8_t field_descriptor_byte2, uint8_t field_length2, uint8_t field_data2[]) {
  uint8_t buffer[BUFFER_SIZE];

  uint8_t idx = 0;
  buffer[idx++] = 0x75;
  buffer[idx++] = 0x65;
  buffer[idx++] = descriptor_set;
  buffer[idx++] = field_length1 + field_length2 + 2 + 2; // plus field descriptor and field len
  buffer[idx++] = field_length1 + 2;
  buffer[idx++] = field_descriptor_byte1;
  
  for (int i = 0;i<field_length1;i++) 
    buffer [idx++] = field_data1[i];

  buffer[idx++] = field_length2 + 2;
  buffer[idx++] = field_descriptor_byte2;
  
  for (int i = 0;i<field_length2;i++) 
    buffer [idx++] = field_data2[i];

  uint16_t checksum = generateChecksum(buffer, idx);
  buffer[idx++] = checksum >> 8;
  buffer[idx++] = checksum & 0xFF;

  uint16_t buffer_len = 8 + field_length1 + field_length2;
  assert(buffer_len < BUFFER_SIZE, "createCommand buffer overflow");
  for (int i = 0;i<buffer_len;i++)
    serial->write(buffer[i]);

}

bool readResponseChar(CommandData &res){
      if (serial->available()) {
          uint8_t ch = serial->read();

          if (res.buffer_res_idx < 4) {
            res.buffer_res[res.buffer_res_idx++] = ch;
            // wait until we have 4 bytes in the buffer, which is the header of the package
            if (res.buffer_res_idx == 4) {
              // header is complete, check the header and get the package length
              if ((res.buffer_res[0] != 0x75) || (res.buffer_res[1] != 0x65)) {
                // forget the first character, shift the 4 bytes to the left and try again with the next character
                res.buffer_res[0] = res.buffer_res[1];
                res.buffer_res[1] = res.buffer_res[2];
                res.buffer_res[2] = res.buffer_res[3];
                res.buffer_res_idx = 3;
              } else {
                // header is alright, get length and descriptor 
                res.descriptor_set_byte = res.buffer_res[2];
                res.payload_len = res.buffer_res[3];
                res.buffer_res_len = res.payload_len + 6; // 4 byte in header = 4 plus 2 byte checksum 
                res.buffer_res_idx = 4;
              }
            }
          } else {
            res.buffer_res[res.buffer_res_idx++] = ch;
            if ((res.buffer_res_idx > res.buffer_res_len) || (res.buffer_res_idx >= BUFFER_SIZE-1)) {
                println("response buffer overflow.");
                // reset current package and forget this package
                res.buffer_res_idx = 0;
            }
            if (res.buffer_res_idx == res.buffer_res_len) {
              // we have the complete package now.

              // fetch checksum
              uint16_t chk_asis = (((uint16_t)res.buffer_res[4+res.payload_len]) << 8) + ((uint16_t)res.buffer_res[5+res.payload_len]);
              uint16_t chk_tobe = generateChecksum(res.buffer_res, 4+res.payload_len);
              if (chk_asis != chk_tobe) {
                println("checksum error: is %d, but should be %d",chk_asis, chk_tobe);
                res.buffer_res_idx = 0; // reset current package and forget this package
                return false;
              }
              // extract all the fields
              uint8_t field_idx = 0;
              uint8_t copy_idx = 4;
              res.no_fields = 0;

              while (copy_idx <= res.payload_len ) {
                assert(field_idx < MAX_FIELDS, "not enough fields");
                assert(copy_idx+1 < BUFFER_SIZE, "response buffer too short");
                res.fields[field_idx].len =   res.buffer_res[copy_idx];
                res.fields[field_idx].descr = res.buffer_res[copy_idx+1];

                for (int i = 0;i<res.fields[field_idx].len;i++) {
                  assert(field_idx < MAX_FIELDS, "not enough fields");
                  assert(i < FIELD_BUFFER_SIZE, "field buffer too short");
                  res.fields[field_idx].payload[i] = res.buffer_res[copy_idx+i];
                };

                copy_idx += res.fields[field_idx].len;
                field_idx++;
              }
              res.no_fields = field_idx;


              // buffer has been completely transfered to buffer variables, r for the next package
              res.buffer_res_idx = 0;

              // indicate that a full package is available in ResponseParser
              return true;
            }
          } 
      }

      // package hasnt been read completely 
      return false;
}

bool readResponse(CommandData &res){

    // this timeout is for direct commands, not for the datastream
    // these commands might actually take that long
    res.max_timestamp_ms = millis() + 100;

    res.payload_len = 0;
    res.parse_idx = 0;
    res.field_idx = 0;
    res.no_fields = 0;

    bool ok = false;
    while (!ok && (millis() < res.max_timestamp_ms )) {
        ok = readResponseChar(res);

        // is the package completely read?
        if (ok) {
          return true;
        }
    }
    if (!ok) {
      Serial.println("readResponse timeout.");
    }

    return false;
}

bool MicrostrainIMU::expectResponse(){
    bool ok = readResponse(res);
    if (!ok) {
      println("missed expected response %s",res.name.c_str());
    }

    return ok;
}

bool MicrostrainIMU::expectAckNackResponse() {
  bool ok = expectResponse();

  if (ok) {
    res.field_idx = 0;
    res.parse_idx = 2;
    uint8_t cmd_byte = parseU8(res); // command_code
    uint8_t error_code = parseU8(res);
    if (!assert(error_code == 0, "NACK")) {
      println("cmd_byte:%x error_code:%x", cmd_byte, error_code);
      printErrorCode(error_code);
    }
  }
  return ok;
}

bool MicrostrainIMU::sendPing() {
  CommandData res("ping");
  uint8_t field[] = { 0x01, 0x00};
  createCommand1(0x01, 
                 0x01, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

bool MicrostrainIMU::sendSetToIdle() {
  CommandData res("SetToIdle");
  uint8_t field[] = {  };
  createCommand1(0x01, 
                 0x02, sizeof(field), field);
  
  bool ok = expectAckNackResponse();
  
  return ok;
}


bool MicrostrainIMU::sendResumeDevice() {
  CommandData res("ResumeDevice");
  uint8_t field[] = { };
  createCommand1(0x01, 
                 0x06, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

// Send message format defines the format of the data stream coming from the IMU
// chapter 4.2.3 IMU Message format (0x0C, 0x08)
bool MicrostrainIMU::sendSetIMUMessageFormat() {
  CommandData res("SetIMUMessageFormat");
  uint16_t rate_decimation = 1000/targetFreq; // according to data sheet
  const int no_fields = 3; 
  uint8_t field[] = { 0x01,         // function use new settings"
                      no_fields,    // 3 fields
                      0x04, uint8_t(rate_decimation >> 8) , (uint8_t)(rate_decimation & 0xFF) ,    // Scaled Acc, chapter 5.1.1 
                      0x05, uint8_t(rate_decimation >> 8) , (uint8_t)(rate_decimation & 0xFF) ,    // Scaled Gyro, chapter 5.1.2
                      0x08, uint8_t(rate_decimation >> 8) , (uint8_t)(rate_decimation & 0xFF) ,    // Delta velocity, chapter 5.1.4
                      // 0x0C, uint8_t(rate >> 8) , (uint8_t)(rate & 0xFF),     // Euler Angles
                      // 0x07, uint8_t(rate >> 8) , (uint8_t)(rate & 0xFF),     // Delta Theta vector (integrated angular rate in x,y,z [RAD]
                      // 0x0A, uint8_t(rate >> 8) , (uint8_t)(rate & 0xFF),      // Quaternion*/
  };


  createCommand1(0x0C, 
                 0x08, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

// save the raw and the EF format 
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 18. "4. Save the IMU and Estimation Filter MIP Message Format" 
bool MicrostrainIMU::sendSaveFormat() {
  CommandData res("saveFormat");
  uint8_t field[] = { 0x03,0x00}; //  Save Current IMU Message Format

  createCommand1(0x0C, 
                 0x08, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Chapter 4.2.4, "Enable/Disable Continous Data Stream"
bool MicrostrainIMU::sendEnableDataStream(bool enable) {
  CommandData res("EnableDataStream");
  uint8_t field[] = { 0x01,0x01, (uint8_t)(enable==true?0x01:0x00)};    // Enable Continuous IMU Message 

  createCommand1(0x0C, 
                0x11, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

bool MicrostrainIMU::sendSetHeading() {
  CommandData res("SetHeading");
  uint8_t field[] = { 0x00,0x00, 0x00, 0x00};
  createCommand1(0x0D, 
                0x03, sizeof(field), field);
  bool ok = expectAckNackResponse();
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 43. "4.1.9 Device Reset (0x01, 0x7E)"
bool MicrostrainIMU::sendResetDevice() {
  CommandData res("ResetHDevice");
  uint8_t field1[] = { };

  createCommand1(0x01, 
                0x7E, sizeof(field1), field1);
  bool ok = expectAckNackResponse();
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 34. "4.1.3 Get Device Information (0x01, 0x03)"
void MicrostrainIMU::sendGetDeviceInformation() {
  CommandData res("GetDeviceInformation");
  uint8_t field[] = { };

  createCommand1(0x01, 
                0x03, sizeof(field), field);
  // printCmdBuffer(res);

  bool ok = readResponse(res);

  if (!assert(ok != 0, "response invalid")) return;

  if (!assert((res.no_fields >= 1) && (res.fields[0].descr == 0xF1), "1st field descriptor wrong")) return;
  if (!assert((res.no_fields >= 2) && (res.fields[1].descr == 0x81), "2nd field descriptor wrong")) return;

  // parse first package
  res.field_idx = 0;
  res.parse_idx = 2; // ignore field length & field descr
  uint8_t cmd_code = parseU8(res); // command_code
  uint8_t error_code = parseU8(res);
  // println("ccmd_code %d error_code %d", cmd_code,error_code);
  if (!assert(error_code == 0, "NACK")) {
    println("cmd_byte = %x error_code = %x", cmd_code, error_code);
    printErrorCode(error_code);
    return;
  }
  // parse second package
  res.field_idx = 1;
  res.parse_idx = 2; // ignore field length & field descr
  uint16_t firmware_version = parseU16(res);
  String model_name =         parseString(res, 16);
  String model_number =       parseString(res, 16);
  String serial_string =      parseString(res, 16);
  String reserved_string =    parseString(res, 16);
  String options =            parseString(res, 16);
  println("IMU %s", model_name.c_str());
  Serial.print("   firmware      :             ");
  Serial.println(firmware_version);
  Serial.print("   serial        : ");
  Serial.println(serial_string);
  Serial.print("   options       : ");
  Serial.println(options);
}


// change the baud rate
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 62 "4.2.15 UART Baud Rate (0x0C, 0x40)"
bool MicrostrainIMU::sendChangeBaudRate(uint32_t baud) {
  CommandData res("ChangeBaudRate");
  uint8_t field[] = { 0x01,                    // 01 = change the baud rate!  
                       (uint8_t)(baud >> 24), (uint8_t)((baud >> 16) & 0xFF), (uint8_t)((baud >> 8) & 0xFF), (uint8_t)((baud >> 0) & 0xFF)   
                     };

  createCommand1(0x0C, 
                0x40, sizeof(field), field);

  bool ok = readResponse( res);
  if (!assert(ok != 0, "response invalid")) return false;
  if (!assert((res.no_fields >= 0) && (res.fields[0].descr == 0xF1), "field descriptor wrong")) return false;

  // parse first package
  parseU8(res); // command_code
  uint8_t error_code = parseU8(res);
  if (!assert(error_code != 0, "NACK")) {
    printErrorCode(error_code);
    return false;
  }

  // wait 250ms until the new baud rate takes place
  delay(300);

  // next call is gonne be with the new rate
  serial->flush();
  serial->end();
  serial->begin(baud);
  return true;
}


void MicrostrainIMU::clearBuffer() {        
  serial->flush();
  while (serial->available())
    serial->read();
}


bool MicrostrainIMU::setup(HardwareSerial* sn, uint16_t sampleFreq) {
  serial = sn;

  // sample rate of the outgoing data stream
  targetFreq = sampleFreq;

  // check if device is responding
  // println("ping");
   bool ok = true;
   // ok= sendPing();
   if (!ok)
    return false;

  // if IMU is data stream for any reasons (maybe it has been turned on beforehand)
  // then try to stop that first. Otherwise the stream messes up the responses of the configuration
  // We need to try that a couple of times until the call sneaks in the middle of the  data stream
  baud_rate = 912600;
  serial->begin(baud_rate);
  ok = false;
  /*
  // setting idle disables the datastream
  ok = sendSetToIdle();
  if (!ok) {
    println("Could not set to idle");
    return false;
  }
 
  println("IMU: get device information");
  sendGetDeviceInformation();
  if (!ok)
    return false;

  println("IMU: set message format");
  ok = sendSetIMUMessageFormat();
  if (!ok)
    return false;

  println("IMU: enable data stream");
  ok = sendEnableDataStream(true);
   if (!ok)
    return false;
  */

  const int no_fields = 3;
  println("baud rate   : %d baud", baud_rate);
  println("frequency   : %d Hz", targetFreq);
  println("packetsize  : %d Byte", 16*no_fields);
  println("min baud    : %d baud", no_fields*16*targetFreq*10);

  // read response from scratch;
  res.buffer_res_idx = 0;
  is_initialised = true;
  last_data_package_ts = 0;

  return true;
}

bool isModified(IMUSensorData &imu_data) {
  return (imu_data.acc_modified || imu_data.gyro_modified || imu_data.delta_theta_modified || imu_data.quat_modified || imu_data.rpy_modified || imu_data.delta_velocity_modified);
}

void MicrostrainIMU::loop() {
    if (!is_initialised) {
      return;
    }

    // read and process characters from Serial as long as possible, but dont wait if the queue is empty
    bool fullPackageAvailable = readResponseChar(res);
    // a full package as been read 
    if (fullPackageAvailable) {
        last_data_package_ts = micros();
        // parse the package
        for (res.field_idx = 0;res.field_idx < res.no_fields;res.field_idx++) {
          res.parse_idx = 2;
          // Scaled Accelerometer Vector [g]
          if ((res.descriptor_set_byte == 0x80)) {
            switch (res.fields[res.field_idx].descr) {
              case 0x04:  // Scaled Accelerometer Vector (0x80, 0x04)
                imu_data.acc_x = parseFloat(res);
                imu_data.acc_y = parseFloat(res);
                imu_data.acc_z = parseFloat(res);
                imu_data.acc_modified  = true;
                break;
              case 0x05:  // Scaled Gyro Vector (0x80, 0x05)
                imu_data.gyro_x = parseFloat(res);
                imu_data.gyro_y = parseFloat(res);
                imu_data.gyro_z = parseFloat(res);
                imu_data.gyro_modified  = true;
                break;
              case 0x08:  // Delta Velocity Vector (0x80, 0x07) in [g/s2]
                imu_data.delta_velocity_x = parseFloat(res) ; 
                imu_data.delta_velocity_y = parseFloat(res) ;
                imu_data.delta_velocity_z = parseFloat(res) ;
                imu_data.delta_velocity_modified  = true;
                break;
              case 0x0A:  // Quaternion w, q1, q2, q3
                imu_data.quat_w   = parseFloat(res);
                imu_data.quat_q1   = parseFloat(res);
                imu_data.quat_q2   = parseFloat(res);
                imu_data.quat_q3   = parseFloat(res);
                imu_data.quat_modified  = true;
                break;
              case 0x0C:  // Euler Angles (0x80, 0x0C)
                imu_data.roll   = parseFloat(res);
                imu_data.pitch  = parseFloat(res);
                imu_data.yaw    = parseFloat(res);
                imu_data.rpy_modified  = true;
                break;
              default:
                println("unknown field 0x%.2x.", res.fields[res.field_idx].descr);
            }
          }
          else {
                println("unknown descriptor 0x%.2x.", res.descriptor_set_byte);
          }
        }
        dataStreamClock.tick();
    } else {
      // we only have 1 ms to receive an answer
      // After 2ms we escalate 
      uint32_t time_since_last_package_us = micros() - last_data_package_ts;  
      if ((last_data_package_ts > 0) && (time_since_last_package_us > 5*1000000/targetFreq)) {
          println("missing data stream for %dus, recovery procedure", time_since_last_package_us);
          serial->flush();
          serial->begin(baud_rate);
          if (time_since_last_package_us > 100*1000000/targetFreq) {
              // is_initialised = false leads to a new setup in IMUManager
              is_initialised = false;
              last_data_package_ts = 0;
          }
          
      }
    }
}

Measurement& MicrostrainIMU::getMeasuremt() {
  return dataStreamClock;
}
bool MicrostrainIMU::isNewPackageAvailable() {
  bool m = isModified(imu_data);
  imu_data.delta_theta_modified = false;
  imu_data.acc_modified = false;
  imu_data.gyro_modified = false;
  imu_data.quat_modified = false;
  imu_data.rpy_modified = false;
  imu_data.delta_velocity_modified = false;
  
  return m;
} 

void MicrostrainIMU::printData() {
  /*
  print("\r\nIMU \r\n   acc  :(%.2f %.2f %.2f)\n\r   Gyro :(%.2f %.2f %.2f)\r\n   dVel  :(%.3f %.3f %.3f)\r\n",
         imu_data.acc_x,
         imu_data.acc_y,
         imu_data.acc_z,
         imu_data.gyro_x,
         imu_data.gyro_y,
         imu_data.gyro_z,
         imu_data.delta_velocity_x,
         imu_data.delta_velocity_y,
         imu_data.delta_velocity_z);
         */
  print("\r\nIMU \r\n   acc  :(%.2f %.2f %.2f)\n\r   Gyro :(%.2f %.2f %.2f)\r\n",
         imu_data.acc_x,
         imu_data.acc_y,
         imu_data.acc_z,
         imu_data.gyro_x,
         imu_data.gyro_y,
         imu_data.gyro_z);
}
 
