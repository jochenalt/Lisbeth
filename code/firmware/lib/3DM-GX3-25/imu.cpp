#include <Arduino.h>
#include "HardwareSerial.h"
#include "imu.h"

HardwareSerial* serial = &Serial8;


union float_int {
  float f;
  unsigned long ul;
};


/*
union double_long {
  double f;
  uint8_t buffer[8];
};

static double parseDouble(ResponseParser &res) {
  double_long dl;
  for (int i = 0;i<8;i++)
    dl.buffer[i] = res.fields[res.field_idx].payload[res.parse_idx+i];
  res.parse_idx += 8;
  return dl.f;
}
*/
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


#define FLOAT_FROM_BYTE_ARRAY(buff, n) ((buff[n] << 24) | (buff[n + 1] << 16) | (buff[n + 2] << 8) | (buff[n + 3]));

static void print(const char* format, ...) {
	char s[256];
	__gnuc_va_list  args;
		  
	va_start (args, format);
	vsprintf (s, format, args);
	va_end (args);		
    Serial.print(s);
}

static void println(const char* format, ...) {
	char s[256];
	__gnuc_va_list  args;
		  
	va_start (args, format);
	vsprintf (s, format, args);
	va_end (args);		
    Serial.println(s);
};

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define BUF_SIZE 100

//possition of the data in the IMU message
#define ACCX_POS 6
#define ACCY_POS 10
#define ACCZ_POS 14
#define GYRX_POS 20
#define GYRY_POS 24
#define GYRZ_POS 28

//possition of the data in the EF message
#define EFR_POS 6
#define EFP_POS 10
#define EFY_POS 14
#define EFLINACCX_POS 22
#define EFLINACCY_POS 26
#define EFLINACCZ_POS 30
#define FLOAT_FROM_BYTE_ARRAY(buff, n) ((buff[n] << 24) | (buff[n + 1] << 16) | (buff[n + 2] << 8) | (buff[n + 3]));

/* Qvalues for each fields */
#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 13

struct strcut_imu_data
{
  union float_int acc_x;
  union float_int acc_y;
  union float_int acc_z;
  union float_int gyr_x;
  union float_int gyr_y;
  union float_int gyr_z;
  union float_int roll;
  union float_int pitch;
  union float_int yaw;
  union float_int linacc_x;
  union float_int linacc_y;
  union float_int linacc_z;
};
static struct strcut_imu_data imu = {0};

// Receive buffer to collect incoming data
// Here we use a ring buffer to protect the read/write memmory access.
uint8_t rxbuf[256];        //default buffer
uint8_t rxbuf_imu[3][256]; //buffer for  IMU packets
uint8_t rxbuf_ef[3][256];  //buffer for estimation filter packets

int intr_cpt = 0;
uint8_t read_index_imu = 0; // where to read the latest updated imu data
uint8_t read_index_ef = 0;  // where to read the latest updated ef data

uint8_t headerbuffer[4];
uint8_t headerPos = 0;


// Define UART interrupt subroutine to ackowledge interrupt
void readPackage()
{
  uint16_t rx_fifo_len;
  uint16_t i = 0;
  uint8_t header[4];
  uint8_t *buffer_ptr;

  rx_fifo_len = serial->available(); // read number of bytes in UART buffer
  if (rx_fifo_len > 0)
    print("len=%d ",rx_fifo_len);
  while (rx_fifo_len > 4) //While there is at least 4 bytes to read (the header size)
  {
    intr_cpt ++;
    //read header (4bytes) [0x75 - 0x65 - descriptor - payload_len]
    for (int j = 0; j < 4; j++)
    {
      header[j] = serial->read();
      rx_fifo_len--;
    }
    //point to the corresponding packet buffer.
    switch (header[2])
    {
    case (0x80):
      read_index_imu = ((read_index_imu + 1) % 3); //position of the new valid data after this ISR
      buffer_ptr = rxbuf_imu[read_index_imu];
      break;
    case (0x82):
      read_index_ef = ((read_index_ef + 1) % 3); //position of the new valid data after this ISR
      buffer_ptr = rxbuf_ef[read_index_ef];
      break;
    default:
      buffer_ptr = rxbuf;
    }
    //copy the header
    for (int j = 0; j < 4; j++)
    {
      buffer_ptr[j] = header[j];
    }
    int len = header[3] + 2;
    if (len > rx_fifo_len)
    {
      break; // The message is too short.
    }
    //copy the rest of the packet
    for (i = 0; i < len; i++)
    {
      buffer_ptr[i + 4] = serial->read();
      rx_fifo_len--;
    }
  }
  //Flush the UART fifo? is it usefull?
  while ((serial->available() > 0) && rx_fifo_len)
  {
    serial->read();
    rx_fifo_len--;
  }
}

inline bool check_IMU_CRC(unsigned char *data, int len)
{
  if (len < 2)
    return false;
  unsigned char checksum_byte1 = 0;
  unsigned char checksum_byte2 = 0;
  for (int i = 0; i < (len - 2); i++)
  {
    checksum_byte1 += data[i];
    checksum_byte2 += checksum_byte1;
  }
  return (data[len - 2] == checksum_byte1 && data[len - 1] == checksum_byte2);
}

int parse_IMU_data()
{
  // IMU
  if (check_IMU_CRC(rxbuf_imu[read_index_imu], 34))
  {
    imu.acc_x.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], ACCX_POS);
    imu.acc_y.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], ACCY_POS);
    imu.acc_z.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], ACCZ_POS);
    imu.gyr_x.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], GYRX_POS);
    imu.gyr_y.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], GYRY_POS);
    imu.gyr_z.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_imu[read_index_imu], GYRZ_POS);
  }
  // EF
  if (check_IMU_CRC(rxbuf_ef[read_index_ef], 38))
  {
    imu.roll.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFR_POS);
    imu.pitch.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFP_POS);
    imu.yaw.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFY_POS);
    imu.linacc_x.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFLINACCX_POS);
    imu.linacc_y.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFLINACCY_POS);
    imu.linacc_z.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_ef[read_index_ef], EFLINACCZ_POS);
  }
  return 0;
}

uint16_t get_acc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_x.f, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_y.f, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_z.f, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_x.f, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_y.f, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_z.f, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN() { return FLOAT_TO_D16QN(imu.roll.f, IMU_QN_EF); }
uint16_t get_pitch_in_D16QN() { return FLOAT_TO_D16QN(imu.pitch.f, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN() { return FLOAT_TO_D16QN(imu.yaw.f, IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_x.f, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_y.f, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_z.f, IMU_QN_ACC); }

void print_imu()
{
  print("\r\nIMU %d\r\n%f\t%f\t%f\n\r%f\t%f\t%f\n\r%f\t%f\t%f\n\r%f\t%f\t%f\t",
         intr_cpt,
         imu.acc_x.f,
         imu.acc_y.f,
         imu.acc_z.f,
         imu.gyr_x.f,
         imu.gyr_y.f,
         imu.gyr_z.f,
         imu.roll.f,
         imu.pitch.f,
         imu.yaw.f,
         imu.linacc_x.f,
         imu.linacc_y.f,
         imu.linacc_z.f);
}

void print_table(uint8_t *ptr, int len)
{
  for (int i = 0; i < len; i++)
  {
    print("%02x", ptr[i]);
  }
  print("\n");
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

void printCmdBuffer(CommandData& res) {
  printBuffer(res.name, res.buffer_cmd, res.buffer_cmd_len);
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

void createCommand(uint8_t descriptor_set, 
                   uint8_t field_descriptor_byte, uint8_t field_length, uint8_t field_data[],
                   uint8_t buffer[], uint8_t &buffer_len) {
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

  buffer_len = 8 + field_length;

  for (int i = 0;i<buffer_len;i++)
    serial->write(buffer[i]);
}

void createCommand(uint8_t descriptor_set, 
                   uint8_t field_descriptor_byte1, uint8_t field_length1, uint8_t field_data1[],
                   uint8_t field_descriptor_byte2, uint8_t field_length2, uint8_t field_data2[],
                   uint8_t buffer[], uint8_t &buffer_len) {
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

  buffer_len = 8 + field_length1 + field_length2;
  for (int i = 0;i<buffer_len;i++)
    serial->write(buffer[i]);

}

bool readResponseChar(CommandData &res){
      if (serial->available()) {
          uint8_t ch = serial->read();
          // print(" 0x%#.2x", ch);
          if (res.buffer_res_idx < 4) {
            res.buffer_res[res.buffer_res_idx++] = ch;
            // wait until we have 4 bytes in the buffer, which is the header of the package
            if (res.buffer_res_idx == 4) {
              // header is complete, check the header and get the package length
              if ((res.buffer_res[0] != 0x75) || (res.buffer_res[1] != 0x65)) {
                print("unexpected character %d. Shifting header", ch);
                // forget the first character, shift the 4 bytes to the left and try again with the next character
                res.buffer_res[0] = res.buffer_res[1];
                res.buffer_res[1] = res.buffer_res[2];
                res.buffer_res[2] = res.buffer_res[3];
                res.buffer_res_idx--;
              } else {
                // header is alright, get length and descriptor 
                res.descriptor_set_byte = res.buffer_res[2];
                res.payload_len = res.buffer_res[3];
                res.buffer_res_len = res.payload_len + 6; // 4 byte in header = 4 plus 2 byte checksum 
                res.buffer_res_idx = 4;
                // println("header complete");
                // println("payload1 %d",res.payload_len);
              }
            }
          } else {
            res.buffer_res[res.buffer_res_idx++] = ch;
            if (res.buffer_res_idx == res.buffer_res_len) {
              // we have the complete package now.
              //printResponseBuffer(res);
              //Serial.println("response complete");

              // fetch checksum
              uint16_t chk_asis = (((uint16_t)res.buffer_res[4+res.payload_len]) << 8) + ((uint16_t)res.buffer_res[5+res.payload_len]);
              uint16_t chk_tobe = generateChecksum(res.buffer_res, 4+res.payload_len);
              if (chk_asis != chk_tobe) {
                println("checksum is %d but should be %d",chk_asis, chk_tobe);
                return false;
              }
              // Serial.println("checksum ok");
              // extract all the fields
              uint8_t field_idx = 0;
              uint8_t copy_idx = 4;
              res.no_fields = 0;
              // println("payload %d",res.payload_len);

              while (copy_idx <= res.payload_len ) {
                // println("cooy_idx%d",copy_idx);
                res.fields[field_idx].len =   res.buffer_res[copy_idx];
                res.fields[field_idx].descr = res.buffer_res[copy_idx+1];
                // println("field[%d].len = %d",field_idx, res.fields[field_idx].len);

                for (int i = 0;i<res.fields[field_idx].len;i++) {
                  res.fields[field_idx].payload[i] = res.buffer_res[copy_idx+i];
                };
                // print("FieldXX[%d].len=%d: ", field_idx, res.fields[field_idx].len);
                // printBuffer("", res.fields[field_idx].payload, res.fields[field_idx].len);

                copy_idx += res.fields[field_idx].len;
                field_idx++;
              }
              res.no_fields = field_idx;

              // for (uint8_t i = 0;i<res.no_fields;i++) {
              //  print("Field[%d].len=%d: ", i, res.fields[i].len);
              //  printBuffer("", res.fields[i].payload, res.fields[i].len);
              //}

              // buffer has been completely transfered to buffer variables, reset for the next package
              res.buffer_res_idx = 0;

              // indicate that a full package is available in ResponseParser
              // Serial.println("response is ok");
              return true;
            }
          } 
      }

      // package hasnt been read completely 
      return false;
}

bool readResponse(CommandData &res){
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

    return false;
}

bool expectResponse(CommandData &res){
    bool ok = readResponse(res);
    if (!ok) {
      println("missed expected response %s",res.name.c_str());
    }

    return ok;
}

bool expectAckNackResponse(CommandData res) {
  bool ok = expectResponse(res);

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

bool sendPing() {
  CommandData res("ping");
  uint8_t field[] = { 0x01, 0x00};
  createCommand(0x01, 
                0x01, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

bool sendSetToIdle() {
  CommandData res("SetToIdle");
  uint8_t field[] = {  };
  createCommand(0x01, 
                0x02, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}


bool sendResumeDevice() {
  CommandData res("ResumeDevice");
  uint8_t field[] = { };
  createCommand(0x01, 
                0x06, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

bool sendSetIMUMessageFormat() {
  CommandData res("SetIMUMessageFormat");
  uint8_t rate = 1000/100;
  uint8_t field[] = { 0x01,0x02, 
                      0x04, 0x00, rate, // Accel Desc
                      0x05, 0x00, rate};// Gyro Descr
  createCommand(0x0C, 
                0x08, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

bool sendEstimationFilterDataFormat() {
  CommandData res("SetEstimationFilterDataFormat");
  uint8_t rate = 1000/100;
  uint8_t field[] = { 0x01,
                        0x03,               // 4 fields 
                        0x05, 0x00, rate,   // EF Euler
                        0x0D, 0x00, rate,   // EF Accel
                        0x0E, 0x00, rate};  // EF Angular rate
  createCommand(0x0C, 
                0x0A, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

// save the raw and the EF format 
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 18. "4. Save the IMU and Estimation Filter MIP Message Format" 
bool sendSaveFormat() {
  CommandData res("saveFormat");
  uint8_t field1[] = { 0x03,0x00}; //  Save Current IMU Message Format
  uint8_t field2[] = { 0x03,0x00}; //  Save Current Estimation Filter Message Format

  createCommand(0x0C, 
                0x08, sizeof(field1), field1,
                0x0A, sizeof(field2), field2,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 19. "5. Enable the IMU and Estimation Filter Data-streams"
bool sendEnableDataStream(bool enable) {
  CommandData res("EnableDataStream");
  uint8_t field1[] = { 0x01,0x01, (uint8_t)(enable==true?0x01:0x00)};    // Enable Continuous IMU Message 
  uint8_t field2[] = { 0x01,0x03, (uint8_t)(enable==true?0x01:0x00)};    // Enable Continuous Estimation Filter Message

  createCommand(0x0C, 
                0x11, sizeof(field1), field1,
                0x11, sizeof(field2), field2,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

bool sendSetHeading() {
  CommandData res("SetHeading");
  uint8_t field1[] = { 0x00,0x00, 0x00, 0x00};
  createCommand(0x0D, 
                0x0D, sizeof(field1), field1,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 43. "4.1.9 Device Reset (0x01, 0x7E)"
bool sendResetDevice() {
  CommandData res("ResetHeading");
  uint8_t field1[] = { };

  createCommand(0x01, 
                0x7E, sizeof(field1), field1,
                res.buffer_cmd, res.buffer_cmd_len);
  bool ok = expectAckNackResponse(res);
  return ok;
}

// enable/disable the data stream
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 34. "4.1.3 Get Device Information (0x01, 0x03)"
void sendGetDeviceInformation() {
  CommandData res("GetDeviceInformation");
  uint8_t field[] = { };

  createCommand(0x01, 
                0x03, sizeof(field), field,
                res.buffer_cmd, res.buffer_cmd_len);
  // printCmdBuffer(res);

  bool ok = readResponse(res);

  if (!assert(ok != 0, "response invalid")) return;

  if (!assert((res.no_fields >= 1) && (res.fields[0].descr == 0xF1), "field0 descriptor wrong")) return;
  if (!assert((res.no_fields >= 2) && (res.fields[1].descr == 0x81), "field1 descriptor wrong")) return;

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
  Serial.println("IMU:");

  Serial.print("   firmware      :             ");
  Serial.println(firmware_version);
  Serial.print("   model_name    : ");
  Serial.println(model_name);
  Serial.print("   serial_string : ");
  Serial.println(serial_string);
  Serial.print("   options       : ");
  Serial.println(options);
}


// change the baud rate
// according to 3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf
// Pg 62 "4.2.15 UART Baud Rate (0x0C, 0x40)"
bool sendChangeBaudRate() {
  CommandData res("ChangeBaudRate");
  uint8_t field1[] = { 0x01,                    // 01 = change the baud rate!  
                       0x00, 0x0E, 0x10, 0x00   // 921600 baud
                     };

  createCommand(0x0C, 
                0x07, sizeof(field1), field1,
                res.buffer_cmd, res.buffer_cmd_len);

  printCmdBuffer(res);
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
  delay(250);
  // next call is gonne be with the new rate
  serial->flush();
  serial->end();
  serial->begin(921600);
  return true;
}


int imu_init()
{
  /* Init uart */
  println("initialising uart for IMU");
  /* Configure parameters of an UART driver,
     * communication pins and install the driver */
  // serial->begin(115200);
  serial->begin(115200);
  serial->flush();

  // init IMU
  println("initialising CX5-25 IMU");
  /*
    75 65 01 02 02 02 E1 C7                           // Put the Device in Idle Mode
    75 65 0C 0A 0A 08 01 02 04 00 01 05 00 01 10 73   // IMU data: acc+gyr at 1000Hz
    75 65 0C 0A 0A 0A 01 02 05 00 01 0D 00 01 1B A3   // EF data: RPY + LinACC at 500Hz (max)
    75 65 0C 07 07 0A 01 01 05 00 01 06 23            // EF data: RPY at 500Hz (max)
    75 65 0C 0A 05 11 01 01 01 05 11 01 03 01 24 CC   // Enable the data stream for IMU and EF
    75 65 0D 06 06 03 00 00 00 00 F6 E4               // set heading at 0
    75 65 01 02 02 06 E5 CB                           // Resume the Device (is it needed?)
  */

  const char enableCmd[11] = {0x75, 0x65, 0x80,0x1C, 0x0E,0x04,0xBC,0x98,0x31,0x3A,0x3C};
  
  const char cmd0[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7};
  const char cmd1[16] = {0x75, 0x65, 0x0C, 0x03A, 0x0A, 0x08, 0x01, 0x02, 0x04, 0x00, 0x01, 0x05, 0x00, 0x01, 0x10, 0x73}; //IMU 1000Hz
  //const char cmd1[16] = {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x08, 0x01, 0x02, 0x04, 0x00, 0x0A, 0x05, 0x00, 0x0A, 0x22, 0xa0}; //IMU 100Hz
  //const char cmd2[13] = {0x75, 0x65, 0x0C, 0x07, 0x07, 0x0A, 0x01, 0x01, 0x05, 0x00, 0x01, 0x06, 0x23}; //EF RPY 500Hz
  //const char cmd2[13] = {0x75, 0x65, 0x0C, 0x07, 0x07, 0x0A, 0x01, 0x01, 0x05, 0x00, 0x0A, 0x0f, 0x2c};//EF RPY 50Hz
  const char cmd2[16] = {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x0A, 0x01, 0x02, 0x05, 0x00, 0x01, 0x0D, 0x00, 0x01, 0x1b, 0xa3}; //EF RPY + LinACC 500Hz

  const char cmd3[16] = {0x75, 0x65, 0x0C, 0x0A, 0x05, 0x11, 0x01, 0x01, 0x01, 0x05, 0x11, 0x01, 0x03, 0x01, 0x24, 0xCC};
  const char cmd4[12] = {0x75, 0x65, 0x0D, 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF6, 0xE4};
  const char cmd5[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xE5, 0xCB};
  // const char cmd6[13] = {0x75, 0x65, 0x0C, 0x07, 0x07, 0x40, 0x01, 0x00, 0x0E, 0x10, 0x00, 0x53, 0x9D}; // 921600 bauds

  println("Device Startup Setting");
  sendSetToIdle();
  sendSetIMUMessageFormat();
  sendEstimationFilterDataFormat();
  sendSaveFormat();
  sendResumeDevice();
  sendSetHeading();
  sendEnableDataStream(true);

  return true;
  println("IMU data: acc+gyr at 1000Hz");
   serial->write( enableCmd, sizeof(enableCmd));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }

  return true;
  println("Send Idle Mode");
  serial->write(cmd0, sizeof(cmd0));

  // println("sending new baud rate setting to the IMU");
  // serial->write(cmd6, sizeof(cmd6));

  // The ACK package is send at the old baud rate. The IMU switches to the
  // new baud rate after 250 ms. Waiting for that period of time to make sure
  // following messages are processed correctly.
  // delay(300);

  // Setup the uart handler.
  // serial->end();
  // serial->begin(921600);


  println("IMU data: acc+gyr at 1000Hz");
   serial->write( cmd1, sizeof(cmd1));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }
   println("EF data: RPY + LinACC at 500Hz (max)");
   serial->write( cmd2, sizeof(cmd2));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }

  println("Enable Data Streaming");
   serial->write( cmd3, sizeof(cmd3));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }

  println("Set heading at 0");
  serial->write( cmd4, sizeof(cmd4));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }

  println("Send Resume Mode");
  serial->write( cmd5, sizeof(cmd5));
   for (int i = 0;i<10000;i++)
      if (serial->available()) {
        Serial.print(" 0x");
        Serial.print(serial->read(),HEX);
      }

  println("Ready.");

  while (0) //for debug
  {

    print(" intr_cpt:%d\n", intr_cpt);
    parse_IMU_data();
    print_imu();
    delay(100);
  }
  return 0;
}


void IMU::setup(HardwareSerial* s) {
  serial = &Serial4;
  serial->begin(115200);

  // uint8_t test[] = { 0x75,0x65,0x01,0x02,0x02,0x02,0xE1,0xC7 };
  // serial->write(test, sizeof(test));
  // readPrint();

  // check if device is responding
   bool ok = sendPing();
   if (!ok)
    return;

  sendGetDeviceInformation();
  // sendResumeDevice();

  /*

  sendSetToIdle();
  sendSetIMUMessageFormat();
  sendEstimationFilterDataFormat();
  sendSaveFormat();
  sendResumeDevice();
  sendSetHeading();
  sendEnableDataStream(true);
  */
}

void IMU::loop() {
    if (!is_initialised)
      return;

    // read and process characters from Serial as long as possible, but dont wait if the queue is empty
    bool fullPackageAvailable = readResponseChar(res);

    // a full package as been read 
    if (fullPackageAvailable) {
        // parse the package
        res.field_idx = 1;
        res.parse_idx = 0;
        for (res.field_idx = 0;res.field_idx < res.no_fields;res.field_idx++) {
          // GPS timestamp
          // 5.1.12 GPS Correlation Timestamp (0x80, 0x12)
          // if (res.descriptor_set_byte == 0x80 && res.fields[res.field_idx].descr == 0x12) {
          //  double gps_time_of_week = parseDouble(res);
          //  uint16_t gps_week = parseU16(res);
          //  uint16_t timestamp_flags = parseU16(res);
          //}

          // Estimation Filter Calculated Value Timestamp Data
          // 5.2.2 GPS Timestamp (0x82, 0x11)
          // if (res.descriptor_set_byte == 0x82 && res.fields[res.field_idx].descr == 0x11) {
          //  double gps_time_of_week = parseDouble(res);
          //  uint16_t gps_week_number = parseU16(res);
          //  uint16_t timestamp_flags = parseU16(res);
          //}

          // Scaled Accelerometer Vector
          // 5.1.2 Scaled Gyro Vector (0x80, 0x05)
          if ((res.descriptor_set_byte == 0x80) && (res.fields[res.field_idx].descr == 0x04)) {
            imu_data.acc_x = parseFloat(res);
            imu_data.acc_y = parseFloat(res);
            imu_data.acc_z = parseFloat(res);
          }

          // Scaled Gyro Vector
          // 5.1.1 Scaled Accelerometer Vector (0x80, 0x04)
          if ((res.descriptor_set_byte == 0x80) && (res.fields[res.field_idx].descr == 0x04)) {
            imu_data.gyro_x = parseFloat(res);
            imu_data.gyro_y = parseFloat(res);
            imu_data.gyro_z = parseFloat(res);
          }

          // roll pitch yaw 
          // 5.2.5 Orientation, Euler Angles (0x82, 0x05)
          if ((res.descriptor_set_byte == 0x82) && (res.fields[res.field_idx].descr ==0x05)) {
              imu_data.roll = parseFloat(res);
              imu_data.pitch = parseFloat(res);
              imu_data.yaw = parseFloat(res);
          }

          // Linear Acceleration X,Y,Z
          // 5.2.12 Linear Acceleration (0x82, 0x0D)
          if ((res.descriptor_set_byte == 0x82 && res.fields[res.field_idx].descr ==0x0D)) {
              imu_data.lin_acc_x = parseFloat(res);
              imu_data.lin_acc_y = parseFloat(res);
              imu_data.lin_acc_z= parseFloat(res);
          }

          // Compensated Angular Rate
          // 5.2.8 Compensated Angular Rate (0x82, 0x0E)
          if ((res.descriptor_set_byte == 0x82) && (res.fields[res.field_idx].descr ==0x0E)) {
              imu_data.ang_rate_x = parseFloat(res);
              imu_data.ang_rate_y = parseFloat(res);
              imu_data.ang_rate_z = parseFloat(res);
          }
        }

        // we updated the IMU data package
        imu_data_modified = true;
        imu_data_modified_us = micros();
    }
}

bool IMU::isNewPackageAvailable() {
  bool m = imu_data_modified; 
  imu_data_modified = false; 
  return m;
} 

void IMU::printData() {
  print("\r\nIMU \r\n acc=(%f\t%f\t%f)\n\rGyro:(%f\t%f\t%f)\n\rrpy:(%f\t%f\t%f)\n\rlinacc:(%f\t%f\t%f)\r\nangrate:(%f\t%f\t%f)\t",
         imu_data.acc_x,
         imu_data.acc_y,
         imu_data.acc_z,
         imu_data.gyro_x,
         imu_data.gyro_y,
         imu_data.gyro_z,
         imu_data.roll,
         imu_data.pitch,
         imu_data.yaw,
         imu_data.lin_acc_x,
         imu_data.lin_acc_y,
         imu_data.lin_acc_z,
         imu_data.ang_rate_x,
         imu_data.ang_rate_y,
         imu_data.ang_rate_z);
}
 
