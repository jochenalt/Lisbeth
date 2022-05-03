
#include "Arduino.h"
#include "ODrive.h"

bool doCheckSums = true;        // use checksums for all communication to ODrive
const uint8_t NO_ERROR = 0;
const uint8_t ERROR_COMM_ERROR = 100;
static uint8_t error = NO_ERROR;

#define BUFFER_SIZE 64
void ODrive::resetError() {
    error = NO_ERROR ;
}

uint8_t ODrive::getLastError() {
    return error;
}

uint16_t commTimeout_ms = 100;  // [ms] give up to wait for a result after this time
// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

uint16_t  addChecksum(bool withChecksum, char buffer[], uint16_t len) {
    if (withChecksum) {
        uint16_t sum = 0;
        for (uint8_t i = 0;i<len;i++) {
            sum ^= buffer[i];
        }
        uint16_t newLen = sprintf(&buffer[len], "*%d\n", sum);
        return newLen + len;
    } else {
        uint16_t newLen = sprintf(&buffer[len], "\n");
        return newLen + len;
    }
}

int32_t ODrive::readInt(bool withCheckSum, uint32_t &delay) {
    return readString(withCheckSum, delay).toInt();
}

bool ODrive::readBool(bool withCheckSum, uint32_t &delay) {
    String s = readString(withCheckSum, delay);
    if ((s == "1"))
        return true;
    if ((s == "0"))
        return false;
    Serial.print("Reading bool from ");
    Serial.print(s);
    Serial.print(" failed.");
    error = ERROR_COMM_ERROR;
    return false;
}

bool ODrive::run_state(bool withCheckSum, int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    uint32_t del;
    *serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            *serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt(withCheckSum, del) != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODrive::readString(bool withCheckSum, uint32_t &delay_us) {
    String str;
    uint32_t start = micros();
    uint16_t retrievedCheckSum = 0;
    uint16_t actualCheckSum = 0;
    bool inCheckSum = false;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_->available()) {
            if (millis() - timeout_start >= commTimeout_ms) {
                Serial.print("Timeout whilst reading: ");
                Serial.println(str);
                error = ERROR_COMM_ERROR;
                return str;
            }
        }
        if (str.length() == 0) {
            delay_us = micros() - start;
        }
        char c = serial_->read();
        if (c == '\n')
            break;
        if (c != '\r') {// charavcter before '\n', ignore it
            if (withCheckSum) {
                if (inCheckSum) {
                    retrievedCheckSum = (uint8_t)(c - '0') + retrievedCheckSum*10;
                } else {
                    if (c == '*')  {
                        inCheckSum = true;
                    }
                    else 
                    {
                        actualCheckSum ^= (uint8_t)c;
                    }
                }
            }
        }
        if (!inCheckSum && (c != '\n') && (c != '\r'))
            str.append(c);
    }
    if (withCheckSum) {
        if (retrievedCheckSum != actualCheckSum) {
            error = ERROR_COMM_ERROR;
            Serial.print("Checksum retrieved is ");
            Serial.print(retrievedCheckSum);
            Serial.print(" instead of ");
            Serial.println(actualCheckSum);
        }
    }
    return str;
}



float ODrive::readFloat(bool withCheckSum, uint32_t &delay) {
    return readString(withCheckSum, delay).toFloat();
}


ODrive::ODrive() {
}

void ODrive::clearErrors() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "sc"));
    serial_->write(buffer,strLen );
}
void ODrive::eraseConfiguration() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "se"));
    serial_->write(buffer,strLen );
}

void ODrive::saveConfiguration() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "ss"));
    serial_->write(buffer,strLen );
}

void ODrive::reboot() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "sr"));
    serial_->write(buffer,strLen );
}


void ODrive::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    *serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODrive::SetPosition(float position0, float velocity_feedforward0, float current_feedforward0,
                         float position1, float velocity_feedforward1, float current_feedforward1) {
    *serial_ << "p " << " " << position0 << " " << velocity_feedforward0 << " " << current_feedforward0 
                            << position1 << " " << velocity_feedforward1 << " " << current_feedforward1 
             << "\n";
}

void ODrive::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODrive::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    *serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODrive::SetCurrent(int motor_number, float current) {
    *serial_ << "c " << motor_number << " " << current << "\n";
}

void ODrive::TrapezoidalMove(int motor_number, float position) {
    *serial_ << "t " << motor_number << " " << position << "\n";
}


float ODrive::GetVelocity(int motor_number) {
    uint32_t delay;
	*serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODrive::readFloat(false, delay);
}

float ODrive::GetPosition(int motor_number) {
    uint32_t delay;
    *serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODrive::readFloat(false, delay);
}


void ODrive::setup(HardwareSerial& serial, uint32_t baudrate) {
    serial_ = &serial;
    serial_->begin(baudrate);
}

void ODrive::sendReadParamRequest(String name) {
    char buffer[BUFFER_SIZE];  
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "r %s", name.c_str()));
    serial_->write(buffer,strLen );
}

void ODrive::sendWriteParamRequest(String name, float value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "w %s %f", name.c_str(), value));
    serial_->write(buffer,strLen );
}

void ODrive::sendWriteParamRequest(String name, int32_t value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "w %s %ld", name.c_str(), value));
    serial_->write(buffer,strLen );
}

void ODrive::sendWriteParamRequest(String name, bool value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen;
    if (value)
        strLen = addChecksum(true, buffer, sprintf(buffer, "w %s 1", name.c_str()));
    else        
        strLen = addChecksum(true, buffer, sprintf(buffer, "w %s 0", name.c_str()));

    serial_->write(buffer,strLen );
}

bool ODrive::setParamFloat(String name, float value) {
    uint32_t delay;
    sendReadParamRequest(name);
    float value_read = readFloat(true, delay);
    if (abs(value_read - value) > 0.000001) {
        sendWriteParamRequest(name, value);
        sendReadParamRequest(name);
        value_read = readFloat(true, delay);
        if (abs(value_read - value) > 0.000001) {
            Serial.print("set value of");
            Serial.print(name);
            Serial.print(" := ");
            Serial.print(value);
            Serial.print(", but is now ");
            Serial.println(value_read);
        }
        return true;
    }
    return false;
}

bool ODrive::setParamBool(String name, bool value) {
    uint32_t delay;
    sendReadParamRequest(name);
    bool value_read = readBool(true, delay);
    if (value_read !=  value) {
        Serial.print("bool value wrong");
        sendWriteParamRequest(name, value);
        uint32_t delay;
        String s = readString(true, delay);
        Serial.print("readBW:");
        Serial.print(s);
        sendReadParamRequest(name);
        value_read = readBool(true, delay);
        if (value_read != value) {
            Serial.print("set value of");
            Serial.print(name);
            Serial.print(" := ");
            Serial.print(value);
            Serial.print(", but is now ");
            Serial.println(value_read);
        }
        return true;
    }
    return false;
}

bool ODrive::setParamInt(String name, int32_t value) {
    uint32_t delay;
    sendReadParamRequest(name);
    int value_read = readInt(true, delay);
    if (value != value_read) {
        sendWriteParamRequest(name, value);
        sendReadParamRequest(name);
        value_read = readInt(true, delay);
        if (value_read != value) {
            Serial.print("set value of");
            Serial.print(name);
            Serial.print(" := ");
            Serial.print(value);
            Serial.print(", but is now ");
            Serial.println(value_read);
        }
        return true;
    }
    return false;
}

bool ODrive::setParamAxisFloat(String name, float value) {
    bool valueChanged = setParamFloat("axis0." + name, value);
    valueChanged =setParamFloat("axis1." + name, value) || valueChanged;
    return valueChanged;
}

bool ODrive::setParamAxisInt(String name, int32_t value) {
    bool valueChanged = setParamInt("axis0." + name, value);
    valueChanged = setParamInt("axis1." + name, value) || valueChanged;
    return valueChanged;
}

bool ODrive::setParamAxisBool(String name, bool value) {
    bool valueChanged = setParamBool("axis0." + name, value);
    valueChanged = setParamBool("axis1." + name, value) || valueChanged;
    return valueChanged;
}

float ODrive::getParamFloat(String name) {
    uint32_t delay;
    sendReadParamRequest(name);
    float value_read = readFloat(true, delay);
    return value_read;
}

int32_t ODrive::getParamInt(String name) {
    uint32_t delay;
    sendReadParamRequest(name);
    int32_t value_read = readInt(true, delay);
    return value_read;
}

bool ODrive::getParamBool(String name) {
    uint32_t delay;
    sendReadParamRequest(name);
    bool value_read = readInt(true, delay);
    return value_read;
}

float ODrive::getVBusVoltage() {
    char buffer[BUFFER_SIZE];
    uint32_t delay;
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "r vbus_voltage"));
    serial_->write(buffer,strLen );
    return ODrive::readFloat(true, delay);
}

void ODrive::getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision) {
    char buffer[BUFFER_SIZE];
    uint32_t delay;
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "r fw_version_major"));
    serial_->write(buffer,strLen );
    major = readInt(true,delay);
    strLen = addChecksum(true,buffer, sprintf(buffer, "r fw_version_minor"));
    serial_->write(buffer,strLen );
    minor = readInt(true,delay);
    strLen = addChecksum(true,buffer, sprintf(buffer, "r fw_version_revision"));
    serial_->write(buffer,strLen );
    revision = readInt(true,delay);
}

String ODrive::getInfoDump() {
    char buffer[BUFFER_SIZE];
    uint32_t delay = 0;
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "i"));
    serial_->write(buffer,strLen );
    String str = readString(true, delay);
    str += "\r\n" + readString(true,delay);
    str += "\r\n" + readString(true,delay);
    return str;
}


void ODrive::getFeedback(int motor_number, float &currentPosition, float &currentVelocity, float &currentCurrent) {
    char buffer[BUFFER_SIZE];
    uint32_t delay = 0;
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "f %d", motor_number));
    
    serial_->write(buffer,strLen);
    String response = readString(true,delay);
    char responseBuffer[response.length()];
    response.toCharArray (responseBuffer,response.length() );
    int items = sscanf(responseBuffer, "%f %f %f",  &currentPosition, &currentVelocity, &currentCurrent);
    if (items != 3) {
            error = ERROR_COMM_ERROR;
            Serial.print("did not receive 3 items but");
            Serial.print(items);
            Serial.print(" in ");
            Serial.print(responseBuffer);
            Serial.print("EOL");
            Serial.println();
    }
}


void ODrive::getFeedback(float &currentPosition1, float &currentVelocity1, float &currentCurrent1,
                         float &currentPosition2, float &currentVelocity2, float &currentCurrent2) {

    sendRequestForFeedback();
    uint32_t delay;
    receiveFeedback(delay,
                    currentPosition1, currentVelocity1,currentCurrent1,
                    currentPosition2, currentVelocity2,currentCurrent2);
}

void ODrive::sendRequestForFeedback() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "f"));
    serial_->write(buffer,strLen);
}


void ODrive::receiveFeedback(uint32_t &delay, float &currentPosition1, float &currentVelocity1, float &currentCurrent1,
                             float &currentPosition2, float &currentVelocity2, float &currentCurrent2) {
    String response = readString(true, delay);
    char responseBuffer[response.length()];
    response.toCharArray (responseBuffer,response.length());
    int items = sscanf(responseBuffer, "%f %f %f %f %f %f", 
                &currentPosition1, &currentVelocity1, &currentCurrent1,
                &currentPosition2, &currentVelocity2, &currentCurrent2);
    if (items != 6) {
            error = ERROR_COMM_ERROR;
            Serial.print("did not receive 6 items in");
            Serial.print(responseBuffer);
            Serial.println();
    }
}
ODrives::ODrives() {
    num_odrives = 0;
    loopAvrTime_us = 0;
}

void ODrives::addODrive(HardwareSerial& s, uint32_t baudrate) {
    if (num_odrives == MAX_ODRIVES) {
        Serial.println("Max number of ODrives exceeded.");
    }
    odriveSerial[num_odrives] = &s;
    baudrates[num_odrives] = baudrate;
    num_odrives++;

}

ODrive& ODrives::operator[](uint8_t i) {
    return odrive[i];
}

void ODrives::setup() {
    for (int i = 0;i<num_odrives;i++) {
        odrive[i].setup(*odriveSerial[i], baudrates[i]);
    }
}

void ODrives::loop() {
    uint32_t start = micros();
    // send all requests for feedback first
    for (int i = 0;i<num_odrives;i++) {
        odrive[i].sendRequestForFeedback();
    };
    uint32_t afterSend = micros();
    for (int i = 0;i<num_odrives;i++) {
        Feedback& fb1 = feedback[i*2];
        Feedback& fb2 = feedback[i*2+1];
        uint32_t delay;
        odrive[i].receiveFeedback(delay, fb1.pos, fb1.vel, fb1.curr, fb2.pos, fb2.vel, fb2.curr);
        avrDelayTime_us = (avrDelayTime_us + delay)/2;
    }
    uint32_t end = micros();
    loopAvrTime_us = (loopAvrTime_us + (end-start)) / 2;
    loopSendAvrTime_us = (loopAvrTime_us + (afterSend-start)) / 2; 
}

void ODrives::getFeedback(Feedback fb[]) {
    for (int i = 0;i<num_odrives;i++) {
        fb[i] = feedback[i];
    } 
}

void ODrives::setParams() {
    for (int i = 0;i<num_odrives;i++) {
        // general
        odrive[i].setParamBool("config.enable_brake_resistor",true);
        odrive[i].setParamInt("config.brake_resistance",2);
        odrive[i].setParamAxisBool("config.startup_encoder_offset_calibration",false);

        // motor
        odrive[i].setParamAxisInt("motor.config.current_lim",10);
        odrive[i].setParamAxisInt("motor.config.pole_pairs",12);
        odrive[i].setParamAxisFloat("motor.config.torque_constant",8.27 / 300);
        odrive[i].setParamAxisInt("motor.config.motor_type",MOTOR_TYPE_HIGH_CURRENT);
        odrive[i].setParamAxisFloat("motor.config.resistance_calib_max_voltage",4.0);
        odrive[i].setParamAxisFloat("motor.config.calibration_current",4.0);
        odrive[i].setParamAxisFloat("motor.config.current_lim",4.0);
        odrive[i].setParamAxisFloat("motor.config.current_control_bandwidth ",200);

        // controller
        odrive[i].setParamAxisFloat("controller.config.vel_limit",10);
        odrive[i].setParamAxisFloat("controller.config.pos_gain",100);
        odrive[i].setParamAxisFloat("controller.config.vel_integrator_gain",0.02);
        odrive[i].setParamAxisFloat("controller.config.vel_gain",0.02);

        // encoder
        odrive[i].setParamAxisInt("encoder.config.cpr",20000);
        odrive[i].setParamAxisInt("encoder.config.direction",-1);
        odrive[i].setParamAxisBool("encoder.config.use_index",true);
        odrive[i].setParamAxisFloat("controller.config.input_filter_bandwidth",20);
        odrive[i].setParamAxisInt("controller.config.input_mode",INPUT_MODE_POS_FILTER);
    }
}

