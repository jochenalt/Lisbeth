
#include "Arduino.h"
#include "ODrive.h"

bool doCheckSums = true;        // use checksums for all communication to ODrive
uint16_t commTimeout_ms = 100;  // [ms] give up to wait for a result after this time
// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


int32_t ODrive::readInt(bool withCheckSum) {
    return readString(withCheckSum).toInt();
}

bool ODrive::run_state(bool withCheckSum, int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    *serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            *serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt(withCheckSum) != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODrive::readString(bool withCheckSum) {
    String str;
    uint16_t retrievedCheckSum = 0;
    uint16_t actualCheckSum = 0;
    bool inCheckSum = false;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_->available()) {
            if (millis() - timeout_start >= commTimeout_ms) {
                return str;
            }
        }
        char c = serial_->read();
        if (c == '\n')
            break;
        if (c != '\r') // charavcter before '\n', ignore it
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
        str += c;
    }
    if (withCheckSum) {
        if (retrievedCheckSum != actualCheckSum) {
            Serial.print("Checksum retrieved is ");
            Serial.print(retrievedCheckSum);
            Serial.print(" but should be ");
            Serial.println(actualCheckSum);
        }
    }
    return str;
}

float ODrive::readFloat(bool withCheckSum) {
    return readString(withCheckSum).toFloat();
}

ODrive::ODrive() {
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
	*serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODrive::readFloat(false);
}

float ODrive::GetPosition(int motor_number) {
    *serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODrive::readFloat(false);
}

uint16_t  addChecksum(char buffer[], uint16_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0;i<len;i++) {
        sum ^= buffer[i];
    }
    uint16_t newLen = sprintf(&buffer[len], "*%d\n", sum);
    return newLen + len;
}


void ODrive::setup(HardwareSerial& serial, uint32_t baudrate) {
    serial_ = &serial;
    serial_->begin(baudrate);
}

float ODrive::getVBusVoltage() {
    char buffer[40];
    uint16_t strLen = addChecksum(buffer, sprintf(buffer, "r vbus_voltage"));
    serial_->write(buffer,strLen );
    return ODrive::readFloat(true);
}

void ODrive::getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision) {
    char buffer[40];
    uint16_t strLen = addChecksum(buffer, sprintf(buffer, "r fw_version_major"));
    serial_->write(buffer,strLen );
    major = readInt(true);
    strLen = addChecksum(buffer, sprintf(buffer, "r fw_version_minor"));
    serial_->write(buffer,strLen );
    minor = readInt(true);
    strLen = addChecksum(buffer, sprintf(buffer, "r fw_version_revision"));
    serial_->write(buffer,strLen );
    revision = readInt(true);
}

