
#include "Arduino.h"
#include "ODrive.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODrive::ODrive() {
}


void ODrive::setup(Stream& serial) {
    serial_ = &serial;
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

float ODrive::readFloat() {
    return readString().toFloat();
}

float ODrive::GetVelocity(int motor_number) {
	*serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODrive::readFloat();
}

float ODrive::GetPosition(int motor_number) {
    *serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODrive::readFloat();
}

float ODrive::getVBusVoltage() {
    (*serial_) << "r vbus_voltage\n";
    return ODrive::readFloat();
}

void ODrive::getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision) {
    *serial_ << "r fw_version_major\n";
    major = readInt();
    *serial_ << "r fw_version_minor\n";
    minor = readInt();
    *serial_ << "r fw_version_revision\n";
    revision = readInt();
}


int32_t ODrive::readInt() {
    return readString().toInt();
}

bool ODrive::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    *serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            *serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODrive::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_->available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_->read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
