
#ifndef ODRIVE_H_
#define ODRIVE_H_

#include "Arduino.h"
#include "ODriveEnums.h"

class ODrive {

public:
    ODrive();
    void setup(Stream& serial);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);
    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    // General params
    float readFloat();
    int32_t readInt();

    // return the voltage of the ODrive board
    float getVBusVoltage();
    // return the version of the firmware
    void getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision);

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);
private:
    String readString();

    Stream* serial_ = NULL;
};

#endif 
