
#ifndef ODRIVE_H_
#define ODRIVE_H_

#include "Arduino.h"
#include "ODriveEnums.h"

class ODrive {

public:
    ODrive();

    // setup ODrive by establishing the communication via the passed UART stream
    void setup(HardwareSerial& serial, uint32_t baudrate );

    // return the voltage of the ODrive board
    float getVBusVoltage();

    // return the version of the firmware
    void getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision);

    //  retrieve current state of motor
    void getState(float &currentPosition, float &currentVelocity, float &currentCurrent);

    // define desired state of motor and return the current state
    void setTargetState(float position, float velocity, float torque, 
                        float &currentPosition, float &currentVelocity, float &currentCurrent);

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


    // State helper
    bool run_state(bool withCheckSum, int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);
private:
    float readFloat(bool withCheckSum);
    int32_t readInt(bool withCheckSum);
    String readString(bool withCheckSum);

    HardwareSerial* serial_ = NULL;
};

#endif 
