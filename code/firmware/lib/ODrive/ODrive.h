
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

    // dump firmware and HW version of ODrive
    String getInfoDump();

    void clearErrors();
    void eraseConfiguration();
    void saveConfiguration();
    void reboot();

    //  retrieve current state of motor
    void getFeedback(float &currentPosition1, float &currentVelocity1, float &currentCurrent1,
                     float &currentPosition2, float &currentVelocity2, float &currentCurrent2);

    void getFeedback(int motor_number, float &currentPosition, float &currentVelocity, float &currentCurrent);

    // define desired state of motor and return the current state
    void setTargetState(float position, float velocity, float torque, 
                        float &currentPosition, float &currentVelocity, float &currentCurrent);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetPosition(float position0, float velocity_feedforward0, float current_feedforward0,
                     float position1, float velocity_feedforward1, float current_feedforward1);

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
    float readFloatFromBuffer ( char buffer[], uint8_t bufferLen, uint8_t &pos);


    HardwareSerial* serial_ = NULL;
};

#endif 
