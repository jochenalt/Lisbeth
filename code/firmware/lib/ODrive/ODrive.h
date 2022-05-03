
#ifndef ODRIVE_H_
#define ODRIVE_H_

#include "Arduino.h"
#include "ODriveEnums.h"

class ODrive {

public:
    ODrive();

    // reset the global error flag
    void resetError();

    // return last error, if everything is fine, NO_ERROR is returned 
    uint8_t getLastError();

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
    void sendRequestForFeedback();
    void receiveFeedback(uint32_t &delay, float &currentPosition1, float &currentVelocity1, float &currentCurrent1,
                         float &currentPosition2, float &currentVelocity2, float &currentCurrent2);

    void getFeedback(int motor_number, float &currentPosition, float &currentVelocity, float &currentCurrent);

    // define desired state of motor and return the current state
    void setTargetState(float position, float velocity, float torque, 
                        float &currentPosition, float &currentVelocity, float &currentCurrent);

    // parameter
    void sendReadParamRequest(String name);

    void sendWriteParamRequest(String name, float value);
    void sendWriteParamRequest(String name, bool value);
    void sendWriteParamRequest(String name, int32_t value);
    bool setParamFloat(String name, float value);
    bool setParamBool(String name, bool value);
    bool setParamInt(String name, int32_t value);
    float getParamFloat(String name);
    bool getParamBool(String name);
    int32_t getParamInt(String name);
    bool setParamAxisInt(String name, int32_t value);
    bool setParamAxisFloat(String name, float value);
    bool setParamAxisBool(String name, bool value);

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

    float readFloat(bool withCheckSum, uint32_t &delay);
    int32_t readInt(bool withCheckSum, uint32_t &delay);
    String readString(bool withCheckSum, uint32_t &delay_us);
    bool readBool(bool withCheckSum, uint32_t &delay_us);

    float readFloatFromBuffer ( char buffer[], uint8_t bufferLen, uint8_t &pos);


    HardwareSerial* serial_ = NULL;
};

#define MAX_ODRIVES 6

struct Feedback {
    float pos;
    float vel;
    float curr;
};

class ODrives {
    public:
        ODrives();
        void addODrive(HardwareSerial& s, uint32_t baudrate);

        // initialise all ODrives
        void setup();

        // set all ODrive parameters (but do not calibrate)
        void setParams(); 

        // return one individual ODrive
        ODrive& operator[](uint8_t i); 
        void loop();
        void getFeedback(Feedback feedback[]);

        uint32_t loopAvrTime_us;                        // avr. time to execute loop
        uint32_t avrDelayTime_us;
        uint32_t loopSendAvrTime_us;

    private:
        uint8_t num_odrives;
        ODrive odrive[MAX_ODRIVES];                     //  ODrive objects, one per ODrive
        HardwareSerial* odriveSerial[MAX_ODRIVES];      // their UART interface
        uint32_t baudrates[MAX_ODRIVES];
        Feedback feedback[MAX_ODRIVES*2];               // feedback of all motors      
};
#endif 
