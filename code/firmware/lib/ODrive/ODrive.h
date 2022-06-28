
#ifndef ODRIVE_H_
#define ODRIVE_H_

#include "Arduino.h"
#include "ODriveEnums.h"

#define ODRIVE_STANDARD_SERIAL_BAUD_RATE 115200                  // default of odrive, is reconfigured during startup to 
#define ODRIVE_HIGH_SERIAL_BAUD_RATE 921600                      // default of odrive, is reconfigured during startup to 

struct Feedback {
    float pos;
    float vel;
    float curr;
};

class ODrive {
    friend class ODrives;
public:
    ODrive();

    // returns true, if that motor has been activated
    bool isActive(int motor_number);

    // activate a sepcific motor
    void activate(int motor_number);

    // name this instance
    void setName(String name1, String name2, String oname="");

    // retrieve that name
    String getName(int motor_number);

    // reset the global error flag
    void resetError();

    // return last error, if everything is fine, NO_ERROR is returned 
    uint8_t getLastError();

    // setup ODrive by establishing the communication via the passed UART stream
    void setup(HardwareSerial& serial);

    // set all ODrive specific Parameters for our motor and encoder
    void setODriveParams();

    // set the right baudrate (which is higher than ODrives default 115200)
    void setBaudRate();

    // reset everything (including the calibration of both motors) but set motor/encoder specifics
    void factoryReset();

    // start motor calibration
    void calibrate (int motor_number);

    // startup a motor by finding the index position and setting it in the  closed loop control 
    void startup(int motor_number = -1);

    // shutdown a motor, by putting it into ide mode
    void shutdown(int motor_number);

    // return the voltage of the ODrive board
    float getVBusVoltage();

    // return the version of the firmware
    void getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision);

    // dump firmware and HW version of ODrive
    String getInfoDump();

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

private:
    bool errorIfNotActive(int motor_number, String text="");
    void clearErrors();
    bool readError(int motor_number, bool printError);
    bool readError(bool printError);
    void eraseConfiguration();
    void saveConfiguration();
    void reboot(bool waitUntilDone = true);
    void printErrors();
    float readFloat(bool withCheckSum, uint32_t &delay);
    int32_t readInt(bool withCheckSum, uint32_t &delay);
    String readString(bool withCheckSum, uint32_t &delay_us);
    bool readBool(bool withCheckSum, uint32_t &delay_us);

    float readFloatFromBuffer ( char buffer[], uint8_t bufferLen, uint8_t &pos);
    bool waitForState (int motor_number, String name, int32_t value, float timeout_s);
    void requestedState(int motor_number, int32_t requested_state) ;
    bool requestedState(int axis, int32_t requested_state, bool wait_for_idle, float timeout = 10.0f);
    
    HardwareSerial* serial_ = NULL;
    String oname;           // name of this ODrive
    String mname[2] ;       // names of the motors 
    bool active[2];         // true if the corresponding motor has been activated (allowing to use only one motor our of two )
};

#define MAX_ODRIVES 6

class ODrives {
    public:
        // turn on log output of all communication to ODrives
        static void debugComm(bool onOff);

        // turn on log output of all ODrive API calls
        static void debugAPI(bool onOff);
        
        ODrives();
        void addODrive(HardwareSerial& s, String name0, String name1, String on="");

        uint8_t getNumberODrives() { return num_odrives; };
        // initialise all ODrives
        void setup();

        // set all ODrive parameters (but do not calibrate)
        void setParams(); 

        // calibrate all motors at once
        void calibrate();

        // start all motors
        void startup();

        // shutdown all motors
        void shutdown();

        // return true if correctly setup()
        bool isSetup();

        // return one individual ODrive
        ODrive& operator[](uint8_t i); 
        void loop();
        void getFeedback(Feedback feedback[]);

        uint32_t loopAvrTime_us;                        // avr. time to execute loop
        uint32_t avrDelayTime_us;
        uint32_t loopSendAvrTime_us;

    private:
        bool waitForState (String name, int32_t value, float timeout_s);

        uint8_t num_odrives;
        ODrive odrive[MAX_ODRIVES];                     //  ODrive objects, one per ODrive
        HardwareSerial* odriveSerial[MAX_ODRIVES];      // their UART interface
        uint32_t baudrates[MAX_ODRIVES];
        Feedback feedback[MAX_ODRIVES*2];               // feedback of all motors  

        bool is_setup = false;

};
#endif 
