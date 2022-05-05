
#include "Arduino.h"
#include "ODrive.h"

bool doCheckSums = true;        // use checksums for all communication to ODrive
const uint32_t NO_ERROR = 0;
const uint32_t ERROR_COMM_ERROR = 100;
const uint32_t ERROR_CALIBRATION_FAILED = 101;
const uint32_t ERROR_ODRIVE_ERROR = 102;

static uint32_t error = NO_ERROR;

// set this if all communication should be logged
#define DEBUG_COMM
// set this if all API calls should be logged
#define DEBUG_API

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

void ODrive::setName(String s1, String s2) {
    name1 = s1;
    name2 = s2;
}

String ODrive::getName(int motornumber) {
    return (motornumber==0)?name1:((motornumber==1)?name2:"");
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

bool ODrive::requestedState(int motor_number, int32_t requested_state, bool wait_for_idle, float timeout_s) {
    uint32_t endoftime_ms = millis() + timeout_s *1000.0;
    bool timeoutFired = false;
    String axis = "axis" + String(motor_number);
#ifdef DEBUG_API
    Serial.print(axis + ".requested_state = ");
    Serial.println(requested_state);
#endif

    sendWriteParamRequest(axis + ".requested_state", requested_state);
    int current_state = 0;
    if (wait_for_idle) {
        do {
            delay(100);
            current_state = getParamInt(axis + ".current_state");
            timeoutFired = millis() > endoftime_ms;
        } while (current_state != AXIS_STATE_IDLE && !timeoutFired);
    }

    return !timeoutFired;
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
                Serial.print("<T>");
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
                    if (c == '*')  
                        inCheckSum = true;
                    else 
                        actualCheckSum ^= (uint8_t)c;
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
#ifdef DEBUG_COMM    
    Serial.print("   -> ");
    Serial.println(str);
#endif
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
    error = 0;
}

bool ODrive::readError(int motor_number, bool printError) {
    String axis = "axis" + String(motor_number); 

    uint32_t odrive_error = getParamInt("error");
    uint32_t axis_error = getParamInt(axis + ".error");
    uint32_t motor_error = getParamInt(axis + ".motor.error");
    uint32_t encoder_error = getParamInt(axis + ".encoder.error");
    uint32_t controller_error = getParamInt(axis + ".controller.error");
    uint32_t sensorless_estimator_error = getParamInt(axis + ".sensorless_estimator.error");

    if (printError) {
        if (odrive_error > 0) {
            Serial.print("odriv.error=");
            Serial.println(odrive_error);
        }
        if (axis_error > 0) {
            Serial.print(axis + ".error=");
            Serial.println(axis_error);
        }
        if (motor_error > 0) {
            Serial.print(axis + ".motor.error=");
            Serial.println(motor_error);
        }
        if (encoder_error > 0) {
            Serial.print(axis + ".encoder.error=");
            Serial.println(encoder_error);
        }
        if (controller_error > 0) {
            Serial.print(axis + ".controller.error=");
            Serial.println(controller_error);
        }
        if (sensorless_estimator_error > 0) {
            Serial.print(axis + ".sensorless_estimator.error=");
            Serial.println(sensorless_estimator_error);
        }
    }
    bool anyError = ((odrive_error > 0) || (axis_error > 0) || (motor_error > 0) || (encoder_error > 0) || (controller_error > 0) || (sensorless_estimator_error > 0));
    if (anyError) {
        error = ERROR_ODRIVE_ERROR;
    }
    return anyError; 
}


bool ODrive::readError(bool printError) {
    bool isError = readError(0);
    isError = readError(0) || isError;
    return isError;
}

void ODrive::eraseConfiguration() {

    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "se"));
    serial_->write(buffer,strLen );
#ifdef DEBUG_API
    Serial.println("eraseConfiguration");
#endif
#ifdef DEBUG_COMM
    Serial.print(buffer);
#endif
    delay(500);
}

void ODrive::saveConfiguration() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "ss"));
    serial_->write(buffer,strLen );
#ifdef DEBUG_API
    Serial.println("saveConfiguration");
#endif
#ifdef DEBUG_COMM
    Serial.print(buffer);
#endif
    delay(500);
}

void ODrive::reboot() {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true,buffer, sprintf(buffer, "sr"));
    serial_->write(buffer,strLen );
#ifdef DEBUG_API
    Serial.println("reboot");
#endif
#ifdef DEBUG_COMM
    Serial.print(buffer);   
#endif
    delay(500);
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


void ODrive::setup(HardwareSerial& serial) {
    serial_ = &serial;
    setBaudRate();
}

void ODrive::sendReadParamRequest(String name) {
    char buffer[BUFFER_SIZE];  
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "r %s", name.c_str()));
    serial_->write(buffer,strLen );
#ifdef DEBUG_COMM
    Serial.print("   Send:");
    Serial.print(buffer); // the string contains a \n already, but not yet a \r
    Serial.print('\r');
#endif
}

void ODrive::sendWriteParamRequest(String name, float value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "w %s %f", name.c_str(), value));
    serial_->write(buffer,strLen );
#ifdef DEBUG_COMM
    Serial.print("Send:");
    Serial.println(buffer);
    Serial.print('\r');
#endif
}

void ODrive::sendWriteParamRequest(String name, int32_t value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen = addChecksum(true, buffer, sprintf(buffer, "w %s %ld", name.c_str(), value));
    serial_->write(buffer,strLen );
#ifdef DEBUG_COMM
    Serial.print("Send:");
    Serial.println(buffer);
    Serial.print('\r');
#endif

}

void ODrive::sendWriteParamRequest(String name, bool value) {
    char buffer[BUFFER_SIZE];
    uint16_t strLen;
    if (value)
        strLen = addChecksum(true, buffer, sprintf(buffer, "w %s 1", name.c_str()));
    else        
        strLen = addChecksum(true, buffer, sprintf(buffer, "w %s 0", name.c_str()));

    serial_->write(buffer,strLen );
#ifdef DEBUG_COMM
    Serial.print("Send:");
    Serial.println(buffer);
#endif

}

bool ODrive::setParamFloat(String name, float value) {
#ifdef DEBUG_API
    Serial.print("set ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value);
#endif
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
#ifdef DEBUG_API
    Serial.print("set ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value);
#endif

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
#ifdef DEBUG_API
    Serial.print("set ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value);
#endif

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
#ifdef DEBUG_API
    Serial.print("get ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value_read);
#endif

    return value_read;
}

int32_t ODrive::getParamInt(String name) {
    uint32_t delay;
    sendReadParamRequest(name);
    int32_t value_read = readInt(true, delay);
#ifdef DEBUG_API
    Serial.print("get ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value_read);
#endif

    return value_read;
}

bool ODrive::getParamBool(String name) {
    uint32_t delay;
    sendReadParamRequest(name);
    bool value_read = readBool(true, delay);
#ifdef DEBUG_API
    Serial.print("get ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value_read);
#endif

    return value_read;
}

float ODrive::getVBusVoltage() {
    float voltage = getParamFloat("vbus_voltage");
    return voltage;
}

void ODrive::getVersion(uint16_t &major, uint16_t &minor, uint16_t &revision) {
    major = getParamInt("fw_version_major");
    minor = getParamInt("fw_version_minor");
    revision = getParamInt("fw_version_revision");
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
            Serial.print("did not receive 3 items but ");
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

void ODrive::setODriveParams() {
    // general
    setParamBool("config.enable_brake_resistor",true);
    setParamInt("config.brake_resistance",2);
    setParamAxisBool("config.startup_encoder_offset_calibration",false);

    // motor
    setParamAxisInt("motor.config.current_lim",10);
    setParamAxisInt("motor.config.pole_pairs",12);
    setParamAxisFloat("motor.config.torque_constant",8.27 / 300);
    setParamAxisInt("motor.config.motor_type",MOTOR_TYPE_HIGH_CURRENT);
    setParamAxisFloat("motor.config.resistance_calib_max_voltage",4.0);
    setParamAxisFloat("motor.config.calibration_current",4.0);
    setParamAxisFloat("motor.config.current_lim",4.0);
    setParamAxisFloat("motor.config.current_control_bandwidth ",200);

    // controller
    setParamAxisFloat("controller.config.vel_limit",10);
    setParamAxisFloat("controller.config.pos_gain",100);
    setParamAxisFloat("controller.config.vel_integrator_gain",0.02);
    setParamAxisFloat("controller.config.vel_gain",0.02);

    // encoder
    setParamAxisInt("encoder.config.cpr",20000);
    setParamAxisInt("encoder.config.direction",-1);
    setParamAxisBool("encoder.config.use_index",true);
    setParamAxisFloat("controller.config.input_filter_bandwidth",20);
    setParamAxisInt("controller.config.input_mode",INPUT_MODE_POS_FILTER);
}

void ODrive::setBaudRate() {
    // check the baud rate first, by reading system information
    serial_->begin(ODRIVE_HIGH_SERIAL_BAUD_RATE);
    serial_->flush();    
    String s1 = getInfoDump();
    int idx = s1.indexOf("Hardware");
    if (idx >= 0) {
        return; // all good
    } else {
        serial_->flush();
        // there was no "Hardware" in the system information, so either we had a timeout or a scrambled response
        // try the standard baudrate 
        serial_->end();
        serial_->begin(ODRIVE_STANDARD_SERIAL_BAUD_RATE);
        s1 = getInfoDump();
        int idx = s1.indexOf("Hardware");
         if (idx >= 0) {
            setParamInt("config.uart_a_baudrate",ODRIVE_HIGH_SERIAL_BAUD_RATE);
            saveConfiguration();
            serial_->flush();
            serial_->end();
            serial_->begin(ODRIVE_HIGH_SERIAL_BAUD_RATE);
            // final check
            s1 = getInfoDump();
            idx = s1.indexOf("Hardware");
            if (idx < 0) {
                Serial.println("Switch to 921600 baud did not work.");
                error = ERROR_COMM_ERROR; 
            }
         } else {
             Serial.println("Neither 115200 baud nor 921600 baud worked.");
             error = ERROR_COMM_ERROR; 
         }
    }

}

void ODrive::factoryReset() {
    eraseConfiguration(); // reset to factory settings, this will also reset the baud rate to ODRIVE_SERIAL_BAUD_RATE ally baud rate
    serial_->end();
    serial_->flush();    
    serial_->begin(ODRIVE_STANDARD_SERIAL_BAUD_RATE);

    // set all ODrive parameters specific to our motor and encoder
    setODriveParams();

    // raise baud rate and save to activate it 
    setParamInt("config.uart_a_baudrate",ODRIVE_HIGH_SERIAL_BAUD_RATE);
    saveConfiguration();

    // from now on, we need to communicate with the higher baud rate
    serial_->end();
    serial_->flush();    
    serial_->begin(ODRIVE_HIGH_SERIAL_BAUD_RATE);
}

void ODrive::calibrate(int motor_number) {
    Serial.print("Calibration process of ");
    Serial.print(motor_number);
    Serial.print(" \"");
    Serial.print(getName(motor_number));
    Serial.println("\"");

    clearErrors();

    // set all ODrive parameters specific to our motor and encoder
    Serial.println("Setting Parameters");
    setODriveParams();

    // start calibration process
    String axis = "axis" + String(motor_number); 
    Serial.println("Starting calibration procedure");
    requestedState(motor_number, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true /* wait until calibration is done */, 60 /* timeout [s] */);

    readError(motor_number, true);
    if (error == NO_ERROR) {
        setParamBool(axis + ".motor.config.pre_calibrated",true);
        setParamBool(axis + ".encoder.config.pre_calibrated",true);
        Serial.println("Saving calibration.");
        saveConfiguration();
        Serial.println("Done.");
    } else {
        Serial.print("Calibration failed.");
    }
}

/*****************************************/
/*** Class ODrives, managing a group of ODrives ***/
/*****************************************/

ODrives::ODrives() {
    num_odrives = 0;
    loopAvrTime_us = 0;
}

void ODrives::addODrive(HardwareSerial& ser, String name1, String name2) {
    if (num_odrives == MAX_ODRIVES) {
        Serial.println("Max number of ODrives exceeded.");
    }
    odriveSerial[num_odrives] = &ser;
    odrive[num_odrives].setName(name1, name2);
    num_odrives++;
}

ODrive& ODrives::operator[](uint8_t i) {
    if ( i > num_odrives) {
        Serial.print("Requested ");
        Serial.print(i);
        Serial.print(" Odrive, but only ");
        Serial.print(num_odrives);
        Serial.println(" are installed");
        delay(5000);
    } 
    return odrive[i];
}

void ODrives::setup() {
    for (int i = 0;i<num_odrives;i++) {
        odrive[i].setup(*odriveSerial[i]);
    }
}


void ODrives::loop() {
    // To utilise all hardware serials in parallel, data is send to all ODrives first,
    // then the results are collected
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
        odrive[i].setODriveParams();
    }
}

void ODrives::calibrate() {

    for (int i = 0;i<num_odrives;i++) {
        odrive[i].calibrate(0);
        odrive[i].calibrate(1);
    }
}