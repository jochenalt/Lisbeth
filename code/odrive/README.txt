this folder contains the changed files of the firmware 0.5.4 that enable the speed of communication to 
6 ODrives up to 1000 Hz

The original firmware is in ODrive-Firmware-0.54-src.zip
I modified 

/tools/odrive - changed the version number to 1.55. As a result, in odrivetool if you type odrv0 you see
fw_version_major: 1 (uint8)
fw_version_minor: 5 (uint8)
fw_version_revision: 5 (uint8)
fw_version_unreleased: 0 (uint8)

/communication/ascii_protocol.cpp:

In void AsciiProtocol::cmd_set_position(char * pStr, bool use_checksum) 

I added another variant of the command "p <motornumber> <setpoint> <velocity> <torque>"

The new variant allows to set both motors at the same time
p <position0> <velocity0> <torque0> <position1> <velocity1> <torque1>
and returns the current values
<position0> <velocity0> <current0> <position1> <velocity1> <current1>

if only one motor is to be drive, you can pass 4 values
p <motornumber o or 1> <position> <velocity> <torque>
and it returns this motor's values
<position> <velocity> <current>


Code change in /communication/ascii_protocol.cpp in cmd_set_position
    if (numscan == 6) {
		Axis& axis0 = axes[0];
		Axis& axis1 = axes[1];

		axis0.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
		axis0.controller_.input_pos_ = pos_setpoint0;
		axis0.controller_.input_vel_ = vel_feed_forward0;
		axis0.controller_.input_torque_ = torque_feed_forward0;
		axis0.controller_.input_pos_updated();
		axis0.watchdog_feed();
		axis1.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
		axis1.controller_.input_pos_ = pos_setpoint1;
		axis1.controller_.input_vel_ = vel_feed_forward1;
		axis1.controller_.input_torque_ = torque_feed_forward1;
		axis1.controller_.input_pos_updated();
		axis1.watchdog_feed();
	} else {
