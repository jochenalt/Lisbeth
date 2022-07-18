
Implementation
==============

The implementation is hosted on the mainboard's Teensy 4.1, and as you might see from the algorithm above the Unscented Kalman filter is quite a lot of code. It is part of the  main board, the relevant part of the firmware is `this  <https://github.com/jochenalt/Lisbeth/tree/main/code/firmware/lib/IMU>`_. 

The overall structure looks like this:

.. image:: /images/IMU_SW_Architecture.png
	:width: 700
	:alt: Conventions


**Contents**

*  The unscented Kalmanfilter is implemented in the class `UKF <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/ukf.cpp>`_. I used `this <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_ as a basis, but modified quite a lot to make it fast and robust

*  As matrix library I used `matrix <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/matrix.h>`_ , coming from `here <https://github.com/pronenewbits>`_

*  The communication with a Microstrain device is implemented in the class `Microstrain <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/MicrostrainComm.cpp>`_ . This class assumes that the IMU is preconfigured, such that it only reads an incoming datastream. However, it has some ressilience built in, like constant checking of the timing, checksums, recovery of a a lost communcation, and resetting the  device with a separate power pin. It  implements Microstrains `data communciation protocol <https://github.com/jochenalt/Lisbeth/blob/main/datasheets/Microstrain%203DM-CV5-IMU/3DM-CV5-10%20IMU%20Data%20Communication%20Protocol%20Manualpdf.pdf>`_.

*  The communication with a Microstrain device is implemented in the class `Microstrain <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/MicrostrainComm.cpp>`_ . This class assumes that the IMU is preconfigured, such that it only reads an incoming datastream. However, it has some ressilience built in, like constant checking of the timing, checksums, recovery of a a lost communcation, and resetting the  device with a separate power pin. It  implements Microstrains `data communciation protocol <https://github.com/jochenalt/Lisbeth/blob/main/datasheets/Microstrain%203DM-CV5-IMU/3DM-CV5-10%20IMU%20Data%20Communication%20Protocol%20Manualpdf.pdf>`_.

*  The magnetometer is the popular LIS3MLD device. Communication over I\ :sup:`2`\C happens in the class `LIS3MLD  <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/LIS3MDL.cpp>`_.

*  The magnetometer is the popular LIS3MLD device. Communication over I\ :sup:`2`\C happens in the class `LIS3MLD  <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/LIS3MDL.cpp>`_.

*  Finally, all devices and filters are glued together in the class IMUManager  and the magnetometer, aligns the frames, filters the output and watches that everything is working correctly. The outcome is a datastream that returns
      * the pose in RPY convention in [rad] and quaternion,
      * the angular rate in [rad/s]
      * the linear acceleration with the gravity vector removed in [m/s\ :sup:`2`\]

*  Finally, all devices and filters are glued together in the class IMUManager  `here <https://github.com/pronenewbits>`_ and the magnetometer, aligns the frames, filters the output and watches that everything is working correctly. The outcome is a datastream that returns
      * the pose in RPY convention in [rad] and quaternion,
      * the angular rate in [rad/s]
      * the linear acceleration with the gravity vector removed in [m/s\ :sup:`2`\]

*  Finally, all devices and filters are glued together in the class `IMUManager <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/IMUManager.cpp>`_. It takes care of the power management of the IMU and the magnetometer, aligns the frames, filters the output and watches that everything is working correctly. The outcome is a datastream that returns
      * the pose in RPY convention in [rad] and quaternion,
      * the angular rate in [rad/s]
      * the linear acceleration with the gravity vector removed in [m/s\ :sup:`2`\]

