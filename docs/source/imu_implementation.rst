
Implementation
==============

The implementation is hosted on the mainboard's Teensy 4.1, and as you might see the unscented Kalman filter is quite a lot of code. The IMU management is only a part of the whole firmware of the board, source can be found `here  <https://github.com/jochenalt/Lisbeth/tree/main/code/firmware/lib/IMU>`_. 

The overall structure looks like this:

.. image:: /images/IMU_SW_Architecture.png
	:width: 700
	:alt: Conventions


**Contents**

*  The unscented Kalmanfilter is implemented in the class `UKF <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/ukf.cpp>`_. I used `this <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_ as a basis, but modified some bits about robustness and speed. That is the only bit that cannot be understood without reading the `UKF algorithm  <https://lisbeth.readthedocs.io/en/latest/imu_filter.html#filtering-imu-data>`_, the rest of the code is rather straight forward and should be easy to grasp.  

*  As matrix stuff I used `matrix <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/matrix.h>`_ , originating from `here <https://github.com/pronenewbits/Arduino_AHRS_System/blob/master/ahrs_ekf/matrix.h>`_

*  The communication with a Microstrain device is implemented in the class `Microstrain <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/MicrostrainComm.cpp>`_ . This class assumes that the IMU is preconfigured, such that it only reads an incoming datastream. It has some ressilience built in, like constant checking of the timing, checksums, recovery of a lost communcation, and resetting the  device with a separate pin. It implements Microstrains `data communciation protocol <https://github.com/jochenalt/Lisbeth/blob/main/datasheets/Microstrain%203DM-CV5-IMU/3DM-CV5-10%20IMU%20Data%20Communication%20Protocol%20Manualpdf.pdf>`_, which is tedious to programme, and Microstrain should be punished for not providing a µC library! 

*  The magnetometer is the popular LIS3MLD device. Communication over I\ :sup:`2`\C happens in the class `LIS3MLD  <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/LIS3MDL.cpp>`_. The device is driven with the *High performance mode*  at 155 Hz. Since the IMU runs at 1000Hz, the magnetometer values used are not always refreshed in a loop.

*  Finally, all devices and filters are glued together in the class `IMUManager <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/IMUManager.cpp>`_. It takes care of the power management and timeouts, aligns the frames of the IMU and the magnetometer, filters the output and watches the sensor's datastream in realtime. The outcome is a datastream of the pose [rad & quaternion], the angular rate [rad/s], and the linear acceleration (with the gravity vector removed) in [m/s\ :sup:`2`\] at a rate of 1000Hz.

The performance is okay, running the filter takes less than 200µs, therefore the IMU uses up around 20% of the Teensy's CPU.
