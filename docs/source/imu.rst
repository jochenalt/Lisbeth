Inertial Measurement Unit (IMU)
===============================

The IMU is the source of the entire pipeline, therefore it is needs to be very fast, if the pipeline is supposed to work with a decent frequency. Most hobby  IMU go up to 100Hz, which is okay, but above that things quickly get expensive. Heavy hearted I went with the `Lord Microstrain 3DM-CV5 IMU <https://www.microstrain.com/inertial-sensors/3dm-cv5-10>`_. Don't even ask.

.. image:: /images/Lord_Microstrain_3DMCV5-IMU.png
	:width: 150
	:alt: Lord Microstrain 3DM-CV5-10
	:target: https://www.microstrain.com/inertial-sensors/3dm-cv5-10


To set it up, it makes sense to try out the `SensorConnect <https://www.microstrain.com/software/sensorconnect`_ first, that allows to set baud rate (460800 baud) and to see the accel and gyro live

.. video:: /videos/SensorConnect_demo.mp4
	:width: 300
	:alt: Microstrain SensorConnect
	:target: https://www.microstrain.com/software/sensorconnect

In order to do so, it needs to be connected to a regular UART->USB converter. I used a `IDC(SWT) cable https://www.adafruit.com/product/1675`_ from Adafruit, cut of one end and connected it to a regular 5-pin JST XH connector:

.. image:: /images/IMU_Cable_Layout.png
	:width: 450
	:alt: 2x5 pin 1.27mm IDC cable
	:target: https://www.adafruit.com/product/1675

The pins stated below are coming from the `IMUs User Manual https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf`_.

.. image:: /images/3DM-CV5-10_Pin_layout.png
	:width: 500
	:alt: 3DM-CV5-10 User manual
	:target: https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf


