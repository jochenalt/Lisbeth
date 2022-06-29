Inertial Measurement Unit (IMU)
===============================

The IMU is the source of the entire pipeline, therefore it is needs to be very fast, if the pipeline is supposed to work with a decent frequency. Most hobby  IMU go up to 100Hz, which is okay, but above that things quickly get expensive. Heavy hearted I went with the `Lord Microstrain 3DM-CV5 IMU <https://www.microstrain.com/inertial-sensors/3dm-cv5-10>`_. Don't even ask.

.. image:: /images/Lord_Microstrain_3DMCV5-IMU.png
	:width: 150
	:alt: Lord Microstrain 3DM-CV5-10
	:target: https://www.microstrain.com/inertial-sensors/3dm-cv5-10


To set it up, it makes sense to try out the `SensorConnect <https://www.microstrain.com/software/sensorconnect>`_ first, that allows to set baud rate to 460800 baud and to see the accel and gyro live. 460800 baud is also needed by the firmware to establish a connection to the IMU  

.. raw:: html
	
    <iframe width="560" height="315"  allow="fullscreen; autoplay" src="https://www.microstrain.com/sites/default/files/sconn_widgets_demo.mp4?autoplay=1" frameborder="0"></iframe>

.. raw:: html

    <div><video loop="loop" autoplay="autoplay" muted="muted" style="width:100%"><source src="/sites/default/files/sconn_widgets_demo.mp4" type="video/mp4"></video></div>


In order to do so, it needs to be connected to a regular UART->USB converter. I used a `IDC(SWT) cable <https://www.adafruit.com/product/1675>`_ from Adafruit, cut off one end and connected it to a regular 5-pin JST XH connector:

.. image:: /images/IMU_Cable_Layout.png
	:width: 700
	:alt: 2x5 pin 1.27mm IDC cable
	:target: https://www.adafruit.com/product/1675

The pins shown are specified in the  `IMUs User Manual <https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf>`_.

.. image:: /images/3DM-CV5-10_Pin_layout.png
	:width: 700
	:alt: 3DM-CV5-10 User manual
	:target: https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf

Coming back to the setup. This needs to be done only once, so it is perfectly okay to use flying wires like this

.. image:: /images/IMU_to_USB.png
	:width: 700
	:alt: FTDI Adapter to USB


After plugging in, Sensorconnect is able to connect to the device with the default baud rate of 115200. I changed it to 460800 to be able to run it at 1000Hz.


