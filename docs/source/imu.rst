.. |br| raw:: html

  <br/>

Inertial Measurement Unit (IMU)
===============================

..  image:: /images/Lord_Microstrain_3DMCV5-IMU.png
   :width: 20%
   :alt: Lord Microstrain 3DM-CV5-10
   :target: https://www.microstrain.com/inertial-sensors/3dm-cv5-10
 	:class: float-left

The IMU is the source of the entire pipeline, therefore it is needs to be very fast as the pipeline is supposed to work with a decent frequency. Most hobby IMUs go up to 100Hz, which is okay, but above that things quickly get expensive. Heavy hearted I went with the `Lord Microstrain 3DM-CV5 IMU <https://www.microstrain.com/inertial-sensors/3dm-cv5-10>`_. Don't even ask.


Wiring the IMU
------------------

All Microstrain IMUs have this annoying 1.27mm IDC socket, which is hard to source. I used a `IDC(SWT) cable <https://www.adafruit.com/product/1675>`_ from Adafruit, cut off one end and connected it to a regular 5-pin JST XH connector:

.. image:: /images/IMU_Cable_Layout.png
	:width: 700
	:alt: 2x5 pin 1.27mm IDC cable
	:target: https://www.adafruit.com/product/1675

The pins in the column "IMU", are coming from the  `IMUs User Manual <https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf>`_, they specify the pins of the IMUs 2x5 IDC socket. The column PIN denotes the number of the JST socket.

.. image:: /images/3DM-CV5-10_Pin_layout.png
	:width: 500
	:alt: 3DM-CV5-10 User manual
	:target: https://www.microstrain.com/sites/default/files/3dm-cv5-10_user_manual_8500-0074_1.pdf

Before the IMU can be used, baud rate, frequency, and message format have to be defined. The manufacturer provides a tool called `Sensor Connect  <https://www.microstrain.com/software/sensorconnect>` for this, and a regular UART to USB adapter is used to connect it. (It is done only once, so it is perfectly okay to use flying wires like this)

.. image:: /images/IMU_to_USB.png
	:width: 700
	:alt: FTDI Adapter to USB


After pluging in, Sensorconnect should be able to connect to the device with the default baud rate of 115200. Some settings need to be done, at 1000Hz we need 916200 baud having in mind that one data packet is 48 bytes:

.. image:: /images/Sensorconnect_baudrate.png
	:width: 500
	:alt: Setting the baud rate

As message format coming from the IMU we need ther acceleration, the gyro, and the delta velocity (for the linear acceleration):

.. image:: /images/Sensorconnect_message_format.png
	:width: 500
	:alt: Set the message format

Finally, we need to define this as startup settings:

.. image:: /images/Sensorconnect_save_setting.png
	:width: 500
	:alt: Save the settings

And the data streaming should start right away after startup:

.. image:: /images/Sensorconnect_start_streaming.png
	:width: 500
	:alt: Start streaming after start


Filtering Sensor Data
---------------------

Filtering the data from an IMU is essential. Acceleration sensors are noisy, and gyros drift over time. 

The easiest way to solve this is a complementary filter that only takes the changes of the gyro into account, but uses the acceleration data as source for the angle. 

The implementation integrates the gyro data over time resulting in a drifting but non-noisy angle, then sends the result through a high pass filter to get rid of the drift, and fuses it with low passed acceleration data to get rid of its noise.

.. image:: /images/Complementary_Filter.png
	:width: 500
	:alt:  Complementary Filter

Luckily, it is easier to see in code:

.. code-block:: C++

	alpha = 0.98;
	angle = alpha * (angle + gyro * dT) + (1-alpha) * acceleration;

That looks too easy to be true, and it isn't. The cut-off frequency determining the factor :math:`{\alpha}` = 0.98 is hard to calibrate, and even worse, if the sensor has some dynamic behaviour like being non-linear , a static value is just wrong.

All this solved Rudolf E. Kálmán's famous `Kalman Filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Kalman1960.pdf>`_. A digestable description can be found `here <https://www.kalmanfilter.net/default.aspx>`_.

Multiple version of the filter are available, and the most common one is probably the Extended Kalman filter. However a rather new variant came up a while ago, which is the `Unscented Kalman filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf>`_, that is supposed to `provide a slightly better performance <https://www.gegi.usherbrooke.ca/LIV/index_htm_files/IEEEivsV2.pdf>`_.

Let's be honest, in the usecase of a quadruped the difference is neglectable. Anyhow, understanding that beast is a mental challenge, and who would not want that.


Fusing the state of the filter with incoming sensor data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: /images/RPY.png
	:width: 200
	:alt: Conventions
 	:class: float-left

Sensor fusion means merging the drifty gyro data with the noisy acceleration data. Spoiler alert, as if the IMU above is not yet expensive enough, we also need a magnetometer that is not only noisy, but also needs to be corrected because of the earth's tilted magnetic field. 
`Quaternions <https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>`_ avoid a  gimbal lock and are computational less intense(not really relevant actually, but lovely to work with). As usual we use the convention `roll, pitch, and yaw <https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)>`_ to avoid breaking fingers when picturing vectors.


The conventions used in the following are:

.. image:: /images/Quaternion_nomenklatur.png
	:width: 700
	:alt: Conventions

The state of the filter will be represented by a quaternion. The gyro is delivering angular rate, so we will need to rotate the state by these angles

.. image:: /images/Quaternion_derivative.png
	:width: 200
	:class: float-left

 |br| |br| Eq(1) |br| |br|



Considering the acceleration data, the quaternion should represent the rotation relative to the gravity vector |Gravity|. So we need to find a transformation matrix |AccelerationTransformation| that rotates the gravity vector such that it becomes our acceleration vector |QuatGravity|. This equation can be solved with something called the `Direct Cosine Matrix(DCM) <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_, leading to this equation

.. |Gravity| image:: /images/Gravity_vector.png
.. |QuatGravity| image:: /images/Quaternion_gravity.png
.. |AccelerationTransformation| image:: /images/Acceleration_Transformation.png


.. image:: /images/Quaternion_Acceleration_Fusion.png
	:width: 600
	:alt: accelerationfusion
	:class: float-left

 |br| |br| Eq(2) |br| |br|

Same thing happens to the data from the magnetic sensor. Again, the quaternion should represent the rotation relative to the magnetic vector |MagneticVector|. So we need to find a transformation matrix |AccelerationTransformation| that rotates the gravity vector such that it becomes our acceleration vector |QuatMagnetic|. The same nice `DCM Article <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_  leads to 

.. |MagneticVector| image:: /images/Magnetic_vector.png
.. |QuatMagnetic| image:: /images/Quaternion_Magneticfield.png
.. |AccelerationTransformation| image:: /images/MagneticField_Transformation.png

.. image:: /images/Quaternion_MagneticField_Fusion.png
	:width: 550
	:alt: MagneticFusion
	:class: float-left

|br| |br| |br| Eq(3) |br| |br| |br| |br|

Now we know how to change the state of our filter represented by a quaternion on the basis of incoming acceleration, gyro, and magnetometer data. 


The filter variables
^^^^^^^^^^^^^^^^^^^^

Lets continue with the space state description. It is approached as a descrete stochastic non-linear dynamic system:

.. image:: /images/State_Space_Equation.png
	:width: 700
	:alt: Conventions


As indicated before, our state |StateVariableX| is a quaternion representing the pose of the IMU. The input/control vector |StateVariableU| is the gyro data, since that is not noisy and most precise in the short term. Finally, the acceleration and magnetometer vectors represent the output vector |StateVariableY|.

.. image:: /images/State_Space_Variables.png
	:width: 400
	:alt: Conventions


.. math::
	
	x(k) = f(x(k-1),u(k-1))+v_{k}
.. math::

	u(k) = \bar{\omega} =  \begin{bmatrix} p  & q & r \end{bmatrix}  ^{T}
.. math::


	y(k) = \begin{bmatrix}{\bar{A}_{N}^{T}} & \bar{M}_{N}^{T} \end{bmatrix}^{T} = \begin{bmatrix} a_{x,N} & a_{y,N} & a_{z,N} & m_{x,N} & m_{y,N} & m_{z,N} \end{bmatrix}

.. |StateVariableX| image:: /images/state_variable_x.png
	:width: 35
.. |StateVariableU| image:: /images/state_variable_u.png
	:width: 35
.. |StateVariableY| image:: /images/state_variable_y.png
	:width: 35

The Kalman filter predicts the next state by the current state and input vector (gyro). Therefore, equation (1) gives 

.. image:: /images/next_state_prediction.png
	:width: 250
	:alt: Conventions

The change of the output is also already done with equation (2) and equation (3):

.. image:: /images/output_equation.png
	:width: 500
	:alt: Conventions

And that's all we need to feed into the Unscented Kalman filter.

The unscented Kalman filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The description of the algorithm has been borrowed from `here <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_:

First some definitions:

.. image:: /images/UKF_Definition.png
	:width: 700
	:alt: Conventions

Then, the UKF algorithm works like this:

.. image:: /images/UKF_Calculation.png
	:width: 700
	:alt: Conventions


Implementation
--------------

The implementation is hosted on the mainboard's Teensy 4.1, and as you might see from the algorithm above the Unscented Kalman filter is quite a lot of code. It is part of the entire main board, and the parts relevant for the IMU are `here <https://github.com/jochenalt/Lisbeth/tree/main/code/firmware/lib/IMU>`_. 

Contents: 

* `The Unscented Kalman filter <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/ukf.cpp>`_ 
   I used `this <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_ as a basis, but modified quite a lot to make it fast and robust
* `A matrix library <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/matrix.h>`_ 
	This is is coming from `here <https://github.com/pronenewbits>`_
* `The communication to the IMU <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/MicrostrainComm.cpp>`_ 
	This class implements Microstrains `data communciation protocol <https://github.com/jochenalt/Lisbeth/blob/main/datasheets/Microstrain%203DM-CV5-IMU/3DM-CV5-10%20IMU%20Data%20Communication%20Protocol%20Manualpdf.pdf>`_
* `The communication to the magnetometer <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/LIS3MDL.cpp>`_ 
   The magnetometer LIS3MDL communicates via I\ :sup:`2`\C with the main board. 
* `The integrating class IMUManager <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/IMUManager.cpp>`_ 
	Everthing is glued together in this class. It tkes care of the power management, i.e. it turns on/off the IMU and the magnetometer, watches 
	the incoming datastream, aligns the frames of the IMU and the magnetometer, converts the units into SI, and returns 
		* the pose in RPY convention in [rad] and quaternion,
		* the angular rate in[rad/s]
		* the linear acceleration without gravity vector in m/s\ :sup:`2`\

