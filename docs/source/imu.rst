Inertial Measurement Unit (IMU)
===============================

The IMU is the source of the entire pipeline, therefore it is needs to be very fast as the pipeline is supposed to work with a decent frequency. Most hobby IMUs go up to 100Hz, which is okay, but above that things quickly get expensive. Heavy hearted I went with the `Lord Microstrain 3DM-CV5 IMU <https://www.microstrain.com/inertial-sensors/3dm-cv5-10>`_. Don't even ask.


Setting up the IMU
------------------

To set it up, it makes sense to try out `SensorConnect <https://www.microstrain.com/software/sensorconnect>`_ first, that allows to set the baud rate, and to see the accel and gyro live. 460800 baud is also needed by the firmware to establish a connection to the IMU. This is a little show-off from the vendor site how sensor connect looks like:   

.. |pic1| image:: /images/Lord_Microstrain_3DMCV5-IMU.png
   :width: 20%
   :alt: Lord Microstrain 3DM-CV5-10
   :target: https://www.microstrain.com/inertial-sensors/3dm-cv5-10

.. |pic2| image:: /videos/SensorConnect.gif
   :width: 50%
   :alt: Microstrain SensorConnect Application
   :target: https://www.microstrain.com/software/sensorconnect

|pic1| 			|pic2| 


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

Coming back to the setup. This needs to be done only once, so it is perfectly okay to use flying wires like this

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

Filtering the data from an IMU is essential. Acceleration sensors are noise, and gyros drift over time. 

The easiest way to solve this is a complementary filter that only takes the changes of the gyro into account, but uses the acceleration data as source for the angle. 

The implementation integrates the gyro data over time resulting in a drifting but non-noisy angle, then sends the result through a high pass filter to get rid of the drift, and fuses it with low passed acceleration data to get rid of its noise.

.. image:: /images/Complementary_Filter.png
	:width: 500
	:alt:  Complementary Filter

That looks too easy to be true, and it isn't. In reality the cut off frequency (in the code above that is determining the factor :math:`{\alpha}` = 0.98) is hard to calibrate, and even worse, if the sensor has some dynamic behaviour like not being linear or changes its noise, drift or behaviour, a static value is just arbitrary.

All this solved by Rudolf E. Kálmán's famous `Kalman Filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Kalman1960.pdf>`_. A digestable description can be found `here <https://www.kalmanfilter.net/default.aspx>`_.

Multiple version of the filter are available, and the most common one is probably the Extended Kalman filter. However a rather new variant came up a while ago, which is the `Unscented Kalman filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf>`_, that is supposed to 'provide a slightly better performance <https://www.gegi.usherbrooke.ca/LIV/index_htm_files/IEEEivsV2.pdf>'_.

Let's be honest, in the usecase of a quadruped the difference is neglectable. Anyhow, understanding that beast is a mental challenge, so I started it.


Fusing the sensor data
----------------------

Sensor fusion means merging the drifty gyro data with the noisy acceleration data. Spoiler alert, as if the IMU above is not yet expensive enough, we also need a magnetometer that is not only noisy, but also needs to be corrected because of the earth's tilted magnetic field. 
`Quaternions <https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>`_ avoid a  gimbal lock and are computational less intense(not really relevant actually). As usual we use the convention `roll, pitch, and yaw <https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)>`_ to avoid breaking fingers when picturing vectors.

The conventions used in the following are:
.. image:: /images/Quaternion_nomenklatur.png
	:width: 500
	:alt: Conventions

The state of the filter will be represented by a quaternion. The gyro is delivering angular rate, so we will need to rotate the state by these angles

.. image:: /images/Quaternion_derivative.png
	:width: 200
	:alt: Conventions

Considering the acceleration data, the quaternion should represent the rotation relative to the gravity vector|Gravity|. So we need to find a transformation matrix|AccelerationTransformation|that rotates the gravity vector such that it becomes our acceleration vector|QuatGravity|. This equation can be solved with something called the `Direct Cosine Matrix(DCM) <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_, leading to this equation

.. |Gravity| image:: /images/Gravity_vector.png
.. |QuatGravity| image:: /images/Quaternion_gravity.png
.. |AccelerationTransformation| image:: /images/Acceleration_Transformation.png

.. image:: /images/Quaternion_Acceleration_Fusion.png
	:width: 600
	:alt: Conventions


Same thing happens to the magnetic sensor, but with the magnetic vector. Again, the quaternion should represent the rotation relative to the magnetic vector|MagneticVector|. So we need to find a transformation matrix|AccelerationTransformation|that rotates the gravity vector such that it becomes our acceleration vector|QuatMagnetic|. The same nice `DCM Article <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_  leads to 

.. |MagneticVector| image:: /images/Magnetic_vector.png
.. |QuatMagnetic| image:: /images/Quaternion_Magneticfield.png
.. |AccelerationTransformation| image:: /images/MagneticField_Transformation.png

.. image:: /images/Quaternion_MagneticField_Fusion.png
	:width: 500
	:alt: Conventions


Now we know how to change the state of our filter represented by a quaternion on the basis of incoming acceleration, gyro, and magnetometer data. 



We then can re-describe the kinematics equations into a (continuous) nonlinear state space equation:
<p align="center"><img src="Continuous_State_Space_Equation.png" alt="Continuous State Space Equation"></p>

For sensor fusion algorithm, I use (discrete) Extended Kalman Filter and (discrete) Unscented Kalman Filter library I've made in [this repository](https://github.com/pronenewbits/Embedded_EKF_Library) (for EKF) and [this repository](https://github.com/pronenewbits/Embedded_UKF_Library) (for UKF). Because of that, we need to transform the equations above into the discrete form that can be used by EKF & UKF. Assuming ZOH-based sensor sampling, the discrete system (and the Jacobian for EKF) can be derived using Euler method as:
<p align="center"><img src="Discrete_State_Space_Equation.png" alt="Discrete State Space Equation"></p>

**Remark**: This is the simplest state space system for quaternion based AHRS using MEMS IMU sensor. Many publication use additional state to compensate gyroscope bias and accelerometer bias (some also set the magnetometer bias as another estimated state, not as parameters in the calibration phase), while others expand further by adding state for 3D speed or position (with additional GPS and pressure sensor, or even camera based machine vision) to make a complete guidance system. I hope by using this framework as a base, you can easily explore many different model.

&nbsp;

### Subsystem 2: The Magnetometer Calibration Algorithm

Each of the three sensors (accelerometer, gyroscope, magnetometer) have 12 parameters that needs to be calibrated to make sure sensor data have no offset or deformity. They are:

- 3 parameters of sensor bias <img src="eq_render/bias_sensor.gif" align="top"/>.
- 9 parameters of sensor deformity matrix that represented by 3x3 matrix <img src="eq_render/deformity_sensor.gif" align="middle"/>.

In general, if we have measurement from one of the three sensors <img src="eq_render/measured_sensor.gif" align="top"/>, then the true sensor data <img src="eq_render/true_sensor.gif" align="top"/> can be calculated by this equation:

<p align="center"><img src="eq_render/sensor_calib_eq.gif" align="middle"/></p>
**Note** that this equation doesn't consider stochastic noise (i.e. this is still a deterministic calibration equation), the stochastic noise will be dealt with sensor fusion algorithm described above.

&nbsp;

In total, we have 12 parameters x 3 sensors = 36 parameters total for IMU sensor (actually we haven't consider cross sensor calibration, e.g. gyroscopic sensitivity from linear acceleration parameters, or the temperature-dependence parameters. [Analog Devices made excellent article about them](https://www.analog.com/en/technical-articles/gyro-mechanical-performance.html)). So the total calibration parameters is more than that.

Fortunately, for many cases the magnetometer bias (the so called hard-iron bias) is the only dominating one. So by compensating the hard-iron bias we can get a good enough sensor data (at least good enough for our purpose). The other sensor's bias and sensor's structural error (hopefully) is not significant enough and can be considered general noise and (hopefully) will be compensated by sensor fusion algorithm. The hard-iron bias identification equation can be derived as:
<p align="center"><img src="Hard_Iron_Bias_Identification_by_Least_Squares.png" align="middle"/></p>

The equation above is an offline or batch identification (you take big enough sample, preferably more than 1000 samples, then using least-squares to identify the bias). The problem with batch identification is the needs for big memory, but we can use [Recursive Least Squares](https://en.wikipedia.org/wiki/Recursive_least_squares_filter) as the online version to calculate the bias (this way we don't need to store all samples data):
<p align="center"><img src="Hard_Iron_Bias_Identification_by_Recursive_Least_Squares.png" align="middle"/></p>

**Note**: The RLS algorithm above is general enough that you could also use it to identify not only hard-iron bias, but also soft-iron bias (or the deformity matrix described above). For example, [this paper](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7014484/pdf/sensors-20-00535.pdf) explore further by using RLS/ML combination to calculate 12 magnetometer calibration parameters.


