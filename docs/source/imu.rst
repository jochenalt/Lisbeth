.. |br| raw:: html

  <br/>

Inertial Measurement Unit
=========================

..  image:: /images/Lord_Microstrain_3DMCV5-IMU.png
   :width: 20%
   :alt: Lord Microstrain 3DM-CV5-10
   :target: https://www.microstrain.com/inertial-sensors/3dm-cv5-10
 	:class: float-left

The IMU is the source of the entire pipeline, therefore it is needs to be very fast as the pipeline is supposed to work with a decent frequency. Most hobby IMUs go up to 100Hz, which is okay, but above that things quickly get expensive. Heavy hearted I went with the `Lord Microstrain 3DM-CV5 IMU <https://www.microstrain.com/inertial-sensors/3dm-cv5-10>`_. Don't even ask.


Wiring the IMU
--------------

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


Configuring the IMU
--------------------

Before the IMU can be used, baud rate, frequency, and message format have to be defined, since the software running in the mainboard assumes that the IMU starts streaming data when powered. The manufacturer provides a tool called `Sensor Connect  <https://www.microstrain.com/software/sensorconnect>` for this, and a regular UART to USB adapter is used to connect it. (It is done only once, so it is perfectly okay to use flying wires like this)

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

Mounting the IMU
----------------

.. image:: /images/Magnetometer_sockets.png
	:width: 200
	:alt: Original
 	:class: float-left


I bundled the Microstrain IMU with a low-cost magnetometer `LIS3DM <https://www.adafruit.com/product/4022>`_. The yaw angle is not really relevant for a quadruped, almost neglectable actually. Anyhow, I found the drift in the z-axis annoying, so I added the magnetometer and printed a holder that clamps it on to of the IMU. 

.. image:: /images/IMU_and_magnetometer_mounted.png
	:width: 300
	:alt: Original
 	:class: float-right

To fit into the holder, the original sockets need to be removed. The final package became a bit bulky, but IMU and Magnetometer are nicely covered. 


Filtering IMU  Data
-------------------


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



.. list-table:: **Conventions**:
   :widths: 25 75

   * - :math:`\bar{q} = \begin{bmatrix}q_{0} & q_{1} & q_{2 } & q_{3} \end{bmatrix}^{T}`
     - Quaternion representing the IMU's pose in the world frame. :math:`\left \| \bar{q} \right \| = 1`
   * - :math:`\overline{\omega } =\begin{bmatrix} p & q & r \end{bmatrix}^{T}`
     - angular rate of the gyro in [rad/s] in the IMUs frame
   * - :math:`\overline{A} =\begin{bmatrix} a_{x} & a_{y} & a_{z} \end{bmatrix}^{T}`
     - acceleration vector from sensor in [:math:`\frac{g}{s^{2}}`] in the IMUs frame
   * - :math:`\overline{M} =\begin{bmatrix} m_{x} & m_{y} & m_{z} \end{bmatrix}^{T}`
     - magnetic vector from magnetometer in [uT] (*micro Tesla*)in the IMUs frame
   * - :math:`\overline{G} =\begin{bmatrix} 0 & 0 & g \end{bmatrix}^{T}`
     - gravity vector in :math:`\begin{bmatrix}\frac{m}{s^{2}}\end{bmatrix}` in the earths/world frame 
   * - :math:`\overline{B} =\begin{bmatrix} B_{0x} & B_{0y} & B_{0z} \end{bmatrix}^{T}`
     - earths magnetic vector in [uT] in the earths/world frame

The state of the filter will be represented by a quaternion. When the gyro is delivering a datapoint of angular rate, we will need to rotate the state by these angles per dt. That's done by :math:`\frac{d\bar{q}(t)}{dt} = \frac{1}{2}\bar{q}(t) \otimes \bar{\omega }(t)`, so we get

.. math:: 
	:label: quaternion_derivative

	\frac{d\bar{q}(t)}{dt} = \frac{1}{2}\begin{bmatrix}
	0 & -p &-q  &-r & \\ 
	p & 0  & r  & -q& \\ 
	q & -r & 0  & p & \\
	r & q  & -p & 0 &
	\end{bmatrix}
	\begin{bmatrix}
	 q_{0}  \\ 
	 q_{1}  \\ 
	 q_{2}  \\
	  q_{3}
	\end{bmatrix}


Now we do the same with the acceleration, i.e. a new datapoint needs to be fused with the state. The quaternion should represent the rotation relative to the gravity vector :math:`\bar{G} = \begin{bmatrix} 0 & 0 & g\end{bmatrix}^{T}`. So we need to find a transformation matrix :math:`C_{n}^{b}` that rotates the gravity vector in a way that it aligns with the acceleration vector :math:`\bar{A}_{N} = C_{n}^{b}\bar{G}_{N}`. This equation can be solved with something called the `Direct Cosine Matrix(DCM) <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_, leading to this equation

.. math:: 
	:label: quarternionaccelerationfusion

	\begin{bmatrix}
	a_{x,N}\\ 
	a_{y,N}\\ 
	a_{z,N}
	\end{bmatrix} 
	&= \begin{bmatrix}
	 q_{0}^{2} + q_{1}^{2} - q_{2}^{2} - q_{3}^{2}& 2(q_{1}q_{1} + q_{0}q_{3}) & 2(q_{1}q_{3} - q_{0}q_{2})\\ 
	 2(q_{1}q_{2} - q_{0}q_{3})&  q_{0}^{2} - q_{1}^{2} + q_{2}^{2} - q_{3}^{2} & 2(q_{2}q_{3} + q_{0}q_{1})\\ 
	 2(q_{1}q_{3} + q_{0}q_{2}) & 2(q_{2}q_{3} - q_{0}q_{1}) &  q_{0}^{2} - q_{1}^{2} - q_{2}^{2} + q_{3}^{2}
	\end{bmatrix}
	\begin{bmatrix}
	0\\ 
	0\\ 
	1\\
	\end{bmatrix}\\
	&= 
	\begin{bmatrix}
	2(q_{1}q_{3} - q_{0}q_{2})\\
	2(q_{1}q_{3} - q_{0}q_{1})\\
	q_{0}^{2} - q_{1}^{2} - q_{2}^{2} + q_{3}^{2}
	\end{bmatrix}


Same thing happens to the data from the magnetic sensor. Again, the quaternion should represent the rotation relative to the magnetic vector :math:`\bar{M} = \begin{bmatrix}m_{x}&m_{z}&m_{z}\end{bmatrix} ^{T}`. So we need to find a transformation matrix :math:`C_{n}^{b}` that rotates the gravity vector such that it becomes our acceleration vector :math:`\overline{M_{N}} = C_{n}^{b }\overline{B_{0,N}}`. The same nice `DCM Article <https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/>`_  leads to 


.. math:: 

	\begin{bmatrix}
	m_{x,N}\\ 
	m_{y,N}\\ 
	m_{z,N}
	\end{bmatrix} 
	= \begin{bmatrix}
	 q_{0}^{2} + q_{1}^{2} - q_{2}^{2} - q_{3}^{2}& 2(q_{1}q_{1} + q_{0}q_{3}) & 2(q_{1}q_{3} - q_{0}q_{2})\\ 
	 2(q_{1}q_{2} - q_{0}q_{3})&  q_{0}^{2} - q_{1}^{2} + q_{2}^{2} - q_{3}^{2} & 2(q_{2}q_{3} + q_{0}q_{1})\\ 
	 2(q_{1}q_{3} + q_{0}q_{2}) & 2(q_{2}q_{3} - q_{0}q_{1}) &  q_{0}^{2} - q_{1}^{2} - q_{2}^{2} + q_{3}^{2}
	\end{bmatrix}
	\begin{bmatrix}
	B_{0x,N}\\ 
	B_{0y,N}\\ 
	B_{0z,N}\\
	\end{bmatrix}\\
	= 
	\begin{bmatrix}
	B_{0x,N}(q_{0}^{2} + q_{1}^{2} - q_{2}^{2} - q_{3}^{2}) &+ B_{0y,N}(2(q_{1}q_{2} - q_{0}q_{3})) &+ B_{0z,N}(2(q_{1}q_{3} - q_{0}q_{2}))\\
	 B_{0x,N}(2(q_{1}q_{2} - q_{0}q_{3})) &+  B_{0y,N}(q_{0}^{2} - q_{1}^{2} + q_{2}^{2} - q_{3}^{2}) &+ B_{0z,N}(2(q_{2}q_{2} + q_{0}q_{3}))\\
	 B_{0x,N}(2(q_{1}q_{3} + q_{0}q_{2})) &+ B_{0y,N}(2(q_{2}q_{3} - q_{0}q_{1})) &+ B_{0z,N}(q_{0}^{2} - q_{1}^{2} - q_{2}^{2} + q_{3}^{2})
	\end{bmatrix}

Now we know how to change the state of our filter represented by a quaternion on the basis of incoming acceleration, gyro, and magnetometer data. 


The Filter Variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's continue with the space state description. In general, we approach the problem as a descrete stochastic non-linear dynamic system:

.. math:: 

	x(k) &= f(x(k-1), u(k-1))+v_{k} \\
	y(k) &= h(x(k))+n_{k}\\

where :math:`x\in R^{N}, u\in R^{M}, z\in R^{z}, v_{k}` is the process noise, and :math:`n_{k}` is the observation noise.

In our case the state :math:`x(k)` is a quaternion representing the pose of the IMU. Our input/control vector :math:`u(k)` is the gyro data that is used for changes in the short term. Finally, the acceleration and magnetometer vectors represent the output vector :math:`y(k)` that is compensating the gyro's drift.

.. math:: 

	\\
	x(k) &= f(x(k-1),u(k-1))+v_{k} \\
	u(k) &= \bar{\omega} =  \begin{bmatrix} p  & q & r \end{bmatrix}  ^{T} \\
	y(k) &= \begin{bmatrix}{\bar{A}_{N}^{T}} & \bar{M}_{N}^{T} \end{bmatrix}^{T} = \begin{bmatrix} a_{x,N} & a_{y,N} & a_{z,N} & m_{x,N} & m_{y,N} & m_{z,N} \end{bmatrix}


The Kalman filter predicts the next state by fusing the current state with the input vector (gyro). Therefore, equation (1) gives 

.. math::
	x(k) = x(k-1) + \frac{\Delta t}{2}\begin{bmatrix}
	-p q_{1} - q q_{2} - r q_{3}\\ 
	-p q_{0} + r q_{2} - q q_{3}\\ 
	q q_{0} - r q_{1} + p q_{3}\\ 
	r q_{0} - q q_{1} - p q_{2}
	\end{bmatrix}

The modification of the output is done with equation (2) and equation (3):

.. math::

	y(k) =\begin{bmatrix}
	2(q_{1}q_{3} - q_{2}q_{2})\\ 
	2(q_{2}q_{3} + q_{0}q_{1})\\ 
	q_{0}^2 -q_{1}^2 -q_{2}^2 + q_{3}^2\\ 
	B_{0x,N}(q_{0}^{2} + q_{1}^{2} - q_{2}^{2} - q_{3}^{2}) &+ B_{0y,N}(2(q_{1}q_{2} - q_{0}q_{3})) &+ B_{0z,N}(2(q_{1}q_{3} - q_{0}q_{2}))\\
	B_{0x,N}(2(q_{1}q_{2} - q_{0}q_{3})) &+  B_{0y,N}(q_{0}^{2} - q_{1}^{2} + q_{2}^{2} - q_{3}^{2}) &+ B_{0z,N}(2(q_{2}q_{2} + q_{0}q_{3}))\\
	B_{0x,N}(2(q_{1}q_{3} + q_{0}q_{2})) &+ B_{0y,N}(2(q_{2}q_{3} - q_{0}q_{1})) &+ B_{0z,N}(q_{0}^{2} - q_{1}^{2} - q_{2}^{2} + q_{3}^{2})
	\end{bmatrix}

And that's all we need to feed into the Unscented Kalman filter.

The Unscented Kalman Algorithm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


The algorithm as described in `A new extension to the Kalman filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf>`_ is listed below,  I borrowed it from `here <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_ .(Frustratingly, it is almost impossible to understand that without having the standard Kalman filter digested)


.. list-table:: Variables used in the Unscented Kalman Filter
   :widths: 25 75

   * - 
     - **Classic Kalman variables**
   * - :math:`\hat{x}(k|k-1)`
     - Prediction of the state variable :math:`x(k)` based on information we know from the previous sampling time (i.e. the estimated state variable  :math:`\hat{x}(k-1)` and :math:`u(k-1)`). We'll get these values at the predciction step, calculated based on the non-linear function :math:`f` defined above. The state variable is a quaternion representing the pose of the IMU. 
   * - :math:`\hat{x}(k|k)`
     - The updated prediction of the state variable :math:`x(k)` by adding information from  :math:`y(k)` which is the acceleration and magnetometer data. We will fetch these values at the correction step. *Note: In the next loop,* :math:`\hat{x}(k|k)` *will become* :math:`\hat{x}(k-1)`
   * - :math:`P(k|k-1)`
     - Covriance matrix of the predicted state variable :math:`x(k)`, defined like :math:`\hat{x}(k|k-1)` above
   * - :math:`P(k|k)`
     - Covariance matrix of the updated state variable :math:`x(k)`, defined like :math:`\hat{x}(k|k)` above
   * - :math:`\hat{y}(k)`
     - Prediction of the output/measurement variable :math:`y(k)`.
   * - :math:`R_{v}`
     - Process noise covariance matrix built as diagonal matrix round :math:`v_{k}` information.
   * - :math:`R_{n}`
     - Measurement noise covariance matrix built as diagonal matrix around :math:`n_{k}`.
   * - 
     - **Sigma-point variables, in the implementation we use** :math:`(2N+1)` **points**
   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`
   * - 
     - **Supporting variables**
   * - :math:`P_{R}(k)`
     - Covariance matrix of the predicted measurement  :math:`y(k)`
   * - :math:`P_{XY}(k)`
     - Cross covariance matrix between predicted state variable :math:`x(k)` and predicted measurement :math:`x(k)`.
   * - :math:`W_{m}`
     - First order weights matrix.
   * - :math:`W_{c}`
     - Second order weights matrix.

Then, the UKF algorithm works like this:

**Initialisation**


1. Set :math:`\hat{x}(k=0) = E\left [x(k=0)  \right ]` 

2. Set :math:`P(k=0) = E\left [(x(k=0) - \hat{x}(k=0))(x(k=0) - \hat{x}(k=0) )^{T} \right ]` 

3. Set noise covariance matrices of the gyro :math:`R_{v} = diag(R_{v}, R_{v},R_{v})`, and the noise of our gyro being :math:`10^{-7}` according to the datasheet.

4.  Set noice covariance matrices of accelerometer and magnetometer to :math:`R_{n} = diag(R_{acc1}, R_{acc},R_{acc}, R_{mag}, R_{max},R_{mag})`, with :math:`R_{acc} = 0.00000316` and :math:`R_{mag} = 0.00000316`, again from the datasheet.

5. Calculate :math:`\alpha, \kappa,\beta, \gamma` constants, first order weights :math:`W_{m}` and second order weights :math:`W_{c}`

   .. math::

 	   \lambda = \lambda = \alpha^{2} (N + \kappa) -N

   .. math::

 	   \gamma = \sqrt{N+\alpha}

   .. math::

 	   W_{m}= \gamma = \sqrt{N+\alpha}

   .. math::

	   W_{m} = \begin{bmatrix}\frac{\lambda}{N+\lambda} & \frac{1}{2(N+\lambda)}  & ... & \frac{1}{2(N+\lambda)} \end{bmatrix} , dim(W_{m}) = 7 

   .. math::

	   W_{c} = \begin{bmatrix}\frac{\lambda}{N+\lambda} + (1-\alpha^{2} + \beta) & \frac{1}{2(N+\lambda)}  & ... & \frac{1}{2(N+\lambda)} \end{bmatrix} , dim(W_{c}) = 7

**The following has to be done repeatedly whenever a new data point is sampled**

#. Construct the sigma-points:

   .. math::

	   X(k-1) = \begin{bmatrix} \hat{x}(k-1) & \hat{x}(k-1) + \gamma \sqrt{P(k-1)}& \hat{x}(k-1) - \gamma \sqrt{P(k-1)} \end{bmatrix}

#. Do the unscented Transformation of the sigma points :math:`X(k-1)`

   Propagate :math:`X(k-1)` through non-linear function :math:`f`. :math:`f` is applied 7 times to the column submatrix of :math:`X(k-1)`

   .. math::
	
	    X(k) = f(X(k-1))

   Calculate :math:`\hat{x}(k|k-1)` as a weighted mean of :math:`X(k-1)`:

   .. math::
	
	    \hat{x}(k|k-1) = \sum_{i=1}^{2N+1=7} (W_{m,i} X_{i}(k))

   Calculate the covariance matrix of the predicted state variable :math:`x(k)`. This operation is substracting the submatrix of :math:`X(k)` by :math:`\hat{x}(k)` repeated 2N+1=7 times 

   .. math::
	
	    \Delta X = \left [ X(k) - \hat{x}(k|k-1) \right ]

   .. math::
	
	    P(k|k-1) = \sum_{i=1}^{2N+1=7} (W_{c,i} \left [\Delta X  \right ] \left [ \Delta X \right ]^{T} + R_{v})

#. Do the unscented Transformation of the sigma-points :math:`X(k)`

   Propagate :math:`X(k)` through non-linear function :math:`h`. (:math:`h` is applied 7 times to the column submatrix of :math:`X(k)`)

   .. math::
	
	    Y(k) = h(X(k))

   Calculate :math:`\hat{y}(k|k-1)` as a weighted mean of :math:`Y(k)`:

   .. math::
	
	    \hat{y}(k|k-1) = \sum_{i=1}^{2N+1=7} (W_{m,i} Y_{i}(k))

   Calculate the covariance matrix of the predicted measurement :math:`y(k)`. This operation is substracting the submatrix of :math:`Y(k)` by :math:`\hat{y}(k)` repeated 2N+1=7 times 

   .. math::
	
	    \Delta Y = \left [ Y(k) - \hat{y}(k|k-1) \right ]


   .. math::
	
	    P_{y}(k) = \sum_{i=1}^{2N+1=7} (W_{c,i} \left [\Delta Y  \right ] \left [ \Delta Y \right ]^{T} + R_{n})

#. Calculate cross-covariance matrix:

   .. math::
	
	    P_{XY}(k) = \sum_{i=1}^{2N+1=7} (W_{c,i} \left [\Delta X  \right ] \left [ \Delta Y \right ]^{T})


#. Calculate the Kalman gain:

   .. math::
	
	    K = P_{XY}(k) (P_{Y}(k))^{-1}

#. Update the estimated state variable

   .. math::
	
	    \hat{x}(k|k) = \hat{x}(k|k-1) + K(y(k)-\hat{y}(k))


#. Update the covariance matrix:

   .. math::

	    P(k|k) = \hat{x}(k|k-1)  K(y(k) - \hat{y}(k))



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

*  The magnetometer is the popular LIS3MLD device. Communication over I\ :sup:`2`\C happens in the class `LIS3MDL  <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/LIS3MDL.cpp>`_. The device is driven with the *High performance mode*  at 155 Hz. Since the IMU runs at 1000Hz, the magnetometer values used are not always refreshed in a loop.

*  Finally, all devices and filters are glued together in the class `IMUManager <https://github.com/jochenalt/Lisbeth/blob/main/code/firmware/lib/IMU/IMUManager.cpp>`_. It takes care of the power management and timeouts, aligns the frames of the IMU and the magnetometer, filters the output and watches the sensor's datastream in realtime. The outcome is a datastream of the pose [rad & quaternion], the angular rate [rad/s], and the linear acceleration (with the gravity vector removed) in [m/s\ :sup:`2`\] at a rate of 1000Hz.

The performance is okay, running the filter takes less than 200µs, therefore the IMU uses up around 20% of the Teensy's CPU.
