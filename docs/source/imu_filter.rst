Filtering IMU  Data
===============================


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
--------------------------------------------------------

.. image:: /images/RPY.png
	:width: 200
	:alt: Conventions
 	:class: float-left

Sensor fusion means merging the drifty gyro data with the noisy acceleration data. Spoiler alert, as if the IMU above is not yet expensive enough, we also need a magnetometer that is not only noisy, but also needs to be corrected because of the earth's tilted magnetic field. 
`Quaternions <https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>`_ avoid a  gimbal lock and are computational less intense(not really relevant actually, but lovely to work with). As usual we use the convention `roll, pitch, and yaw <https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)>`_ to avoid breaking fingers when picturing vectors.




.. list-table:: **Conventions**:
   :header-rows: 1

   * - Symbol
     - Meaning
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
--------------------

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

The Unscented Kalman filter
---------------------------


The algorithm as described in `A new extension to the Kalman filter <https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf>`_ is listed below,  I borrowed it from `here <https://github.com/pronenewbits/Embedded_UKF_Library/blob/master/README.md>`_ .(Frustratingly, it is almost impossible to understand that without having the standard Kalman filter digested)

.. list-table:: **Sigma-point variables, in the implementation we use** :math:`(2N+1)` **points**
   :header-rows: 0
   :widths: 25 75 

   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`


.. list-table:: **Sigma-point variables, in the implementation we use (2N+1) points**
   :header-rows: 0
   :widths: 25 75 

   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`


.. list-table:: **Sigma-point variables, in the implementation we use (2N+1) points**
   :widths: 25 75 

   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`

.. list-table:: **Sigma-point variables, in the implementation we use (2N+1) points**

   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`


The state of the filter will be represented by a quaternion. When the gyro is delivering a datapoint of angular rate, we will need to rotate the state by 

First some more definitions:

.. list-table:: **Classic Kalman variables**
   :header-rows: 0
   :widths: 25 75
 
   * - :math:`\hat{x}(k|k-1)`
     - Prediction of the state variable :math:`x(k)` based on information we know from the previous sampling time (i.e. the estimated state variable  :math:`\hat{x}(k-1)` and :math:`u(k-1)`). We'll get these values at the predciction step, calculated based on the non-linear function :math:`f` defined above. 
   * - :math:`\hat{x}(k|k)`
     - The updated prediction of the state variable :math:`x(k)` by adding information we know from this sampling time (i.e. the observed variable value :math:`y(k)`) We will get these values at correction step, calculated basied on the Kalman gain. *Note: At the next sampling time,* :math:`\hat{x}(k|k)` *will become* :math:`\hat{x}(k-1)`
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


.. list-table:: **Sigma-point variables, in the implementation we use** :math:`(2N+1)` **points**
   :header-rows: 0
   :widths: 25 75 

   * - :math:`X(k-1)`
     - The sigma-points constructed from :math:`\hat{x}(k-1)` and  :math:`P(k-1)`
   * - :math:`X(k)`
     - The sigma-points  :math:`X(k-1)` propagated by non-linear function :math:`f`
   * - :math:`Y(k)`
     - The sigma-points  :math:`X(k)` propagated by non-linear function :math:`h`


.. list-table:: **Supporting variables**
   :header-rows: 0
   :widths: 25 75 

   * - :math:`P_{R}(k)`
     - Covariance matrix of the predicted measurement  :math:`y(k)`
   * - :math:`P_{XY}(k)`
     - Cross covariance matrix between predicted state variable :math:`x(k)` and predicted measurement :math:`x(k)`.
   * - :math:`W_{m}`
     - First order weights mnatrix.
   * - :math:`W_{c}`
     - Second order weights mnatrix.


.. image:: /images/UKF_Definition.png
	:width: 700
	:alt: Conventions

Then, the UKF algorithm works like this:

.. image:: /images/UKF_Calculation.png
	:width: 700
	:alt: Conventions

