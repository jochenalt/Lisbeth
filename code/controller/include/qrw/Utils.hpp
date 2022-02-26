
#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <string>
#include "qrw/Types.h"
bool file_exists (const std::string& name);

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
Vector3 quaternionToRPY(Eigen::Quaterniond quat);
Vector3 cross3(Vector3 left, Vector3 right);
bool array_equal(Vector4 a, Vector4 b);


/**
 * Complementary filter for vectors.
 * Used to filter body velocity and position in x,y,z.
 * Computation uses x and dx, since in both cases the derivative is already available and does not need to be
 * calculated multiple times.
 */
const double no_alpha = NAN;
class ComplementaryFilter  {

	public:

		ComplementaryFilter();
		~ComplementaryFilter() {};

		void initialize(double dT, double fc);

        /** compute complementary filter
         *  x quantity
         *  dx derivative of the quantity
         *  alpha overwrites the cut off frequency and represents the weight of the previous value x */
	    VectorN compute(const VectorN& x, const VectorN& dx, double alpha = NAN);
	    VectorN compute(const VectorN& x, const VectorN& dx, const VectorN& alpha);

	    /** patch the low pass value */
	    void patchLowPassed(int idx, double x);


	private:
	    double dT;				 // time step of the filter [s]
	    double cut_off_freq; 	// cut off frequency of the filter [Hz]

	    double alpha;			// alpha used for filtering, calculated out of cut-off frequency
		VectorN x; 				// x of last call
		VectorN dx;				// dx of last call
		VectorN highpassed_x;	// high pass filtered x
		VectorN lowpassed_x;	// low pass filtered x
};

class KFilterBis {
public:
	KFilterBis();
	~KFilterBis() {};

	class KFilterBis:

	    def __init__(self, dt):
	        self.dt = dt
	        self.n = 3 + 3 + 4 * 3  # State = pos base + vel lin base + feet pos
	        self.m = 4 * 3 + 4  # Measure = relative pos of IMU

	        # State transition matrix
	        self.A = np.eye(self.n)
	        self.A[0:3, 3:6] = dt * np.eye(3)

	        # Control matrix
	        self.B = np.zeros((self.n, 3))
	        for i in range(3):
	            self.B[i, i] = 0.5 * dt**2
	            self.B[i+3, i] = dt

	        # Observation matrix
	        self.H = np.zeros((self.m, self.n))
	        for i in range(4):
	            for j in range(3):
	                self.H[3*i+j, j] = 1.0
	                self.H[3*i+j, j+6+3*i] = -1.0
	            self.H[12+i, 6+3*i+2] = 1.0
	        # Z: m x 1 Measurement vector

	        # Covariance of the process noise
	        self.Q = np.zeros((self.n, self.n))

	        # Covariance of the observation noise
	        self.R = np.zeros((self.m, self.m))

	        # a posteriori estimate covariance
	        self.P = np.eye(self.n)

	        # Optimal Kalman gain
	        self.K = np.zeros((self.n, self.m))

	        # Updated (a posteriori) state estimate
	        self.X = np.zeros((self.n, 1))

	        # Initial state and covariance
	        self.X = np.zeros((self.n, 1))

	        # Parameters to tune
	        self.sigma_kin = 0.1
	        self.sigma_h = 1.0
	        self.sigma_a = 0.1
	        self.sigma_dp = 0.1
	        self.gamma = 30

	    def setFixed(self, A, H, Q, R):
	        self.A = A
	        self.H = H
	        self.Q = Q
	        self.R = R

	    def setInitial(self, X0, P0):
	        # X : initial state of the system
	        # P : initial covariance

	        self.X = X0
	        self.P = P0

	    def predict(self, U):
	        # Make prediction based on physical system
	        # U : control vector (measured acceleration)

	        self.X = (self.A @ self.X) + self.B @ U
	        self.P = (self.A @ self.P @ self.A.T) + self.Q

	    def correct(self, Z):
	        # Correct the prediction, using measurement
	        # Z : measurement vector

	        self.K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
	        self.X = self.X + self.K @ (Z - self.H @ self.X)
	        self.P = self.P - self.K @ self.H @ self.P

	    def updateCoeffs(self, status):
	        # Update noise/covariance matrices depending on feet status

	        for i in range(4):
	            # Trust is between 1 and 0 (cliped to a very low value to avoid division by 0)
	            if status[i] == 0:
	                trust = 0.01
	            else:
	                trust = 1.0
	            self.R[(3*i):(3*(i+1)), (3*i):(3*(i+1))] = self.sigma_kin**2 / trust * np.eye(3)
	            self.R[12+i, 12+i] = self.sigma_h**2 / trust

	            self.Q[(6+3*i):(6+3*(i+1)), (6+3*i):(6+3*(i+1))] = self.sigma_dp**2 * (1+np.exp(self.gamma*(0.5-trust))) * np.eye(3) * self.dt**2

	        self.Q[3:6, 3:6] = self.sigma_a**2 * np.eye(3) * self.dt**2


	class ComplementaryFilter:
	    """Simple complementary filter

	    Args:
	        dt (float): time step of the filter [s]
	        fc (float): cut frequency of the filter [Hz]
	    """

	    def __init__(self, dt, fc):

	        self.dt = dt

	        y = 1 - np.cos(2*np.pi*fc*dt)
	        self.alpha = -y+np.sqrt(y*y+2*y)

	        self.x = np.zeros(3)
	        self.dx = np.zeros(3)
	        self.HP_x = np.zeros(3)
	        self.LP_x = np.zeros(3)
	        self.filt_x = np.zeros(3)

	    def compute(self, x, dx, alpha=None):
	        """Run one step of complementary filter

	        Args:
	            x (N by 1 array): quantity handled by the filter
	            dx (N by 1 array): derivative of the quantity
	            alpha (float): optional, overwrites the fc of the filter
	        """

	        # Update alpha value if the user desires it
	        if alpha is not None:
	            self.alpha = alpha

	        # For logging
	        self.x = x
	        self.dx = dx

	        # Process high pass filter
	        self.HP_x[:] = self.alpha * (self.HP_x + dx * self.dt)

	        # Process low pass filter
	        self.LP_x[:] = self.alpha * self.LP_x + (1.0 - self.alpha) * x

	        # Add both
	        self.filt_x[:] = self.HP_x + self.LP_x

	        return self.filt_x


};
#endif
