/**********************************************************************************************************************
 *  Class for Discrete Unscented Kalman Filter 
 *  Ref: Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic 
 *      State-Space Models (Ph.D. thesis). Oregon Health & Science University.
 *
 *  The system to be estimated is defined as a discrete nonlinear dynamic dystem:
 *              x(k+1) = f[x(k), u(k)] + v(k)           ; x = Nx1,    u = Mx1
 *              y(k)   = h[x(k), u(k)] + n(k)           ; y = Zx1
 *
 *        Where:
 *          x(k) : State Variable at time-k                          : Nx1
 *          y(k) : Measured output at time-k                         : Zx1
 *          u(k) : System input at time-k                            : Mx1
 *          v(k) : Process noise, AWGN assumed, w/ covariance  Rv    : Nx1
 *          n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
 *
 *          f(..), h(..) is a nonlinear transformation of the system to be estimated.
 *
 **********************************************************************************************************************
 *      Unscented Kalman Filter algorithm:
 *          Initialization:
 *              P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
 *              x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *              Rv, Rn      = Covariance matrices of process & measurement. As this implementation 
 *                              the noise as AWGN (and same value for every variable), this is set
 *                              to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
 *              Wc, Wm      = First order & second order weight, respectively.
 *              alpha, beta, kappa, gamma = scalar constants.
 *              
 *              lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)           ...{UKF_1}
 *              Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_2}
 *              Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_3}
 *              
 *          
 *          UKF Calculation (every sampling time):
 *              Calculate the Sigma Point:
 *                  Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
 *                  GPsq = gamma * sqrt(P(k-1))
 *                  XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                    ...{UKF_4}
 *              
 *              
 *              Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:
 *                  XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}
 * 
 *                  x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}
 * 
 *                  DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
 *                                              ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}
 * 
 *                  P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}
 *
 * 
 *              Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:
 *                  YSigma(k) = h(XSigma(k), u(k|k-1))      ; u(k|k-1) = u(k)           ...{UKF_5b}
 * 
 *                  y_est(k) = sum(Wm(i) * YSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6b}
 * 
 *                  DY = YSigma(k)(i) - Ys(k)   ; Ys(k) = [y_est(k) ... y_est(k)]
 *                                              ; Ys(k) = Zx(2N+1)                      ...{UKF_7b}
 * 
 *                  Py(k) = sum(Wc(i)*DY*DY') + Rn          ; i = 1 ... (2N+1)          ...{UKF_8b}
 * 
 * 
 *              Calculate Cross-Covariance Matrix:
 *                  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
 *              
 *              
 *              Calculate the Kalman Gain:
 *                  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
 *              
 *              
 *              Update the Estimated State Variable:
 *                  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
 *              
 *              
 *              Update the Covariance Matrix:
 *                  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
 *        Variabel:
 *          X_kira(k)    : X~(k) = X_Estimasi(k) kalman filter   : Nx1
 *          P(k)         : P(k) = matrix kovarian kalman filter  : NxN
 *          Q            : Matrix kovarian dari w(k)             : NxN
 *          R            : Matrix kovarian dari v(k)             : ZxZ
 *
 **********************************************************************************************************************/

#ifndef UKF_H
#define UKF_H

#include "matrix.h"


class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter() {};

    // initialize filter and tell it the frequency, compute is going to be called  
    void setup(double targetFreq);

    // reset all internal date structures and start from scratch
    void reset();

    // do the filtering. Input is Gyro [rad/s] and Accel [g], output is a quat 
    void compute(double accX, double accY, double accZ, 
                 double gyroX, double gyroY, double gyroZ,
                 double &x, double &y, double &z, double &w);

private:
    // intialise data structures of unscented Kalman filter
    void init(double sampleTime, const double PInit, const double QInit, const double RInit);

    // reset internal data structures
    void resetFilter();

    // carry out filtering
    void updateFilter(Matrix &Z, Matrix &U);

    // return the estimated result of the filter
    Matrix getX() { return X_Est; }

    // return covariance 
    Matrix getP() { return P; }

    // return difference between estimate and prediction
    Matrix getErr() { return Err; }

    // updateNonLinearX/Z is called from unscented Transform twice, a function pointer makes this convinient  
    typedef  void (UnscentedKalmanFilter::*UpdateNonLinear)(Matrix &X_dot, Matrix &X, Matrix &U);
    
    void calculateSigmaPoint();

    void unscentedTransform(Matrix &Out, Matrix &OutSigma, Matrix &P, Matrix &DSig,
                             UpdateNonLinear _vFuncNonLinear,
                             Matrix &InpSigma, Matrix &InpVector,
                              Matrix &_CovNoise);
    void updateNonlinearX(Matrix &X_Next, Matrix &X, Matrix &U);
    void updateNonlinearZ(Matrix &Z_est, Matrix &X, Matrix &U);

    #define SS_X_LEN    (4) // State: Quaternion 
    #define SS_Z_LEN    (3) // Output: Accel
    #define SS_U_LEN    (3) // Input:  Gyro

    Matrix U{SS_U_LEN, 1};                              // input: Gyro
    Matrix Z{SS_Z_LEN, 1};                              // outpu: Accel
    
    Matrix dataQuaternion{SS_X_LEN, 1};                 // output quat
    
    Matrix X_Est{SS_X_LEN, 1};                          // state variable, estimated
    Matrix X_Sigma{SS_X_LEN, (2*SS_X_LEN + 1)};
    
    Matrix Y_Est{SS_Z_LEN, 1};
    Matrix Y_Sigma{SS_Z_LEN, (2*SS_X_LEN + 1)};
    
    Matrix P{SS_X_LEN, SS_X_LEN};                       // covariance matrix
    Matrix P_Chol{SS_X_LEN, SS_X_LEN};
    
    Matrix DX{SS_X_LEN, (2*SS_X_LEN + 1)};              // Cross-Covariance Matrix of X
    Matrix DY{SS_Z_LEN, (2*SS_X_LEN + 1)};              // Cross-Covariance Matrix of Y
    
    Matrix Py{SS_Z_LEN, SS_Z_LEN};
    Matrix Pxy{SS_X_LEN, SS_Z_LEN};
    
    Matrix Wm{1, (2*SS_X_LEN + 1)};
    Matrix Wc{1, (2*SS_X_LEN + 1)};
    
    Matrix Rv{SS_X_LEN, SS_X_LEN};                      // covariance Matrix of process
    Matrix Rn{SS_Z_LEN, SS_Z_LEN};                      // covariance Matrix of measurement

    Matrix Err{SS_Z_LEN, 1};                            // difference between X_Est and X
    Matrix Gain{SS_X_LEN, SS_Z_LEN};                    // Kalman Gain
    double Gamma;

    double Pinit = 1;
    double Qinit = 1e-7;
    double Rinit = 0.0015;

    double dT = 0;
};


#endif // UKF_H