/**********************************************************************************************************************
 *  Fungsi Discrete Unscented-kalman-filter (dihitung secara diskrit), Ref:
 *          Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic
 *          Inference in Dynamic State-Space Models (Ph.D. thesis). Oregon Health &
 *          Science University.
 *
 *         Formulasi plant yang diestimasi:
 *              x*(k) = f[x(k-1), u(k)] + w(k)          ; x=Nx1,    u=Mx1               ...{UKF_1}
 *              z(k)  = h[x(k), u(k)] + v(k)            ; z=Zx1                         ...{UKF_2}
 *
 *        dimana:
 *          x*(k)   : Update x(k-1) terhadap waktu cuplik       : Nx1
 *          x(k-1)  : State Variable sistem waktu ke-k          : Nx1
 *          z(k)    : Keluaran sistem waktu ke-k                : Zx1
 *          u(k)    : Masukan sistem waktu ke-k                 : Mx1
 *          w(k)    : Noise proses, asumsi AWGN, kovarian Q     : Nx1
 *          v(k)    : Noise pengukuran, asumsi AWGN, kovarian R : Nx1
 *
 *          f(..), h(..) adalah model (non-linear) dari sistem yang ingin diprediksi
 *
 **********************************************************************************************************************
 *         Algoritma Discrete Unscented-kalman-filter:
 *          Init:
 *                  P (k=0|k=0) = Identitas * Kovarian(P(k=0)), biasanya diisi dg nilai yg besar
 *                  X~(k=0|k=0) = Ekspektasi(X(k=0)), biasanya = 0
 *                  Wc, Wm      = First order & second order weight
 *          Lakukan iterasi:
 *              Kalkulasi sigma point:
 *                  XSigma(k|k-1) = [X~ Y+sqrt(C*P) Y-sqrt(C*P)]    ; Y = [X~ ... X~]   ...{UKF_3}
 *                                                                  ; Y=NxN
 *                                                                  ; C = (N + Lambda)
 * 
 *              Unscented Transform XSigma [f,XSigma,U,Wm,Wc,Q] -> [X~,XSigma,P,dSigX]:
 *                  XSigma(k|k-1) = f(XSigma(k-1|k-1), u(k))                            ...{UKF_4}
 *                  X~(k|k-1) = sum(Wm(i) * XSigma(k|k-1)(i))       ; i = 1 ... (2N+1)  ...{UKF_5}
 * 
 *                  dSigX = XSigma(k|k-1)(i) - Y(k)  ; Y(k) = [X~(k|k-1) .. X~(k|k-1)]
 *                                                   ; Y(k) = Nx(2N+1)                  ...{UKF_6}
 *                  P(k|k-1) = sum(Wc(i)*dSigX(i)*dSigX(i)') + Q    ; i = 1 ... (2N+1)  ...{UKF_7}
 *
 * 
 *              Unscented Transform ZSigma [h,XSigma,U,Wm,Wc,R] -> [Z~,ZSigma,Pz,dSigZ]:
 *                  ZSigma(k|k-1) = h(XSigma(k|k-1), u(k))                              ...{UKF_4}
 *                  Z~(k|k-1) = sum(Wm(i) * ZSigma(k|k-1)(i))       ; i = 1 ... (2N+1)  ...{UKF_5}
 * 
 *                  dSigZ = ZSigma(k|k-1)(i) - Y(k)  ; Y(k) = [Z~(k|k-1) .. Z~(k|k-1)]
 *                                                   ; Y(k) = Zx(2N+1)                  ...{UKF_6}
 *                  Pz(k|k-1) = sum(Wc(i)*dSigZ(i)*dSigZ(i)') + R   ; i = 1 ... (2N+1)  ...{UKF_7}
 * 
 * 
 *              Update the estimated system:
 *                  CrossCov(k) = sum(Wc(i)*dSigX(i)*dSigZ(i)')     ; i = 1 ... (2N+1)  ...{UKF_8}
 *                  K           = CrossCov(k) * Pz^-1                                   ...{UKF_9}
 *                  X~(k|k)     = X~(k|k-1) + K * (Z(k) - Z~(k|k-1))                    ...{UKF_10}
 *                  P(k|k)      = P(k|k-1) - K*Pz*K'                                    ...{UKF_11}
 *
 *        *Catatan tambahan:
 *              - Perhatikan persamaan f pada {UKF_4} adalah update versi diskrit!!!!
 *                              X~(k) = f(X~(k-1),u(k))
 *              - Dengan asumsi masukan plant ZOH, u(k) = u(k|k-1),
 *                  Dengan asumsi tambahan observer dijalankan sebelum pengendali, u(k|k-1) = u(k-1),
 *                  sehingga u(k) [untuk perhitungan kalman] adalah nilai u(k-1) [dari pengendali].
 *              - Notasi yang benar adalah u(k|k-1), tapi disini menggunakan notasi u(k) untuk
 *                  menyederhanakan penulisan rumus.
 *              - Pada contoh di atas X~(k=0|k=0) = [0]. Untuk mempercepat konvergensi bisa digunakan
 *                  informasi plant-spesific. Misal pada implementasi EKF untuk memfilter sensor
 *                  IMU (Inertial measurement unit) dengan X = [quaternion], dengan asumsi IMU
 *                  awalnya menghadap ke atas tanpa rotasi, X~(k=0|k=0) = [1, 0, 0, 0]'
 *
 *        Variabel:
 *          X_kira(k)    : X~(k) = X_Estimasi(k) kalman filter   : Nx1
 *          P(k)         : P(k) = matrix kovarian kalman filter  : NxN
 *          Q            : Matrix kovarian dari w(k)             : NxN
 *          R            : Matrix kovarian dari v(k)             : ZxZ
 *
 **********************************************************************************************************************/

#include "ukf.h"

#define P_INIT       (1.)
#define Rv_INIT      (1e-7)

// from 3DM-CV5-x datasheet: noise of acceleromter is 100ug/sqrt(frequency) = 
#define Rn_INIT_ACC  (0.00003)
// from LIS3DM datasheet: noise in ultra performance mode is 3.5 milli gauss = 3.5 = 0.35 uT
#define Rn_INIT_MAG  (0.35)
#define IMU_ACC_Z0   (1)

// setup the filter with the targetfrequency. The IMU is configured to deliver a datapoint in that
// frequency, it is not measured in here.
void UnscentedKalmanFilter::setup(double targetFreq) {

    // later on, we need the sample time in [s]
    dT = 1.0/targetFreq;
    
    /* Initialization:
     *  P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
     *  x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
     *  Rv, Rn      = Covariance matrices of process & measurement. As this implementation 
     *                the noise as AWGN (and same value for every variable), this is set
     *                to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
     */
    Pinit = P_INIT;
    Qinit = Rv_INIT;
    Rinit = Rn_INIT_ACC;

    /* Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models 
     * (Ph.D. thesis). Oregon Health & Science University. Page 6:
     * 
     * where λ = α2(L+κ)−L is a scaling parameter. α determines the spread of the sigma points around ̄x and is usually 
     * set to a small positive value (e.g. 1e−2 ≤ α ≤ 1). κ is a secondary scaling parameter which is usually set to either 
     * 0 or 3−L (see [45] for details), and β is an extra degree of freedom scalar parameter used to incorporate any extra 
     * prior knowledge of the distribution of x (for Gaussian distributions, β = 2 is optimal).
     */
    double _alpha   = 1e-2;
    double _k       = 0.0;
    double _beta    = 2.0;

      /* lambda = (alpha^2)*(N+kappa)-N,         
         gamma = sqrt(N+alpha)            ...{UKF_1} */
    double _lambda  = (_alpha*_alpha)*(SS_X_LEN+_k) - SS_X_LEN;
    Gamma = sqrt((SS_X_LEN + _lambda));

    P.setZero();
    Rv.setZero();
    Rn.setZero();
    X_Est.setZero();
    X_Est[0][0] = 1.0;       /* Quaternion(k = 0) = [1 0 0 0]' */

    P.setDiag(Pinit);
    Rv.setDiag(Qinit);
    Rn.setDiag(Rinit);
 #ifdef USE_MAG   
    Rn[3][3] = Rn_INIT_MAG;
    Rn[4][4] = Rn_INIT_MAG;
    Rn[5][5] = Rn_INIT_MAG;
#endif
    /* Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_2} */
    Wm[0][0] = _lambda/(SS_X_LEN + _lambda);
    for (int32_t _i = 1; _i < Wm.getCols(); _i++) {
        Wm[0][_i] = 0.5/(SS_X_LEN + _lambda);
    }

   /* Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_3} */
    Wc = Wm;
    Wc[0][0] = Wc[0][0] + (1.0-(_alpha*_alpha)+_beta);

}

void UnscentedKalmanFilter::setNorthVector( Matrix& northCalibration)
{
    north_vector = northCalibration;
    reset();
}

void UnscentedKalmanFilter::reset()
{
    P.setDiag(Pinit);
    Rv.setDiag(Qinit);
    Rn.setDiag(Rinit);
    X_Est.setZero();
    X_Est[0][0] = 1.0;          // Quaternion = [1 0 0 0]
}

void UnscentedKalmanFilter::compute(double accX, double accY, double accZ, 
                                    double gyroX, double gyroY, double gyroZ,
#ifdef WITH_MAG       
                                    double magX, double magY, double magZ,
#endif                                
                                    double &x, double &y, double &z, double &w) {
    Matrix Y{SS_Z_LEN, 1};          

    // set accel data in [g]
    Y[0][0] = accX;
    Y[1][0] = accY;
    Y[2][0] = accZ;

    // normalise
    double _normG = sqrt((Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]));
    Y[0][0] = Y[0][0] / _normG;
    Y[1][0] = Y[1][0] / _normG;
    Y[2][0] = Y[2][0] / _normG;

#ifdef WITH_MAG
    /* Output 4:6 = magnetometer */
    Y[3][0] = magX; 
    Y[4][0] = magY; 
    Y[5][0] = magZ;

    /* Compensating Hard-Iron Bias for magnetometer */
    Y[3][0] = Y[3][0];
    Y[4][0] = Y[4][0];
    Y[5][0] = Y[5][0];

    double _normM = sqrt(Y[3][0] * Y[3][0]) + (Y[4][0] * Y[4][0]) + (Y[5][0] * Y[5][0]);
    Y[3][0] = Y[3][0] / _normM;
    Y[4][0] = Y[4][0] / _normM;
    Y[5][0] = Y[5][0] / _normM;
#endif


    // set gyro data in [rad/s]
    Matrix U{SS_U_LEN, 1};  
    U[0][0] = gyroX;
    U[1][0] = gyroY;
    U[2][0] = gyroZ;


    // Update Kalman 
    updateFilter(Y, U);

    // get result
    Matrix result  = getX();
    w = result[0][0];
    x = result[1][0];
    y = result[2][0];
    z = result[3][0];
}


void UnscentedKalmanFilter::updateFilter(Matrix &Y, Matrix &U)
{
   /* Run once every sampling time */

    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}  */
    calculateSigmaPoint();
    
    /* Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:       ...{UKF_5a} - {UKF_8a} */
    unscentedTransform(X_Est, X_Sigma, P, DX, (&UnscentedKalmanFilter::updateNonlinearX), X_Sigma, U, Rv);     
    
    /* Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:  ...{UKF_5b} - {UKF_8b} */
    unscentedTransform(Y_Est, Y_Sigma, Py, DY, (&UnscentedKalmanFilter::updateNonlinearY), X_Sigma, U, Rn);     


    /* Calculate Cross-Covariance Matrix:
     *  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
     */
    for (int32_t _i = 0; _i < DX.getRows(); _i++) {
        for (int32_t _j = 0; _j < DX.getCols(); _j++) {
            DX[_i][_j] *= Wc[0][_j];
        }
    }
    Pxy = DX * (DY.Transpose());

    /* Calculate the Kalman Gain:
     *  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
     */
    Matrix PyInv = Py.inverse();

    Gain = Pxy * PyInv;
    /* Update the Estimated State Variable:
     *  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
     */
    X_Est = X_Est + (Gain*(Y - Y_Est));

    /* Update the Covariance Matrix:
     *  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
     */
    P = P - (Gain * Py * Gain.Transpose());

     if (!X_Est.isUnitVector()) {
        Serial.println("System error");
        /* System error, reset EKF */ 
        reset();

    }
}

void UnscentedKalmanFilter::calculateSigmaPoint()
{
    /* Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
     * GPsq = gamma * sqrt(P(k-1))
     * XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}
     */
    /* Use Cholesky Decomposition to compute sqrt(P) */
    P_Chol = P.choleskyDec();
    P_Chol = P_Chol * Gamma;     

    Matrix _Y(SS_X_LEN, SS_X_LEN);
    for (int32_t _i = 0; _i < SS_X_LEN; _i++) {
        _Y = _Y.insertVector(X_Est, _i);
    }
    X_Sigma.setZero();
    /* Set _xSigma = [x 0 0] */
    X_Sigma = X_Sigma.insertVector(X_Est, 0);
    /* Set _xSigma = [x Y+C*sqrt(P) 0] */
    X_Sigma = X_Sigma.InsertSubMatrix((_Y + P_Chol), 0, 1);
    /* Set _xSigma = [x Y+C*sqrt(P) Y-C*sqrt(P)] */
    X_Sigma = X_Sigma.InsertSubMatrix((_Y - P_Chol), 0, (1+SS_X_LEN));

}

void UnscentedKalmanFilter::unscentedTransform(Matrix &Out, Matrix &OutSigma, Matrix &P, Matrix &DSig,
                                             UpdateNonLinear funcNonLinear,
                                             Matrix &inpSigma, Matrix &inpVector,
                                             Matrix &covNoise)
{
    /* XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}  */
    /* x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}  */
    Out.setZero();
    for (int32_t _j = 0; _j < inpSigma.getCols(); _j++) {
        /* Transformasi non-linear per kolom */
        Matrix _AuxSigma1(inpSigma.getRows(), 1);
        Matrix _AuxSigma2(OutSigma.getRows(), 1);
        for (int32_t _i = 0; _i < inpSigma.getRows(); _i++) {
            _AuxSigma1[_i][0] = inpSigma[_i][_j];
        }
        (this->*(funcNonLinear))(_AuxSigma2, _AuxSigma1, inpVector);      /* Welp... */

        /* Combine the transformed vector to construct sigma-points output matrix (OutSigma) */
        OutSigma = OutSigma.insertVector(_AuxSigma2, _j);

        /* Calculate x(k|k-1) as weighted mean of OutSigma */
        _AuxSigma2 = _AuxSigma2 * Wm[0][_j];
        Out = Out + _AuxSigma2;
    }


    /* DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
     *                             ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}  */
    Matrix _AuxSigma1(OutSigma.getRows(), OutSigma.getCols());
    for (int32_t _j = 0; _j < OutSigma.getCols(); _j++) {
        _AuxSigma1 = _AuxSigma1.insertVector(Out, _j);
    }
    DSig = OutSigma - _AuxSigma1;

    /* P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}  */
    _AuxSigma1 = DSig;
    for (int32_t _i = 0; _i < DSig.getRows(); _i++) {
        for (int32_t _j = 0; _j < DSig.getCols(); _j++) {
            _AuxSigma1[_i][_j] *= Wc[0][_j];
        }
    }

    P = (_AuxSigma1 * (DSig.Transpose())) + covNoise;
}

void UnscentedKalmanFilter::updateNonlinearX(Matrix &X_Next, Matrix &X, Matrix &U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    double q0 = X[0][0];
    double q1 = X[1][0];
    double q2 = X[2][0];
    double q3 = X[3][0];

    double p = U[0][0];
    double q = U[1][0];
    double r = U[2][0];

    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*dT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*dT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*dT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*dT + q3;
}

void UnscentedKalmanFilter::updateNonlinearY(Matrix &Y, Matrix &X, Matrix &U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                        2*(q1*q2+q0*q3),                        2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3),     (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                        2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                        2*(q2*q3-q0*q1),     (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    double q0 = X[0][0];
    double q1 = X[1][0];
    double q2 = X[2][0];
    double q3 = X[3][0];

    double q0_2 = q0 * q0;
    double q1_2 = q1 * q1;
    double q2_2 = q2 * q2;
    double q3_2 = q3 * q3;

    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;
    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;
    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;

#ifdef WITH_MAG
    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2))    * north_vector[0][0]
              +2*(q1*q2+q0*q3)                  * north_vector[1][0]
              +2*(q1*q3-q0*q2)                  * north_vector[2][0];

    Y[4][0] = 2*(q1*q2-q0*q3)                   * north_vector[0][0]
              +(+(q0_2)-(q1_2)+(q2_2)-(q3_2))   * north_vector[1][0]
              +2*(q2*q3+q0*q1)                  * north_vector[2][0];

    Y[5][0] =  2*(q1*q3+q0*q2)                  * north_vector[0][0]
              +2*(q2*q3-q0*q1)                  * north_vector[1][0]
              +(+(q0_2)-(q1_2)-(q2_2)+(q3_2))   * north_vector[2][0];
#endif
}
