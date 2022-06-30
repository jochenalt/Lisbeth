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

#ifndef UKF_H
#define UKF_H

#include "matrix.h"


class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter() {};

    void setup(double targetFreq);
    void reset();
    void compute(double accX, double accY, double accZ, 
                   double gyroX, double gyroY, double gyroZ,
                   double &x, double &y, double &z, double &w);

private:
    void init(double sampleTime, const double PInit, const double QInit, const double RInit);
    void vReset();
    void update(Matrix &Z, Matrix &U);
    Matrix getX() { return X_Est; }
    Matrix getP() { return P; }
    Matrix getErr() { return Err; }
    typedef  void (UnscentedKalmanFilter::*UpdateNonLinear)(Matrix &X_dot, Matrix &X, Matrix &U);
    void calculateSigmaPoint();
    void unscentedTransform(Matrix &Out, Matrix &OutSigma, Matrix &P, Matrix &DSig,
                             UpdateNonLinear _vFuncNonLinear,
                             Matrix &InpSigma, Matrix &InpVector,
                              Matrix &_CovNoise);
    void updateNonlinearX(Matrix &X_Next, Matrix &X, Matrix &U);
    void updateNonlinearZ(Matrix &Z_est, Matrix &X, Matrix &U);


    /* State Space dimension */
    #define SS_X_LEN    (4) // State: Quaternion 
    #define SS_Z_LEN    (3) // Output: Accel
    #define SS_U_LEN    (3) // Input:  Gyro

    Matrix X_Est{SS_X_LEN, 1};
    Matrix X_Sigma{SS_X_LEN, (2*SS_X_LEN + 1)};
    
    Matrix Y_Est{SS_Z_LEN, 1};
    Matrix Y_Sigma{SS_Z_LEN, (2*SS_X_LEN + 1)};
    
    Matrix P{SS_X_LEN, SS_X_LEN};
    Matrix P_Chol{SS_X_LEN, SS_X_LEN};
    
    Matrix DX{SS_X_LEN, (2*SS_X_LEN + 1)};
    Matrix Dy{SS_Z_LEN, (2*SS_X_LEN + 1)};
    
    Matrix Py{SS_Z_LEN, SS_Z_LEN};
    Matrix Pxy{SS_X_LEN, SS_Z_LEN};
    
    Matrix Wm{1, (2*SS_X_LEN + 1)};
    Matrix Wc{1, (2*SS_X_LEN + 1)};
    
    Matrix Rv{SS_X_LEN, SS_X_LEN};
    Matrix Rn{SS_Z_LEN, SS_Z_LEN};

    Matrix Err{SS_Z_LEN, 1};
    Matrix Gain{SS_X_LEN, SS_Z_LEN};
    double Gamma;

    double Pinit = 1;
    double Qinit = 1e-7;
    double Rinit = 0.0015;

    double dT = 0;
};


#endif // UKF_H