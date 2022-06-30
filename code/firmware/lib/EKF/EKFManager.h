#include "ekf.h"


aaa
void SPEW_THE_ERROR(char const * str)
{
    Serial.println(str);
    while(1);
}


/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)

/* ============================================ EKF variables/function declaration ============================================ */
/* Just example; in konfig.h: 
 *  SS_X_LEN = 4
 *  SS_Z_LEN = 3
 *  SS_U_LEN = 3
 */
/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.0015/10.)
#define R_INIT_MAG  (0.0015/10.)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,      0,      0,
                                                0,      Q_INIT, 0,      0,
                                                0,      0,      Q_INIT, 0,
                                                0,      0,      0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT_ACC, 0,          0,         
                                                0,          R_INIT_ACC, 0,         
                                                0,          0,          R_INIT_ACC};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* EKF system declaration ------------------------------------------------------------------------------------------- */
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
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
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;
    
    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    p = U[0][0];
    q = U[1][0];
    r = U[2][0];
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false so we can reset the UKF */
        return false;
    }
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;
    
    return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    /* In Main_bUpdateNonlinearX():
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec p, q, r;

    p = U[0][0];
    q = U[1][0];
    r = U[2][0];

    F[0][0] =  1.000;
    F[1][0] =  0.5*p * SS_DT;
    F[2][0] =  0.5*q * SS_DT;
    F[3][0] =  0.5*r * SS_DT;

    F[0][1] = -0.5*p * SS_DT;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * SS_DT;
    F[3][1] =  0.5*q * SS_DT;

    F[0][2] = -0.5*q * SS_DT;
    F[1][2] =  0.5*r * SS_DT;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * SS_DT;

    F[0][3] = -0.5*r * SS_DT;
    F[1][3] = -0.5*q * SS_DT;
    F[2][3] =  0.5*p * SS_DT;
    F[3][3] =  1.000;
    
    return true;
}



bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /* In Main_bUpdateNonlinearY():
     * 
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 -g]            --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    H[0][0] = -2*q2 * IMU_ACC_Z0;
    H[1][0] = +2*q1 * IMU_ACC_Z0;
    H[2][0] = +2*q0 * IMU_ACC_Z0;
    
    H[0][1] = +2*q3 * IMU_ACC_Z0;
    H[1][1] = +2*q0 * IMU_ACC_Z0;
    H[2][1] = -2*q1 * IMU_ACC_Z0;
    
    H[0][2] = -2*q0 * IMU_ACC_Z0;
    H[1][2] = +2*q3 * IMU_ACC_Z0;
    H[2][2] = -2*q2 * IMU_ACC_Z0;
    
    H[0][3] = +2*q1 * IMU_ACC_Z0;
    H[1][3] = +2*q2 * IMU_ACC_Z0;
    H[2][3] = +2*q3 * IMU_ACC_Z0;
    
    return true;
}

void setupKalman() {
  /* EKF initialization ----------------------------------------- */
  /* x(k=0) = [1 0 0 0]' */
  quaternionData.vSetToZero();
  quaternionData[0][0] = 1.0;
  EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
}

void computeKalman(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ,
                double &x,double &y,double &z,double &w) {

        float Ax = accelX;
        float Ay = accelY;
        float Az = accelZ;
        float p =  gyroX;
        float q =  gyroY;
        float r =  gyroZ;

        /* Input 1:3 = gyroscope */
        U[0][0] = p;  U[1][0] = q;  U[2][0] = r;
        /* Output 1:3 = accelerometer */
        Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
 
        /* Normalizing the output vector */
        float_prec _normG = sqrt(Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]);
        Y[0][0] = Y[0][0] / _normG;
        Y[1][0] = Y[1][0] / _normG;
        Y[2][0] = Y[2][0] / _normG;        
        /* ============================= Update the Kalman Filter ============================== */
        if (!EKF_IMU.bUpdate(Y, U)) {
            quaternionData.vSetToZero();
            quaternionData[0][0] = 1.0;
            EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        } 
        quaternionData = EKF_IMU.GetX();
        w = quaternionData[0][0];
        x = quaternionData[1][0];
        y = quaternionData[2][0];
        z = quaternionData[3][0];
}


void resetKalman() {
    quaternionData.vSetToZero();
    quaternionData[0][0] = 1.0;
    EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
}