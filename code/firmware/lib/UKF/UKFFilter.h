#include "konfig.h"
#include "matrix.h"
#include "ukf.h"



#if (SISTEM_IMPLEMENTASI == SISTEM_PC)
    #include <iostream>
    #include <iomanip>      // std::setprecision
    using namespace std;
#endif


elapsedMillis timerLed, timerUKF;
uint64_t u64lamaUKF;

UKF UKF_IMU(1000., 1e-6, 0.0015);       /* Best */
// UKF UKF_IMU(1000., 1e-7, 0.0015);       /* meh */
Matrix Z(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
Matrix dataQuaternion(SS_X_LEN, 1);
/* The command from the PC */

/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Rv_INIT     (1e-6)
#define Rn_INIT_ACC (0.0015)

void setupKalman() {

}

void computeKalman(double accX, double accY, double accZ, 
                   double gyroX, double gyroY, double gyroZ,
                   double &x, double &y, double &z, double &w) {
        Z[0][0] = accX;
        Z[1][0] = accY;
        Z[2][0] = accZ;
        U[0][0] = gyroX;
        U[1][0] = gyroY;
        U[2][0] = gyroZ;
        /* Normalisasi */
        double _normG = (Z[0][0] * Z[0][0]) + (Z[1][0] * Z[1][0]) + (Z[2][0] * Z[2][0]);
        _normG = sqrt(_normG);
        Z[0][0] = Z[0][0] / _normG;
        Z[1][0] = Z[1][0] / _normG;
        Z[2][0] = Z[2][0] / _normG;

        /* Update Kalman */
        UKF_IMU.vUpdate(Z, U);
        dataQuaternion = UKF_IMU.BacaDataX();
        w = dataQuaternion[0][0];
        x = dataQuaternion[1][0];
        y = dataQuaternion[2][0];
        z = dataQuaternion[3][0];
        
}


void resetKalman() {
    dataQuaternion.vSetToZero();
    dataQuaternion[0][0] = 1.0;
    UKF_IMU.vReset( P_INIT, Rv_INIT, Rn_INIT_ACC);
}

void serialFloatPrint(float f) {
    byte * b = (byte *) &f;
    for (int i = 0; i < 4; i++) {
        byte b1 = (b[i] >> 4) & 0x0f;
        byte b2 = (b[i] & 0x0f);

        char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
        char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

        Serial.print(c1);
        Serial.print(c2);
    }
}

