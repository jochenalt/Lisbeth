/*************************************************************************************************************
 * This file contains configuration parameters
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef KONFIG_H
#define KONFIG_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>



/* State Space dimension */
#define SS_X_LEN    (4) // State: Quaternion 
#define SS_Z_LEN    (3) // Output: Accel
#define SS_U_LEN    (3) // Input:  Gyro
#define SS_DT_MILIS (1.0)                            /* 20 ms */
#define SS_DT       (SS_DT_MILIS/1000.)   /* Sampling time */



/* Set this define to choose math precision of the system */


/* Set this define to choose system implementation (mainly used to define how you print the matrix via the Matrix::vCetak() function) */
#define SYSTEM_IMPLEMENTATION                       (SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)



#endif // KONFIG_H
