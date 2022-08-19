#ifndef AVES_INITS_H
#define AVES_INITS_H

#include "Arduino.h"

#define INIT_GPS 0
#define INIT_ALL_LOGS 1

//LOG INITS
#define ANGELS_LOG_FLAG 1
#define ALTITUDE_LOG_FLAG 1
// #define HAND_CONTRL_LOG_FLAoG 1
// #define GPS_LOG_FLAG 1
// #define ALTITUDE_LOG_FLAG 1
// #define VY_LOG_FLAG 1
// #define GYRO_ACCEL_LOG_FLAG 0
// #define ANGLES_LOG_FLAG 1
// #define +SEP 1
// #define DELTA_PHI 1
// #define CTRL_EFFECT_LOG_FLAG 1
#define SEP String(",")
#define END_SEP String("\n")

// LOGS HEADER
#define logs_def(ANY_LOG_INIT_FLAG, NAME) (ANY_LOG_INIT_FLAG)? NAME : ""
String HEADER = "TIME" 
                      + logs_def(ANGELS_LOG_FLAG, ",ROLL,PITCH,YAW")
                      + logs_def(ALTITUDE_LOG_FLAG, ",ALT")
                      + logs_def(INIT_GPS, ",LAT,LNG");

#endif //AVES_INITS _H
