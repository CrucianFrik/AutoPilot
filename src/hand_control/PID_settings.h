#ifndef PID_SETTINGS_H
#define PID_SETTINGS_H

#include "pid.h"

// Vertical speed
#define CY_P 6.0
#define CY_I 0.0
#define CY_D 0.0
// Height
#define HEIGHT_P 0.2
#define HEIGHT_I 0.0
#define HEIGHT_D 0.00000005
// Roll
#define ROLL_P 0.02
#define ROLL_I 0.0
#define ROLL_D 0.0000025
// Pitch
#define PITCH_P 0.012
#define PITCH_I 0.0
#define PITCH_D 0.0//7//00002

// Angle
#define AGL_P 0.5
#define AGL_I 0.0
#define AGL_D 0.02
//NOT_DEFINED !!!
#define GY_P 0.0
#define GY_I 0.0
#define GY_D 0.0


double sys_time(){
  return micros();
}

PID height_pid(HEIGHT_P, HEIGHT_I, HEIGHT_D, sys_time);
PID vy_pid(CY_P, CY_I, CY_D, sys_time);
PID pitch_pid(PITCH_P, PITCH_I, PITCH_D, sys_time);
PID way_angle_pid(AGL_P, AGL_I, AGL_D, sys_time); 
PID roll_pid(ROLL_P, ROLL_I, ROLL_D, sys_time);


void set_pids_borders(){
  vy_pid.set_pid_borders(30.0, -30.0);
  height_pid.set_pid_borders(5.0, -5.0);
}

//PID OVERLOAD_Y(GY_P, GY_I, GY_D, sys_time);  // overload in the direction of the wing
#endif
