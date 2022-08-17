#include "pid.h"
// 
PID::PID(double p, double i, double d, double (*time_func)()){
    set_pid(p,i,d);
    sys_time = time_func;
}

void PID::set_pid(double pp, double ii, double dd) {
    p = pp;
    i = ii;
    d = dd;
}

void PID::set_pid_borders(double max_val, double min_val){
    maxOut = max_val;
    minOut = min_val;
    borders_flag = true;
}

double PID::ctrl(double req, double cur) {
    double res;
    double sys_tm = sys_time();
    dt = sys_tm - t;
    t = sys_tm;
    e = req - cur;
    sum += dt*e;
    res = p*e + d*(e - prev)/dt + sum*i;
    prev  = e;
    if(!borders_flag)
        return res;
    return (res > maxOut)? maxOut : (res < minOut)? minOut : res;
}
void PID::reset(){
    e = 0;
    prev = 0;
    sum = 0;
}
