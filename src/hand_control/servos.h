#ifndef SERVA_H // For Petr
#define SERVA_H

#include <ESP32Servo.h>
#include "FlySkyIBus.h"
#include "PID_settings.h"

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

const int antenaPin     = 39;
const int engine_Pin    = 13;
const int eileron_l_pin = 19;
const int eileron_r_pin = 18;
const int pgo_l_pin     = 23;
const int pgo_r_pin     =  5;


HardwareSerial FSkySerial(1);
int control_mode_flag = 1; // Hand mode = 1

double pitch_ctrl_effect, roll_ctrl_effect;
double pitch_ctrl_effect_2, roll_ctrl_effect_2;
double eilerons_ctrl, pgo_l_ctrl, pgo_r_ctrl;

Servo eileron_l;  
Servo eileron_r; 
Servo pgo_l;
Servo pgo_r;
Servo engine;

double new_pitch_p = 0.005, new_roll_p = 0.02;

double servo_control[12] = {1500,1500,1000,1500,1500,1500,1000,1500,1500,1500,1500,1500};

void init_control() {
  IBus.begin(FSkySerial);
  engine.attach(engine_Pin); 
  eileron_l.attach(eileron_l_pin);
  eileron_r.attach(eileron_r_pin); 
  pgo_l.attach(pgo_l_pin);
  pgo_r.attach(pgo_r_pin);
}

void read_chanel(int channel){
  double servo_control_buf = IBus.readChannel(channel);
  if (servo_control_buf>=900 && servo_control_buf<=2100)
  {
    servo_control[channel] = servo_control_buf;
  }
}

void read_control() {
  IBus.loop();
  read_chanel(0);
  read_chanel(1);
  read_chanel(2);
  read_chanel(3);
  read_chanel(4);
  read_chanel(5);
  read_chanel(6);
  read_chanel(7);
  read_chanel(8);
  read_chanel(9);
  read_chanel(10);
  read_chanel(11);
  control_mode_flag = (servo_control[6] > 1800 )? 3 : (servo_control[6] < 1200)? 1 : 2;
}

void hand_control_mode(){
   eileron_l.write(servo_control[0]);
   eileron_r.write(servo_control[3]);
   pgo_l.write(servo_control[1]);
   pgo_r.write(servo_control[4]);
   engine.write(servo_control[2]);
}

void stabilization_mode_data_update(double pitch_, double roll_){
  pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), pitch_);                         
  roll_ctrl_effect_2  = roll_pid.ctrl(((servo_control[9] - 1500.0) * (45.0/500.0)), roll_);
}

void control_servos(){
  new_pitch_p = 0.005 + ((servo_control[11] - 1000.0))/1000.0*0.02;
  new_roll_p = 0.02 + ((servo_control[10] - 1500.0))/1000.0*0.01;
  pitch_pid.set(new_pitch_p, 0.0, 0.0);
  roll_pid.set(new_roll_p, 0.0, 0.0000025);
  if(control_mode_flag == 1){
    hand_control_mode();
  }
  else{
    if(control_mode_flag == 2){
       pitch_ctrl_effect = pitch_ctrl_effect_2;
       roll_ctrl_effect  = roll_ctrl_effect_2;
    }
    roll_ctrl_effect  = borders(1, -1, roll_ctrl_effect);
    pitch_ctrl_effect = borders(1, -1, pitch_ctrl_effect);
    
    eilerons_ctrl = -500*roll_ctrl_effect + 1500;
    pgo_l_ctrl = -0.7 * 500*(pitch_ctrl_effect - 0.06) + 1500;
    pgo_r_ctrl = 0.7 * 500*(pitch_ctrl_effect - 0.5) + 1500;

    eilerons_ctrl = borders(2000, 1000, eilerons_ctrl);
    pgo_l_ctrl    = borders(2000, 1000, pgo_l_ctrl);
    pgo_r_ctrl    = borders(2000, 1000, pgo_r_ctrl);
    
    // Serial.println(eilerons_ctrl);
    engine.write(servo_control[2]);
    eileron_l.write(eilerons_ctrl);
    eileron_r.write(eilerons_ctrl);
    pgo_l.write(pgo_l_ctrl);
    pgo_r.write(pgo_r_ctrl);
  }
}

#endif
