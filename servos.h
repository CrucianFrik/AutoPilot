#ifndef SERVA_H // For Petr
#define SERVA_H

#include <ESP32Servo.h>
#include "FlySkyIBus.h"

const int antenaPin     = 39;
const int engine_Pin    = 13;
const int eileron_l_pin = 19;
const int eileron_r_pin = 18;
const int pgo_l_pin     = 23;
const int pgo_r_pin     =  5;


HardwareSerial FSkySerial(1);
int control_mode_flag = 1; // Hand mode = 1

Servo eileron_l;  
Servo eileron_r; 
Servo pgo_l;
Servo pgo_r;
Servo engine;

double servo_control[10] = {1500,1500,1000,1500,1500,1500,1000,1500,1500,1500};

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
  control_mode_flag = (servo_control[6] > 1800 )? 3 : (servo_control[6] < 1200)? 1 : 2;
}

void hand_control_mode(){
   eileron_l.write(servo_control[0]);
   eileron_r.write(servo_control[3]);
   pgo_l.write(servo_control[1]);
   pgo_r.write(servo_control[4]);
   engine.write(servo_control[2]);
}

#endif
