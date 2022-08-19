#ifndef PIEZO_H
#define PIEZO_H

#define port 12

bool piezo_flag   = 0;
int piezo_timer = 0.0;

void init_piezo(){
  pinMode(port,OUTPUT);
}

void delay_piezo(int time_delay_in_micros){
  digitalWrite(port,HIGH);
  delay(time_delay_in_micros);
  digitalWrite(port,LOW);
}

void piezo_on(){
  digitalWrite(port,HIGH);
  piezo_flag = 1;
}

void piezo_off(){
  digitalWrite(port,LOW);
  piezo_flag = 0;
}

#endif