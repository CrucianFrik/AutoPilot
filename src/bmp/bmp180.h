#ifndef BMP180_H
#define BMP180_H
#include "Wire.h"
#include "I2Cdev.h"
#include <vector>
#include <Adafruit_BMP085.h>

#include "Filters.h"


#define ALT_BUF_SIZE       20
#define CALIBRATE_ZERO_BUF_SIZE 200
#define vy_BUF_SIZE        10

Adafruit_BMP085 bmp;

double bmp_zero_alt;
double bmp_vertical_speed, bmp_alt_time = 0.0;
double bmp_flying_alt = 0.0, bmp_flying_alt_old = 0.0;

double average_alt = 0.0;
double average_vy  = 0.0;

unsigned long timer_bar;

std::vector<double> alt_data_buf;
std::vector<double> vy_data_buf;

ExpRunAverFilter bmp_alt_filter{0.1, bmp_zero_alt};
ExpRunAverFilter bmp_vy_filter{0.3, 0.0};

bool init_bmp(){
  #ifdef I2C
  #define I2C
  Wire.begin();
  Wire.setClock(400000);
  #endif
  return bmp.begin(0);
}

double alt()
{
  return bmp.readAltitude();
}

double relative_alt()
{
  return (alt() - bmp_zero_alt);
}

void calibrate_bmp()
{
  double buf = 0.0;
  for(int i = 0; i < CALIBRATE_ZERO_BUF_SIZE; ++i){
    buf += alt();
  }
  bmp_zero_alt = buf/double(CALIBRATE_ZERO_BUF_SIZE);
}

double averaged_relative_alt(){
  average_alt -= alt_data_buf.front();
  alt_data_buf.erase(alt_data_buf.begin());
  double height = relative_alt();
  alt_data_buf.push_back(height);
  average_alt += height;
  return average_alt/double(ALT_BUF_SIZE);
}

double get_bmp_vertical_speed(){
  double cur_time = micros();
  double vy = (bmp_flying_alt_old - bmp_flying_alt)/((bmp_alt_time - cur_time)/1000000.0);
  bmp_flying_alt_old = bmp_flying_alt;
  bmp_alt_time = cur_time;
  return vy;
}


void update_flying_alt_bmp(){
  bmp_flying_alt = bmp_alt_filter.get_filtered(relative_alt());
}

#endif




