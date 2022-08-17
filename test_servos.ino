#include "all_data.h"

String ans;
const char sep =',';

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;
double elev_l_ctrl, elev_r_ctrl;

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

double roll_filter(double newVal) {
  static double filVal = 0.0;
  double k = 0.3; //коэффициент сглаживания (0.0 – 1.0), чем он меньше, тем плавнее фильтр
  filVal += (newVal - filVal) * k;
  return filVal;
}

void control(void* pvParameters){
  set_pids_borders();
  while(true){
    read_control();
    // >>>> ВТОРОЙ РЕЖИМ (ОБРАБОТКА ДАННЫХ С ПИДОВ) 
    // pitch_target_2      = vy_pid.ctrl(((servo_control[8] - 1500.0) * (5.0/500.0)), bmp_vertical_speed); // стоит ограничение по вертикальной скорости +-2 м/с на вход, +-15 градусов по тангажу на выход
    pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), pitch);                         
    roll_ctrl_effect_2  = roll_pid.ctrl(-((servo_control[9] - 1500.0) * (45.0/500.0)), roll);
    // <<<< ВТОРОЙ РЕЖИМ (ОБРАБОТКА ДАННЫХ С ПИДОВ) 
    // >>>> ТРЕТИЙ РЕЖИМ (полностью автономный полет)
    pitch_target_3      = vy_pid.ctrl(((servo_control[8] - 1500.0) * (5.0/500.0)), -bmp_vertical_speed); // стоит ограничение по вертикальной скорости +-2 м/с на вход, +-15 градусов по тангажу на выход
    pitch_ctrl_effect_3 = pitch_pid.ctrl(pitch_target_3, pitch);                         
    roll_ctrl_effect_3  = roll_pid.ctrl(-((servo_control[9] - 1500.0) * (45.0/500.0)), roll);
  
    if(control_flag == 1){
      write_control_hand();
    }
    else{
      if(control_flag == 2){
        roll_ctrl_effect = roll_ctrl_effect_2;
        pitch_ctrl_effect = pitch_ctrl_effect_2;
      }
      else if(control_flag == 3){
        roll_ctrl_effect = roll_ctrl_effect_3;
        pitch_ctrl_effect = pitch_ctrl_effect_3;
      }
      roll_ctrl_effect  = borders(1, -1, roll_ctrl_effect);
      pitch_ctrl_effect = borders(1, -1, pitch_ctrl_effect);
      
      eilerons_ctrl = 500*roll_ctrl_effect + 1500;
      pgo_l_ctrl = -0.7 * 500*pitch_ctrl_effect + 1500;
      pgo_r_ctrl = 0.7 * 500*(pitch_ctrl_effect - 0.3) + 1500;

      eilerons_ctrl = -borders(2000, 1000, eilerons_ctrl);
      pgo_l_ctrl    = borders(2000, 1000, pgo_l_ctrl);
      pgo_r_ctrl    = borders(2000, 1000, pgo_r_ctrl);
      
      engine.write(servo_control[2]);
      eileron_l.write(eilerons_ctrl);
      eileron_r.write(eilerons_ctrl);
      pgo_l.write(pgo_l_ctrl);
      pgo_r.write(pgo_r_ctrl);
    }
//    }
    vTaskDelay(1);
  }
}

void data_update(void* pvParameters){
  while(true){
    Serial.print(pitch_buf);Serial.println(" ");
    
//    if(control_flag != 3){end_way_flag = 0;}
    // if (targetPos.lengh(Vec3D{lat(), lng(), 1.5}) < 5.0 && !end_way_flag){
    //       end_way_flag = 1;
    //       delay_piezo(300);
    //     }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // mpu.dmpGetAccel(&aa, fifoBuffer);
        // accel_x = aa.x; accel_y = aa.y; accel_z = aa.z;
        // accel_all = pow(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z, 0.5);
        // mpu.dmpGetGyro(&gg, fifoBuffer);
        // gyro_x = gg.x; gyro_y = gg.y; gyro_z = gg.z;
        roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / M_PI;
        pitch = -asin(2 * (q.w * q.y - q.x * q.z)) * 180 / M_PI;
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / M_PI;
       xSemaphoreGive(xBinarySemaphore);
     }

     if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
        if ((millis()-timer_bar) > 50){
          pitch_buf = pitch;
          update_flying_alt_bmp();
          bmp_vertical_speed = bmp_vy_filter();
 
           timer_bar = millis();
        }
       xSemaphoreGive(xBinarySemaphore);
     }
    }
 vTaskDelay(1);
  }
}
void setup() {
  Serial.begin(115200);
//   init_gps(); calibrate_gps_alt();
  init_piezo();
  if (init_bmp()){delay_piezo(100);}calibrate_bmp();
  delay(100);
  if (init_mpu()){delay_piezo(100);}
  delay(650);
//   if (init_sd()){delay_piezo(100);}
  init_control(); 
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"Task2",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"Task1",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
