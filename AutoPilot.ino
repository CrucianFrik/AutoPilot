//test.ino
#include "all_data.h"
#include "src/mpu9250/I2Cdev.h"
#include "src/mpu9250/MPU9250_9Axis_MotionApps41.h"
#include "Wire.h"

MPU9250 mpu;

uint8_t mpuIntStatus,devStatus,fifoBuffer[64];
uint16_t packetSize,fifoCount,mag[3];
float f_mag[3],d_mag[4];
Quaternion q,q_mag;

#define DATA_UPD_PERIOD = 30 //millis
#define DATA_READING_PERIOD = 10 //millis
#define CTRL_SIGN_PERIOD = 50 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

long data_reading_timing=0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

double roll,pitch,yaw;
double curr_roll, curr_pitch, curr_yaw;

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

void control(void* pvParameters){
  while(true){
    pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), curr_pitch);                         
    roll_ctrl_effect_2  = roll_pid.ctrl(((servo_control[9] - 1500.0) * (45.0/500.0)), curr_roll);
    
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
      
      eilerons_ctrl = - 500*roll_ctrl_effect + 1500;
      pgo_l_ctrl = - 0.7 * 500*(pitch_ctrl_effect - 0.06) + 1500;
      pgo_r_ctrl = 0.7 * 500*(pitch_ctrl_effect - 0.5) + 1500;

      eilerons_ctrl = borders(2000, 1000, eilerons_ctrl);
      pgo_l_ctrl    = borders(2000, 1000, pgo_l_ctrl);
      pgo_r_ctrl    = borders(2000, 1000, pgo_r_ctrl);
      
      engine.write(servo_control[2]);
      eileron_l.write(eilerons_ctrl);
      eileron_r.write(eilerons_ctrl);
      pgo_l.write(pgo_l_ctrl);
      pgo_r.write(pgo_r_ctrl);
    }
  
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS && millis()-data_reading_timing > DATA_UPD_PERIOD ) {
      curr_roll = roll;
      curr_pitch = pitch;
      curr_yaw = yaw;
      data_reading_timing = millis();
      xSemaphoreGive(xBinarySemaphore);
      read_control();
    }
    
    vTaskDelayUntil( &xLastWakeTime, ( CTRL_SIGN_PERIOD / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  while(true){
  while (!mpuInterrupt && fifoCount < packetSize) {}
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); }
  else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      
      if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        roll=-atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))* 180/M_PI;
        pitch=asin(2*(q.w*q.y-q.x*q.z))* 180/M_PI;
        yaw=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))* 180/M_PI;
        xSemaphoreGive(xBinarySemaphore);
      }
  }
  vTaskDelayUntil( &xLastWakeTime, ( DATA_READING_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  mpu.initialize();
  pinMode(5, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(5), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();

  init_control(); 
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
