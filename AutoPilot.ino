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

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis

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
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("control");
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
//    vTaskDelay(1);
  }
}

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("data_update");
  vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
//  vTaskDelay(1);
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
//  delay(500);
//  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
