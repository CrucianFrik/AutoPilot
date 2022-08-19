//test.ino
#include "src/hand_control/all_data.h"
#include "src/mpu9250_and_bmp/mpu9250_wrapper.h"
#include "src/mpu9250_and_bmp/bmp180.h"
#include "src/sd/SdFat_wrapper.h"
#include "src/gps/gps.h"
#include "src/piezo/piezo.h"
#include "init.h"
#include "Filter.h"

#define DATA_UPT_TASK_PERIOD  1  //millis
#define CONTROL_TASK_PERIOD   10 //millis
#define LOG_TASK_PERIOD       10 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
SemaphoreHandle_t xBinarySemaphore;

//globals control/log core
float roll_core1=0.0;
float pitch_core1=0.0;
float yaw_core1=0.0;
float altitude_core1;
float altitude;

double latitude, longitude;
String filestr;

Filter Rollfilter{0.15, 0.07};
Filter PitchFilter{0.15, 0.07};

//Baro/gyro flag
int bar_flag = 0;


void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    update_gps();
    latitude  = lat();
    longitude = lng();
    read_control();                    // read IBas data
    stabilization_mode_data_update(pitch, roll);
    control_servos();
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

 void logs(void* pvParameters){
   portTickType xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount();
   float current_roll=0.0,current_pitch=0.0,current_yaw=0.0;
   while(true){
     if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
       pitch_core1=pitch;
       roll_core1=roll;
       yaw_core1=yaw;
       altitude_core1 = altitude;
       xSemaphoreGive(xBinarySemaphore);
     }    
     //log stream creation
     if(INIT_ALL_LOGS){
      String log_data="";
      log_data += String(millis());
      log_data += SEP + String(roll_core1);
      log_data += SEP + String(pitch_core1);
      log_data += SEP + String(yaw_core1);
      log_data += SEP + String(altitude_core1);
      if(INIT_GPS){
        log_data += SEP + String(latitude, 9);
        log_data += SEP +  String(longitude, 9);
      }

      //log string write
      sd_write(filestr, log_data + END_SEP);
     }
      vTaskDelayUntil( &xLastWakeTime, ( LOG_TASK_PERIOD / portTICK_RATE_MS ) );
   }
 }

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  
  while(true){
    bar_flag = (bar_flag+1)%5; // update counter

    if (bar_flag==0){ //read baro
      float altitude_core0=alt()
      if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
         altitude = altitude_core0;
         xSemaphoreGive(xBinarySemaphore);
       } 
    }
    else{ //read gyro
        VectorFloat angles =  get_mpu9250_data();
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
          roll = Rollfilter.get_filtered(angles.x);
          pitch = PitchFilter.get_filtered(angles.y);
          yaw = angles.z;
        xSemaphoreGive(xBinarySemaphore);
        }
          
    }
   vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);
  filestr="/data.txt";
  if(INIT_GPS){init_gps(); delay_piezo(1000);}
  init_mpu9250();
  if(INIT_ALL_LOGS){sd_init(); sd_write(filestr, HEADER+"\n"); delay_piezo(100);}
  delay(1000);
  init_bmp(); 
  init_control();
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,1);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,0); 
  delay(500);
   xTaskCreatePinnedToCore(logs,"logs",10000,NULL,9,&Task3,0); 
   delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
