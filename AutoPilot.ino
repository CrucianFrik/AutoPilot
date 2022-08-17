//test.ino
TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

void control(void* pvParameters){
  while(true){
  vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  while(true){
  /* Эта задача должна выполняться точно через каждые 10 миллисекунд. */
  vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_RATE_MS ) );
  }
}

void setup() {

  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"Task2",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"Task1",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
