//##########################################################################//
//
//  Found address: 32 (0x20)  PCF8574 port expander
//  Found address: 87 (0x57)  EEprom
//  Found address: 104 (0x68)   RTC DS3231
//
//  test for i2c bus on ESP32
//
//  connected PCF8574 port expander
//
//  connected DS3231 RTC clock
//
//  RTC with eeprom
//
//##########################################################################//

static const char *TAG = "I2C test";

#include "esp_log.h"

#include <Arduino.h>

#include "defines.h"
#include "HAL_ESP32.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"


/* #include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3231.h>
#include <string.h>

#define SDA_GPIO 21
#define SCL_GPIO 22 */

/* for normal hardware wire use below */
//#include <Wire.h> // must be included here so that Arduino library object file references work
//#include <RtcDS3231.h>
// RtcDS3231<TwoWire> Rtc(Wire);
/* for normal hardware wire use above */

SemaphoreHandle_t RTC_Semaphore;

//#define RTCon             // use RTC3231 in this code

TaskHandle_t RTC_task_handle = NULL;
TaskHandle_t tca6408_isr_task_handle = NULL;
TaskHandle_t tca9534_isr_task_handle = NULL;

volatile enumInputState InputState[INPUTS_TOTAL];

HAL_ESP32 hal;

/* #define countof(a) (sizeof(a) / sizeof(a[0])) // for RTC
// print time date for RTC
void printDateTime(const RtcDateTime &dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
} */

void hello_task(void *pvParameter)
{
 
	while(1)
	{
	    printf("Hello world!\n");
	    vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void RTC_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if( xSemaphoreTake( RTC_Semaphore, portMAX_DELAY))    //( TickType_t ) 10 ) == pdTRUE )
        {
            /* We were able to obtain the semaphore and can now access the
            shared resource. */

            /* ... */
            ESP_LOGI(TAG, "RTC_Semaphore obtained");


        }
        else
        {
            /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
            ESP_LOGI(TAG, "RTC_Semaphore NOT obtained");
        }
    // Wait 100ms
    //vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "RTC_task");
    /* We have finished accessing the shared resource.  Release the
    semaphore. */
    xSemaphoreGive( RTC_Semaphore );
    vTaskDelay(1);
  }
  vTaskDelete(NULL); //Delete this task if it exits from the loop above
}

void PCF8574A_isr_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGD(TAG, "tca6408_isr");

    /*     // Read ports A/B/C/D inputs (on PCF8574A)
        uint8_t v = hal.ReadTCA6408InputRegisters();
        // P0=A
        InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        // P1=B
        InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        // P2=C
        InputState[2] = (v & B00000100) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        // P3=D
        InputState[3] = (v & B00001000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH; */
  }
}

void tca9534_isr_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGD(TAG, "tca9534_isr");

    // Read ports
    // The 9534 deals with internal LED outputs and spare IO on J10
    uint8_t v = hal.ReadTCA9534InputRegisters();

    // P4= J13 PIN 1 = WAKE UP TFT FOR DISPLAYS WITHOUT TOUCH
    InputState[4] = (v & B00010000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
    // P6 = spare I/O (on PCB pin)
    InputState[5] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
    // P7 = Emergency Stop
    InputState[6] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

    // Emergency Stop (J1) has triggered
    if (InputState[6] == enumInputState::INPUT_LOW)
    {
      // emergencyStop = true; // test disabled
      // emergencyStop = false;
      ;
    }

    if (InputState[4] == enumInputState::INPUT_LOW)
    {
      // Wake screen on pin going low
      /*       if (tftwakeup_task_handle != NULL)
            {
              xTaskNotify(tftwakeup_task_handle, 0x00, eNotifyAction::eNoAction);
            } */
      ;
    }
  }
}

// Triggered when TCA6408 INT pin goes LOW
void IRAM_ATTR TCA6408Interrupt()
{
  if (tca6408_isr_task_handle != NULL)
  {
    // xTaskNotifyFromISR(tca6408_isr_task_handle, 0x00, eNotifyAction::eNoAction, pdFALSE);
    ;
  }
}

// Triggered when TCA9534A INT pin goes LOW
void IRAM_ATTR TCA9534AInterrupt()
{
  if (tca9534_isr_task_handle != NULL)
  {
    // xTaskNotifyFromISR(tca9534_isr_task_handle, 0x00, eNotifyAction::eNoAction, pdFALSE);
    ;
  }
}

void setup()
{

  // put your setup code here, to run once:

  esp_log_level_set("*", ESP_LOG_INFO);     // set all components to INFO level
  esp_log_level_set("*", ESP_LOG_DEBUG);    // set all components to WARN level
  esp_log_level_set("wifi", ESP_LOG_WARN);  // enable WARN logs from WiFi stack
  esp_log_level_set("dhcpc", ESP_LOG_WARN); // enable INFO logs from DHCP client

  // ESP32 we use the USB serial interface for console/debug messages
  SERIAL_DEBUG.begin(115200, SERIAL_8N1);
  SERIAL_DEBUG.setDebugOutput(true);
  SERIAL_DEBUG.println("");
  SERIAL_DEBUG.println("Serial active");
  SERIAL_DEBUG.print("compiled: ");
  SERIAL_DEBUG.print(__DATE__);
  SERIAL_DEBUG.println(__TIME__);

  // ESP_LOGI(TAG, "CONTROLLER - ver:%s compiled %s", GIT_VERSION, COMPILE_DATE_TIME);

  // esp_chip_info_t chip_info;
  // esp_chip_info(&chip_info);

  // ESP_LOGI(TAG, "ESP32 Chip model = %u, Rev %u, Cores=%u, Features=%u", chip_info.model, chip_info.revision, chip_info.cores, chip_info.features);

  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);

  // xTaskCreate(tca6408_isr_task, "tca6408", 2048, nullptr, configMAX_PRIORITIES - 3, &tca6408_isr_task_handle);
  // xTaskCreate(tca9534_isr_task, "tca9534", 2048, nullptr, configMAX_PRIORITIES - 3, &tca9534_isr_task_handle);
  hal.SetOutputState(0, (RelayState)0);
  hal.SetOutputState(1, (RelayState)0x99);
  hal.SetOutputState(2, (RelayState)0x99);
  for (int i = 3; i < 7; i++)
  {
    hal.SetOutputState(i, (RelayState)255);
  }
  // configure freeRTOS
  RTC_Semaphore = xSemaphoreCreateBinary();
      if( RTC_Semaphore == NULL )
    {
        /* There was insufficient FreeRTOS heap available for the semaphore to
        be created successfully. */
        ESP_LOGD(TAG, "create RTC_Semaphore failed");
    }
    else
    {
        /* The semaphore can now be used. Its handle is stored in the
        xSemahore variable.  Calling xSemaphoreTake() on the semaphore here
        will fail until the semaphore has first been given. */
        ESP_LOGD(TAG, "create RTC_Semaphore success !");
    }
  // xTaskCreate(TaskLed,  "Led", 128, NULL, 0, NULL );         // example
  xTaskCreate(RTC_task, "RTCcode", 2048, nullptr, 1, &RTC_task_handle);
  // xTaskCreate(RTC_task,  "RTC_task", 2048, NULL, 0, NULL );

  // ESP_ERROR_CHECK(i2cdev_init());
  // xTaskCreate(ds3231_test, "ds3231_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

#ifdef RTCon
  // ************************************
  // ** RTC Setup
  // ************************************
  void RTC_Update()
  {
    char datetime[20];
    Serial.print(" this is RTC_Update() ");
    // Do udp NTP lookup, epoch time is unix time - subtract the 30 extra yrs (946684800UL) library expects 2000
    /* timeClient.update();
     if (timeClient.update())  {
       Serial.print("timeclient update OK ");      // Now we can set time in RTC chip
       Serial.println(timeClient.getFormattedTime());
       unsigned long epochTime = timeClient.getEpochTime();
       Serial.print(" timeClient epoch time : ");
       Serial.println(epochTime);
       Rtc.SetDateTime(epochTime - 946684800UL);     // set RTC with epochTime minus 30 years, becaise epochtime counts from 1970 and RTC3231 counts from year 2000
       RtcDateTime currTime = Rtc.GetDateTime();
       Serial.print(" RTC time  : ");
       Serial.print(currTime);
       Serial.print(" : humantime");
       printDateTime(currTime, datetime);
       Serial.println(datetime);
     } else {  */

    unsigned long epochTime = 1622865389;
    Serial.print(" timeClient epoch time : ");
    Serial.println(epochTime);
    Rtc.SetDateTime(epochTime - 946684800UL); // set RTC with epochTime minus 30 years, becaise epochtime counts from 1970 and RTC3231 counts from year 2000    Serial.println("timeclient update FAILED, time will load from RTC ");

    RtcDateTime currTime = Rtc.GetDateTime();
    Serial.print(" RTC time  : ");
    Serial.print(currTime);
    Serial.print(" : humantime");
    printDateTime(currTime, datetime);
    Serial.println(datetime);
    //}
  }
  /*  #ifdef DEBUG5
    if (timeClient.update())  {
      Serial.println("timeclient update OK ");
    } else {
      Serial.println("timeclient update FAILED");
    }
    Serial.print("formatted time : ");
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" epoch time : ");
    Serial.println(epochTime);
    //Rtc.SetDateTime(epochTime);
    currTime = Rtc.GetDateTime();
    Serial.print(" current time : ");
    Serial.println(currTime);
    Serial.print(" getdatetime and print it time : ");
    Serial.println(currTime);
    #endif */

  bool RTC_Valid()
  {
    bool boolCheck = true;
    if (!Rtc.IsDateTimeValid())
    {
      Serial.println("RTC lost confidence in the DateTime!  Updating DateTime");
      boolCheck = false;
      RTC_Update();
    }
    if (!Rtc.GetIsRunning())
    {
      Serial.println("RTC was not actively running, starting now.  Updating Date Time");
      Rtc.SetIsRunning(true);
      boolCheck = false;
      RTC_Update();
    }
    return boolCheck;
  }

  // Utility print function, sets a string with date and time to a char dateandtime[20] array in caling function
  void printDateTime(const RtcDateTime &dt, char *datestring)
  {
    // char datestring[20];
    snprintf_P(datestring,
               // countof(datestring),
               20,
               PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
               dt.Month(),
               dt.Day(),
               dt.Year(),
               dt.Hour(),
               dt.Minute(),
               dt.Second());
#ifdef DEBUG3
    Serial.println(" printDateTime() prints datestring ");
    Serial.println(datestring);
#endif

    /*
    // print string characters one by one
    char c;
    for (int i = 0; i<19; i++)  {
      c = *datestring++;
      Serial.print(c);
    }
    */
  }
#endif

} // end SetUp

unsigned long wifitimer = 0;

unsigned long taskinfotimer = 0;

void loop()
{


    unsigned long currentMillis = millis();


    // on first pass wifitimer is zero
    if (currentMillis - wifitimer > 2000)
    {

      wifitimer = currentMillis;
      SERIAL_DEBUG.println("WiFitimer");
      //xTaskNotify(RTC_task_handle, 0x00, eNotifyAction::eNoAction);
      xSemaphoreGive(RTC_Semaphore);

    }
    
    //xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);

  // put your main code here, to run repeatedly:
} // end loop