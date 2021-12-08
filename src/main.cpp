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
//  RTC with eeprom, write and read JSON string to/from Eeprom via i2c
//
//  imported ADS111x library from UNCLERUS to native ESP code in FreeRTOS
//
//  connected ADS1115 i2c analog to digital converter
//
//##########################################################################//

static const char *TAG = "I2C test";

#include "esp_log.h"

#include <Arduino.h>

#include "defines.h"
#include "HAL_ESP32.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "MyESP32eeprom.h"

#include "driver/gpio.h"

#include "driver/i2c.h"
#include <my_ads111x.h>

/* // ADS1115 ADC Connect ADDR pin to GND
#define ADDR ADS111X_ADDR_GND */
// +-4.096V
#define GAIN ADS111X_GAIN_4V096

// Mytest interrupt defines
#define CONFIG_LED_PIN 2
#define ESP_INTR_FLAG_DEFAULT 0
#define CONFIG_BUTTON_PIN 32

// PCF8574A
// i2c address 0x20 is the PCF8574 chip on the left center of my board, connected to all output relays and output MOSFETS
#define PCF8574A_INTERRUPT_PIN GPIO_NUM_34
#define PCF8574A_ADDRESS 0x20
#define PCF8574A_INPUTMASK B11111000 // input mask, pin 7 is input to interrupt from PCF8574B all other pins are outputs

// debounce interrupt
long debouncing_time = 100;
volatile unsigned long last_micros;

//  #include <string.h>

//#define RTCon             // use RTC3231 in this code

TaskHandle_t RTC_task_handle = NULL;
TaskHandle_t Eeprom_task_handle = NULL;
TaskHandle_t ADS1115_task_handle = NULL;

TaskHandle_t PCF8574A_isr_task_handle = NULL;
TaskHandle_t PCF8574B_isr_task_handle = NULL;

SemaphoreHandle_t RTC_Semaphore; // create a Semaphore Handle

// MyInterrupt test
volatile byte state = LOW;
TaskHandle_t TaskHandle_2;
SemaphoreHandle_t xBinarySemaphore;       // create a Myinterrupt test situation
SemaphoreHandle_t PCF8574A_isr_Semaphore; // create a Semaphore for PCF8574A Input ^^interrupt

volatile uint8_t InputState[INPUTS_TOTAL] = {255};

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

  while (1)
  {
    printf("Hello world!\n");
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void ads1115_task(void *arg)
{
  bool busy = true;
  int16_t raw;
  esp_err_t ret = -1;
  /*  ret = hello_stupid(32);         // test if this threat calls anything ??
  ESP_LOGI(TAG, "hello Stupid my i: %d", ret); */
  if (hal.Geti2cMutex())
  {
    if ((ret = ads111x_set_mode(I2C_NUM_0, ADDR, ADS111X_MODE_CONTINUOUS)) != ESP_OK)
      ESP_LOGD(TAG, "failed to write ADS111x mode %d", ret);

    if (ads111x_set_data_rate(I2C_NUM_0, ADDR, ADS111X_DATA_RATE_32) != ESP_OK)
      ESP_LOGD(TAG, "failed to set ads111x data rate");

    if (ads111x_set_input_mux(I2C_NUM_0, ADDR, ADS111X_MUX_0_GND) != ESP_OK)
      ESP_LOGD(TAG, "failed to set ads111x input MUX");
    if (ads111x_set_gain(I2C_NUM_0, ADDR, GAIN) != ESP_OK)
      ESP_LOGD(TAG, "failed to set ads111x GAIN");
    Serial.println(" Did ADS111x setup ");
    hal.Releasei2cMutex();
  }

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // wait for conversion end
    /*     do
        {
          ads111x_is_busy(I2C_NUM_0, ADDR, &busy);
        } while (busy); */
    if (hal.Geti2cMutex())
    {
      ads111x_is_busy(I2C_NUM_0, ADDR, &busy); // To Do a non blocking way to wait if ADS111x is busy
                                               /*         Serial.print(" !!!!!!!!!!!!!!!!  ADS111x busy value: ");
                                                       Serial.println(busy); */

      // Read result
      ads111x_get_value(I2C_NUM_0, ADDR, &raw);
      hal.Releasei2cMutex();
      float gain_val = ads111x_gain_values[GAIN];
      float voltage = gain_val / ADS111X_MAX_VALUE * raw;
      ESP_LOGI(TAG, " Raw ADC value: %d now formatted:  %.04f in volts", raw, voltage);
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL); // Delete this task if it exits from the loop above
}

void eeprom_task(void *arg)
{

  static int8_t j = 38;
  static char page_write_data[100] = "{'my':100,'due':23,'tres':89,'my1':101,'due1':24,'tres1':90}\0";
  // static char page_write_data[100] = "{'my':100,'due':23,'tres':89,'my1':101,'due1':24,'tres1':90,'quatro':12}\0"; // strlen = 72 + 1 for \0
  // static char page_write_data[100] = "{'my':100,'due':23,'tres':89,'my1':101,'due1':24,'tres1':90,'quatro':12,'my2':102,'due2':25,'tres2':91,'quatro2':14,'cinco:1,'seix':78}\0";  // strlen = 135 + 1 for \0

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    const uint8_t eeprom_address = 0x57;
    const uint16_t starting_address = 0x0000;

    // EEPROM single byte write example
    uint8_t single_write_byte = 0x40;
    eeprom_write_byte(I2C_NUM_0, eeprom_address, starting_address, 0x40);
    printf("Wrote byte 0x%02X to address 0x%04X\n", single_write_byte, starting_address);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // EEPROM random read example
    uint8_t random_read_byte = eeprom_read_byte(I2C_NUM_0, eeprom_address, starting_address);
    printf("Read byte 0x%02X at address 0x%04X\n", random_read_byte, starting_address);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // EEPROM page write example
    // char* page_write_data = "EEPROM page writing allows long strings of text to be written quickly, and with a lot less write cycles!\0";

    j++;
    page_write_data[8] = (char)j;
    eeprom_write(I2C_NUM_0, eeprom_address, starting_address, (uint8_t *)page_write_data, strlen(page_write_data) + 1);
    printf("Wrote the following string to EEPROM: %s\n", page_write_data);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    // EEPROM sequential read example and error checking
    uint8_t *sequential_read_data = (uint8_t *)malloc(strlen(page_write_data) + 1);
    esp_err_t ret = eeprom_read(I2C_NUM_0, eeprom_address, starting_address, sequential_read_data, strlen(page_write_data) + 1);

    if (ret == ESP_ERR_TIMEOUT)
      printf("I2C timeout...\n");
    if (ret == ESP_OK)
      printf("The read operation was successful!\n");
    else
      printf("The read operation was not successful, no ACK recieved.  Is the device connected properly?\n");

    printf("Read the following string from EEPROM: %s\n", (char *)sequential_read_data);

    free(sequential_read_data);

    vTaskDelay(1);
  }
  vTaskDelete(NULL); // Delete this task if it exits from the loop above
}

void RTC_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(RTC_Semaphore, portMAX_DELAY)) //( TickType_t ) 10 ) == pdTRUE )
    {
      // We were able to obtain the semaphore and can now access the shared resource.

      ESP_LOGI(TAG, "RTC_Semaphore obtained \n");
    }
    else
    {
      // We could not obtain the semaphore and can therefore not access
      // the shared resource safely.
      ESP_LOGI(TAG, "RTC_Semaphore NOT obtained \n");
    }
    // Wait 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "RTC_task gives Sephamore back \n");
    /* We have finished accessing the shared resource.  Release the
    semaphore. */
    // xSemaphoreGive(RTC_Semaphore);
    vTaskDelay(1);
  }
  vTaskDelete(NULL); // Delete this task if it exits from the loop above
}

void PCF8574A_isr_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    xSemaphoreTake(PCF8574A_isr_Semaphore, portMAX_DELAY);

    ESP_LOGD(TAG, "PCF8574A_isr_task is fired");

    // Read ports
    // The PCF8574A deals with inputs P0 to P5 and with pulse Relay over P6 and P7
    // Emergency STOP is connected to P0
    uint8_t v = hal.ReadPCF8574AInputRegisters();
    hal.writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_INPUTMASK); // set the input register again
    ESP_LOGD(TAG, "PCF8574A input register %d", v);
    v = (~v) & B11111000; // B00111111; // we invert the input bits here and test for LOW or we dont and test for HIGH
    ESP_LOGD(TAG, "PCF8574A input register after bit inversion %d", v);

    InputState[0] = (v & B00000000) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P0 = J11 pin 1 Emergency Stop
    InputState[1] = (v & B00000001) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P1 = J11 pin 3
    InputState[2] = (v & B00000010) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P2 = J12 pin 1
    InputState[3] = (v & B00000100) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P3 = J12 pin 3
    InputState[4] = (v & B00001000) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P4 = J18 pin 1
    InputState[5] = (v & B00010000) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P5 = J18 pin 2
    InputState[6] = (v & B00100000) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P6 = pulseRelay
    InputState[7] = (v & B01000000) == 0 ? INPUT_LOW : INPUT_HIGH;
    // P7 = pulseRelay
    InputState[8] = (v & B10000000) == 0 ? INPUT_LOW : INPUT_HIGH;

    // Emergency Stop (J1) has triggered
    if (InputState[0] == INPUT_LOW)
    {
      // emergencyStop = true; // test disabled
      // emergencyStop = false;
      ESP_LOGI(TAG, "Input all register on 0");
      InputState[0] = INPUT_HIGH;
    }

    if (InputState[1] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin1 PCF8574A triggered");
      InputState[1] = INPUT_HIGH;
    }

    if (InputState[2] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin2 PCF8574A triggered");
      InputState[2] = INPUT_HIGH;
    }

    if (InputState[3] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin3 PCF8574A triggered");
      InputState[3] = INPUT_HIGH;
    }

    if (InputState[4] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin4 PCF8574A triggered");
      InputState[4] = INPUT_HIGH;
    }
    if (InputState[5] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin5 PCF8574A triggered");
      InputState[5] = INPUT_HIGH;
    }
    if (InputState[6] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin6 PCF8574A triggered");
      InputState[6] = INPUT_HIGH;
    }
    if (InputState[7] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin7 PCF8574A triggered");
      InputState[7] = INPUT_HIGH;
    }
    if (InputState[8] == INPUT_LOW)
    {
      ESP_LOGI(TAG, "Input pin8 PCF8574A triggered");
      InputState[8] = INPUT_HIGH;
    }
  }
}

void PCF8574B_isr_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGD(TAG, "PCF8574B_isr");

    // Read ports
    // The 9534 deals with internal LED outputs and spare IO on J10
    uint8_t v = hal.ReadPCF8574BInputRegisters();

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

// Triggered when PCF8574A INT pin goes LOW
void PCF8574AInterrupt()
{
  /*   SERIAL_DEBUG.println("PCF8574A interrupt fired");
    if (PCF8574A_isr_task_handle != NULL)
    {
      xTaskNotifyFromISR(PCF8574A_isr_task_handle, 0x00, eNotifyAction::eNoAction, pdFALSE);
      SERIAL_DEBUG.println("PCF8574A interrupt fired task handle != NULL");
    } */
  //--

  BaseType_t taskYieldRequired = 0;

  // Serial.println(F("ISR Resuming PCF8574Interrupt"));

  if ((long)(micros() - last_micros) >= debouncing_time * 1000)
  {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* un-block the interrupt processing task now */
    xSemaphoreGiveFromISR(PCF8574A_isr_Semaphore, &xHigherPriorityTaskWoken);

    Serial.println(F("Leaving External Interrupt"));
    last_micros = micros();
  }

  if (taskYieldRequired == 1) // If the taskYield is reuiqred then trigger the same.
  {
    taskYIELD();
  }
}

// Triggered when PCF8574BA INT pin goes LOW
void PCF8574BInterrupt()
{
  if (PCF8574B_isr_task_handle != NULL)
  {
    // xTaskNotifyFromISR(PCF8574B_isr_task_handle, 0x00, eNotifyAction::eNoAction, pdFALSE);
    ;
  }
}
void IRAM_ATTR MyISR_Interrupt()
{
  // SERIAL_DEBUG.println("My Interrupt fired !");
  /* */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* un-block the interrupt processing task now */
  xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
}

void ISRprocessing(void *parameter) /* this function will be invoked when additionalTask was created */
{
  Serial.println((char *)parameter);
  /* loop forever */
  for (;;)
  {
    /* task move to Block state to wait for interrupt event */
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    Serial.println("ISRprocessing is running");
    /* toggle the LED now */
    state = !state;
    digitalWrite(2, state);
  }
  vTaskDelete(NULL);
}

static void ExternalInterrupt()
{
  static int count = 0;
  BaseType_t taskYieldRequired = 0;

  if (count <= 3)
  {
    count++;
  }
  // Serial.println(F("ISR Resuming Task2"));
  // xTaskNotify(RTC_task_handle, 0x00, eNotifyAction::eNoAction);
  // xTaskNotifyFromISR(TaskHandle_2, 0x00, eNotifyAction::eNoAction,NULL);
  if ((long)(micros() - last_micros) >= debouncing_time * 1000)
  {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* un-block the interrupt processing task now */
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
    // taskYieldRequired = xTaskResumeFromISR(TaskHandle_2);
    Serial.println(F("Leaving External Interrupt"));
    last_micros = micros();
  }

  if (taskYieldRequired == 1) // If the taskYield is reuiqred then trigger the same.
  {
    taskYIELD();
  }
}

void setup()
{

  // put your setup code here, to run once:
  hal.ConfigurePins();

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

  hal.ConfigureI2C(PCF8574AInterrupt, PCF8574BInterrupt);

  // attach a test interrupt with semaphore by native code
  /*   gpio_pad_select_gpio(CONFIG_BUTTON_PIN);   // buttonpin / interrupt pin
    gpio_pad_select_gpio(CONFIG_LED_PIN);    // LED pin
    // set the correct direction
    gpio_set_direction((gpio_num_t) CONFIG_BUTTON_PIN,(gpio_mode_t) GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)CONFIG_LED_PIN,(gpio_mode_t) GPIO_MODE_OUTPUT);

    // enable interrupt on falling (1->0) edge for button pin
    gpio_set_intr_type((gpio_num_t)CONFIG_BUTTON_PIN, GPIO_INTR_NEGEDGE);


    //Install the driver’s GPIO ISR handler service, which allows per-pin GPIO interrupt handlers.
    // install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // attach the interrupt service routine
    gpio_isr_handler_add((gpio_num_t)CONFIG_BUTTON_PIN, ExternalInterrupt, NULL); */
  pinMode(34, INPUT);
  // attachInterrupt(34, ExternalInterrupt, FALLING);
  attachInterrupt(PCF8574A_INTERRUPT_PIN, PCF8574AInterrupt, FALLING);
  /* initialize binary semaphore */
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreate(
      ISRprocessing,   /* Task function. */
      "ISRprocessing", /* name of task. */
      1000,            /* Stack size of task */
      NULL,            /* parameter of the task */
      1,               /* priority of the task */
      &TaskHandle_2);

  PCF8574A_isr_Semaphore = xSemaphoreCreateBinary();
  xTaskCreate(PCF8574A_isr_task, "PCF8574A", 2048, nullptr, configMAX_PRIORITIES - 3, &PCF8574A_isr_task_handle);
  // xTaskCreate(PCF8574B_isr_task, "PCF8574B", 2048, nullptr, configMAX_PRIORITIES - 3, &PCF8574B_isr_task_handle);
  // setOutput state directly
  hal.SetOutputState(0, (RelayState)0);
  hal.SetOutputState(1, (RelayState)0x99);
  hal.SetOutputState(2, (RelayState)0xFF);
  /*   // setOutput state in a loop
    for (int i = 0; i < 3; i++)
    {
      hal.SetOutputState(i, (RelayState)255);
    } */
  // configure freeRTOS
  RTC_Semaphore = xSemaphoreCreateBinary(); // create a binary Semaphore
  if (RTC_Semaphore == NULL)
  {
    // There was insufficient FreeRTOS heap available for the semaphore to
    // be created successfully.
    ESP_LOGD(TAG, "create RTC_Semaphore failed");
  }
  else
  {
    // The semaphore can now be used. Its handle is stored in the
    // xSemahore variable.  Calling xSemaphoreTake() on the semaphore here
    // will fail until the semaphore has first been given.
    ESP_LOGD(TAG, "create RTC_Semaphore success !");
  }
  // xTaskCreate(TaskLed,  "Led", 128, NULL, 0, NULL );         // example
  xTaskCreate(RTC_task, "RTCcode", 2048, nullptr, 1, &RTC_task_handle);
  // xTaskCreate(RTC_task, "RTC_task", 2048, NULL, 0, NULL); // this works too
  // xSemaphoreGive(RTC_Semaphore);
  xTaskCreate(&eeprom_task, "eeprom_read_write_demo", 1024 * 2, NULL, 5, &Eeprom_task_handle);

  // configure ADS1115 16bit AD converter
  xTaskCreate(&ads1115_task, "ads1115_demo", 1024 * 2, NULL, 5, &ADS1115_task_handle);

} // end SetUp

unsigned long wifitimer = 0;

unsigned long taskinfotimer = 0;

void loop()
{

  // vTaskSuspend(NULL); // suspend loop task no matter what
  unsigned long currentMillis = millis();

  // on first pass wifitimer is zero
  if (currentMillis - wifitimer > 10000)
  {

    wifitimer = currentMillis;
    SERIAL_DEBUG.println("WiFitimer");
    // xTaskNotify(RTC_task_handle, 0x00, eNotifyAction::eNoAction);
    xTaskNotify(Eeprom_task_handle, 0x00, eNotifyAction::eNoAction);
    xTaskNotify(ADS1115_task_handle, 0x00, eNotifyAction::eNoAction);
    xSemaphoreGive(RTC_Semaphore);
  }

  // xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);

  // put your main code here, to run repeatedly:
} // end loop