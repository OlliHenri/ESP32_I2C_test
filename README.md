# ESP32_I2C_test
I2C with native ESP code and PCF8574 and DS3231

test program for I2C on ESP32 using native ESP code

tested with PCF8574 portexpander and DS3231 real time clock on I2C bus

using FreeRTOS and a binary Semaphore to controll RTC routine
also tested and commented is a TaskNotification to call the RTC routine
