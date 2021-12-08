#pragma once

#include <stdio.h>
#include "driver/i2c.h"

// Definitions for i2c
#define I2C_MASTER_SCL_IO   19
#define I2C_MASTER_SDA_IO   18
#define I2C_MASTER_NUM  I2C_NUM_1
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ  100000

#define EEPROM_WRITE_ADDR   0x00
#define EEPROM_READ_ADDR    0x01

#define EEPROM_PAGE_SIZE	32 // 64

/* void init_i2c_master(); */
esp_err_t eeprom_write_byte(i2c_port_t i2c_num, uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte);
esp_err_t eeprom_write(i2c_port_t i2c_num, uint8_t deviceaddress, uint16_t eeaddress, uint8_t* data, size_t size);

uint8_t eeprom_read_byte(i2c_port_t i2c_num, uint8_t deviceaddress, uint16_t eeaddress);
esp_err_t eeprom_read(i2c_port_t i2c_num, uint8_t deviceaddress, uint16_t eeaddress, uint8_t* data, size_t size);