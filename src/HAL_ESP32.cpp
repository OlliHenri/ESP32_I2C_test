
#include <defines.h>
#include "HAL_ESP32.h"



uint8_t HAL_ESP32::readBytePCF8574(i2c_port_t i2c_num, uint8_t dev)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {

        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Send start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (B01000000 | ((dev << 1) | I2C_MASTER_READ)), I2C_MASTER_NACK);
        // Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        // ESP_LOGD(TAG,"I2C reply %i",ret);

        i2c_cmd_link_delete(cmd); 
        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

uint8_t HAL_ESP32::readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {

        uint8_t data;
        /*     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            //Select the correct register on the i2c device
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, reg, true);
            // Send repeated start, and read the register
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
            //Read single byte and expect NACK in reply
            i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            //esp_err_t ret =
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

            //ESP_LOGD(TAG,"I2C reply %i",ret);

            i2c_cmd_link_delete(cmd);
     */
        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeBytePCF8574(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t data)
{
    if (Geti2cMutex())
    {
        esp_err_t ret = -1;
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
        uint8_t buffer[2];
        buffer[0] = data;
        i2c_master_write(cmd, buffer, 1, I2C_MASTER_NACK);

        // i2c_master_write_byte(cmd, i2cregister, true);
        // i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);

        // esp_err_t ret =
        // ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();

        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

/* // i2c begin transmission
i2c_cmd_handle_t HAL_ESP32::beginI2C(i2c_port_t i2c_num, uint8_t deviceAddress)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
        esp_err_t ret; // To Do delete this only

        return cmd;
    }
    // missing error handler
}

// i2c send data ; i2c: Writes a single byte to a slave devices
esp_err_t HAL_ESP32::writeByteI2C(i2c_cmd_handle_t cmd, uint8_t data)
{
    uint8_t buffer[2];
    buffer[0] = data;
    i2c_master_write(cmd, buffer, 2, true);
}

// i2c read data; reads a singel byte
uint8_t HAL_ESP32::readByteI2C(i2c_cmd_handle_t cmd, uint8_t dev)
{

    uint8_t data;
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    // Read single byte and expect NACK in reply
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    return data;
}

// i2c end transmission ;  terminates i2c transmission
esp_err_t HAL_ESP32::endTransmissionI2C(i2c_port_t i2c_num, i2c_cmd_handle_t cmd)
{

    i2c_master_stop(cmd);

    esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(cmd);

    Releasei2cMutex();

    return ret;
} */

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        /*                 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                        i2c_master_start(cmd);
                        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
                        uint8_t buffer[2];
                        buffer[0] = i2cregister;
                        buffer[1] = data;
                        i2c_master_write(cmd, buffer, 2, true);

                        //i2c_master_write_byte(cmd, i2cregister, true);
                        //i2c_master_write_byte(cmd, data, true);
                        i2c_master_stop(cmd);

                        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
                        i2c_cmd_link_delete(cmd);  */

        Releasei2cMutex();

        esp_err_t ret; // To Do delete this only

        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

uint8_t HAL_ESP32::ReadPCF8574AInputRegisters()
{
    //PCF8574A_Value = readByte(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_INPUTMASK);
    PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
    return PCF8574A_Value & PCF8574A_INPUTMASK;
}

uint8_t HAL_ESP32::ReadPCF8574BInputRegisters()
{
    PCF8574B_Value = readByte(i2c_port_t::I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUTMASK);
    return PCF8574B_Value & PCF8574B_INPUTMASK;
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    ESP_LOGD(TAG, "SetOutputState bit:%u=%u", outputId, state);

    if (outputId <= 2)
    {
        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        ESP_LOGD(TAG, "PCF8574A reply before writing PCF8574A_Value = %i", PCF8574A_Value);
        uint8_t bit = outputId;
        PCF8574A_Value = (state == RelayState::RELAY_ON) ? (PCF8574A_Value | (1 << bit)) : (PCF8574A_Value & ~(1 << bit));
        ESP_ERROR_CHECK_WITHOUT_ABORT(writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_Value));
        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        ESP_LOGD(TAG, "PCF8574A reply after writing PCF8574A_Value = %i", PCF8574A_Value);
        // ESP_LOGD(TAG, "PCF8574B reply %i", ret);
        // TODO: Check return value
        /*         PCF8574B_Value = readByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUT);
                ESP_LOGD(TAG, "1 PCF8574A reply PCF8574A_Value = %i", PCF8574A_Value);
                PCF8574B_Value = readByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUT);
                ESP_LOGD(TAG, "2 PCF8574A reply PCF8574A_Value = %i", PCF8574A_Value);
                PCF8574B_Value = readByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUT);
                ESP_LOGD(TAG, "3 PCF8574A reply PCF8574A_Value = %i", PCF8574A_Value);
                PCF8574B_Value = readByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUT);
                ESP_LOGD(TAG, "4 PCF8574A reply PCF8574A_Value = %i", PCF8574A_Value); */
    }
}

void HAL_ESP32::Led(uint8_t bits)
{
    // Clear LED pins
    PCF8574B_Value = PCF8574B_Value & B11111000;
    // Set on
    PCF8574B_Value = PCF8574B_Value | (bits & B00000111);
    // esp_err_t ret =
    // ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_OUTPUT, PCF8574B_Value));

    // ESP_LOGD(TAG,"PCF8574B LED reply %i",ret);
    // TODO: Check return value
}

void HAL_ESP32::ConfigureCAN()
{
    // Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_16, gpio_num_t::GPIO_NUM_17, CAN_MODE_NORMAL);
    g_config.mode = CAN_MODE_NORMAL;

    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true};

    // Filter out all messages except 0x305 and 0x307
    // https://docs.espressif.com/projects/esp-idf/en/v3.3.5/api-reference/peripherals/can.html
    // 01100000101 00000 00000000 00000000 = 0x60A00000  (0x305)
    // 01100000111 00000 00000000 00000000 = 0x60E00000  (0x307)
    // 00000000010 11111 11111111 11111111 = 0x005FFFFF
    //          ^ THIS BIT IS IGNORED USING THE MASK SO 0x305 and 0x307 are permitted
    f_config.acceptance_code = 0x60A00000;
    f_config.acceptance_mask = 0x005FFFFF;

    // Install CAN driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver installed.  Filter=%u Mask=%u", f_config.acceptance_code, f_config.acceptance_mask);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to install CAN driver");
    }

    // Start CAN driver
    if (can_start() == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver started");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start CAN driver");
    }
}

// Control Silent mode control input on TJA1051T/3
// True = enable CANBUS
void HAL_ESP32::CANBUSEnable(bool value)
{
    // Pin P5
    // Low = Normal mode
    // High = Silent
    PCF8574B_Value = PCF8574B_Value & B11011111;

    if (value == false)
    {
        // Set on
        PCF8574B_Value = PCF8574B_Value | B00100000;
    }

    // ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_OUTPUT, PCF8574B_Value));
}

// Control TFT backlight LED
void HAL_ESP32::TFTScreenBacklight(bool value)
{
    // Clear LED pins
    PCF8574B_Value = PCF8574B_Value & B11110111;

    if (value == true)
    {
        // Set on
        PCF8574B_Value = PCF8574B_Value | B00001000;
    }

    // esp_err_t ret =
    // ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_OUTPUT, PCF8574B_Value));
    // TODO: Check return value
    // ESP_LOGD(TAG,"PCF8574B reply %i",ret);
}

void HAL_ESP32::ConfigurePins(void)
{

    // GPIO34 is interrupt pin from PCF8574A (doesnt have pull up/down resistors)
    pinMode(PCF8574A_INTERRUPT_PIN, INPUT);

    // GPIO34 is interrupt pin from PCF8574B (doesnt have pull up/down resistors)
    pinMode(PCF8574B_INTERRUPT_PIN, INPUT);

    // BOOT Button on ESP32 module is used for resetting wifi details
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    //attachInterrupt(GPIO_NUM_0, WiFiPasswordResetInterrupt, CHANGE);

    // For touch screen
    // GPIO_NUM_36 no internal PULLUP
    pinMode(TOUCH_IRQ, INPUT);
    // attachInterrupt(GPIO_NUM_36, TFTScreenTouch, FALLING);

    // Configure the CHIP SELECT pins as OUTPUT and set HIGH
    pinMode(TOUCH_CHIPSELECT, OUTPUT);
    digitalWrite(TOUCH_CHIPSELECT, HIGH);
    pinMode(SDCARD_CHIPSELECT, OUTPUT);
    digitalWrite(SDCARD_CHIPSELECT, HIGH);

    pinMode(RS485_ENABLE, OUTPUT);
    // Enable receive
    digitalWrite(RS485_ENABLE, LOW);
}

void HAL_ESP32::SwapGPIO0ToOutput()
{
    // BOOT Button on ESP32 module is used for resetting wifi details
    detachInterrupt(GPIO_NUM_0);
    pinMode(GPIO_NUM_0, OUTPUT);
    digitalWrite(GPIO_NUM_0, HIGH);
}

// test if i2c device is present
esp_err_t HAL_ESP32::test4i2cDevice(i2c_port_t i2c_num, uint8_t dev)
{
    esp_err_t ret = -1;

    if (Geti2cMutex())
    {
        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Send start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        // Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =

        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));

        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
        ESP_LOGI(TAG, "Test I2C for device: %i", dev);
        ESP_LOGD(TAG, "ESP_OK = 0, I2C reply %i", ret);
    }
    return ret;
}

/* // DS3231 get register
uint8_t HAL_ESP32::getRegI2C(uint8_t regAddress)
{
    esp_err_t ret = -1;
    i2c_cmd_handle_t cmd;

    cmd = beginI2C(I2C_NUM_0, DS3231_ADDRESS);
    writeByteI2C(cmd, regAddress);
    i2c_master_start(cmd);
    writeByteI2C(cmd, regAddress);
    uint8_t regValue = readByteI2C(cmd, DS3231_ADDRESS);
    ret = endTransmissionI2C(I2C_NUM_0, cmd);
    if (ret != 0)
    {
        return 0;
    }

    return regValue; // return register value
}

// DS3231 test if date time is valid
bool HAL_ESP32::IsDateTimeValid()
{
    uint8_t lastError = 0; //  To Do errorhandling
    uint8_t status = getRegI2C(DS3231_REG_STATUS);
    return (!(status & _BV(DS3231_OSF)) && (lastError == 0));
} */
// read DS3231 register
// *data pointer to data array
// num bytes = number of bytes to read 1 to x bytes

/* uint8_t HAL_ESP32::BcdToUint8(uint8_t val)
{
    return val - 6 * (val >> 4);
} */

uint8_t HAL_ESP32::BcdToUint8(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

uint8_t HAL_ESP32::Uint8ToBcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}
// read a register
esp_err_t HAL_ESP32::readRegDS3231(i2c_port_t i2c_num, uint8_t chip_addr, uint8_t Reg_Adr, uint8_t *data_rd, size_t num_bytes)
{
    esp_err_t ret = ESP_OK;
    if (Geti2cMutex())
    {
        int i = 0;
        if (num_bytes == 0)
        {
            return ESP_OK;
        }
        // first access the ic
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        //--
        if (chip_addr != -1)
        {
            i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
            i2c_master_write_byte(cmd, Reg_Adr, I2C_MASTER_ACK);
            i2c_master_start(cmd);
        }
        //--
        i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
        /*      if (num_bytes > 1)
                {
                    ret = i2c_master_read(cmd, data_rd, num_bytes - 1, I2C_MASTER_ACK);

                    ESP_LOGD(TAG, "readRegDS3231 data_rd ESP_OK = 0, I2C reply %i", ret);
                }
                // ret = i2c_master_read(cmd, data_rd, num_bytes, I2C_MASTER_LAST_NACK);
                ret = i2c_master_read_byte(cmd, data_rd + num_bytes - 1, I2C_MASTER_NACK); */
        ret = i2c_master_read(cmd, data_rd, num_bytes, I2C_MASTER_LAST_NACK);
        ESP_LOGD(TAG, "readRegDS3231 data_rd ESP_OK = 0, I2C reply %i", ret);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)); // 1000 / portTICK_RATE_MS);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", DS3231_ADDRESS, i2c_num, ret);
        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
    }
    return ret;
}
// set a register
esp_err_t HAL_ESP32::setRegDS3231(i2c_port_t i2c_num, uint8_t *Reg_Adr, size_t out_reg_size, uint8_t *data_rd, size_t num_bytes)
{
    esp_err_t ret = ESP_OK;
    if (Geti2cMutex())
    {
        int i = 0;
        if (num_bytes == 0)
        {
            return ESP_OK;
        }
        // first access the ic
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, DS3231_ADDRESS << 1, I2C_MASTER_ACK);
        i2c_master_write_byte(cmd, DS3231_REG_TIMEDATE, I2C_MASTER_ACK);
        i2c_master_write(cmd, data_rd, num_bytes, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)); // 1000 / portTICK_RATE_MS);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", Reg_Adr, i2c_num, ret);
        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
    }
    return ret;
}

// test 4 DS3231, needs further testing
esp_err_t HAL_ESP32::test4DS3231()
{

    esp_err_t ret = -1;
    if (Geti2cMutex())
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        if (DS3231_ADDRESS != -1)
        {
            i2c_master_write_byte(cmd, DS3231_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
            i2c_master_write_byte(cmd, DS3231_REG_STATUS, I2C_MASTER_ACK);
        }
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
    }
    return ret;
}

void HAL_ESP32::ConfigureI2C(void PCF8574AInterrupt(), void PCF8574BInterrupt())
{
    ESP_LOGI(TAG, "Configure I2C");



    // SDA / SCL
    // ESP32 = I2C0-SDA / I2C0-SCL
    // I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    // I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);
    //  Ollis I2C bus:  GPIO 21 (SDA) and GPIO 22 (SCL);

    // Initialize
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = gpio_num_t::GPIO_NUM_21; // 27;
    conf.scl_io_num = gpio_num_t::GPIO_NUM_22; // 26;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = 100000; // A BIT SLOWER FOR OUR DEVICES not 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "Config i2c passed");

    // search for DS3231 RTC
    esp_err_t ret = -1;
    if (ESP_OK == test4i2cDevice(I2C_NUM_0, DS3231_ADDRESS))
    {

        // ESP_LOGI(TAG, "PCF8574A read value: %i", PCF8574A_Value);

        ESP_LOGD(TAG, "DS3231 ESP_OK = 0, DS3231 found ");
    }
    else
    {
        ESP_LOGE(TAG, "DS3231 Error not found");
    }

    // test a second time with this routine
    /*     ret = test4DS3231();
        ESP_LOGD(TAG, "second test, DS3231 ESP_OK = 0, I2C reply: %i ", (int)ret); */

 

    // set time and date in DS3231
    uint8_t data_wr[8] = {0};
    data_wr[0] = Uint8ToBcd(45);    // seconds 0 - 60
    data_wr[1] = Uint8ToBcd(9);    // minutes 0 - 60
    data_wr[2] = Uint8ToBcd(21);    // hours values 00 - 23;    set 12h bit 6 with b0100 0000 = 0x40  | DS3231_12HOUR_FLAG
    data_wr[3] = Uint8ToBcd(1) + 1; // weekday, input 0 - 6 ; DS3231 accepts  1 - 7;
    data_wr[4] = Uint8ToBcd(28);    // mday 0 - 31
    /* data_wr[5] = Uint8ToBcd(4);    // month 01 - 12;  bit 7 is century bit */
    data_wr[6] = Uint8ToBcd(41);    // 00 - 99; if century rolls over set century flag
    // calculate the century flag
    uint8_t year = 2021 - 2000; // to calculate century flag - actual year to set minus 2000
    uint8_t centuryFlag = 0;
    if (year >= 100)
    {
        year -= 100;
        centuryFlag = _BV(7);
    }
    data_wr[5] = (Uint8ToBcd(11) | centuryFlag); // month 01 - 12;  bit 7 is century bit
    data_wr[6] = Uint8ToBcd(21);                 // 00 - 99; if century rolls over set century flag
    // now write the time date write buffer to DS3231
    ret = setRegDS3231(I2C_NUM_0, DS3231_REG_TIMEDATE, 1, data_wr, DS3231_REG_TIMEDATE_SIZE);

    // vTaskDelay(30 / portTICK_RATE_MS);   // if chip needs delay before next instruction

    uint8_t data_rd[8] = {0};
    ret = -1;
    // read the DS3231 time date register
    ret = readRegDS3231(I2C_NUM_0, DS3231_ADDRESS, DS3231_REG_TIMEDATE, data_rd, DS3231_REG_TIMEDATE_SIZE);
    ESP_LOGD(TAG, "readRegDS3231 ESP_OK = 0, I2C reply %i", ret);
    ESP_LOGI(TAG, "DS3231 data_rd bytes raw: sec %d min %d hour %d wday %d ", data_rd[0], data_rd[1], data_rd[2], data_rd[3]);
    // fill the buffer with what was read; convert to unix time structure
    uint8_t second = BcdToUint8(data_rd[0] & 0x7F); // seconds  & 0x7F);
    uint8_t minute = BcdToUint8(data_rd[1] & 0x7F); // minutes
    uint8_t hour = 0;
    if (data_rd[2] & DS3231_12HOUR_FLAG)
    {
        /* 12H */
        hour = BcdToUint8(data_rd[2] & DS3231_12HOUR_MASK); //  - 1; warum -1 ??
        /* AM/PM? */
        if (data_rd[2] & DS3231_PM_FLAG)
            hour += 12; // To Do this is not showing PM or AM, just converts to 24h format
    }
    else
        hour = BcdToUint8(data_rd[2] & DS3231_24HOUR_MASK); /* 24H */

    uint8_t wday = BcdToUint8(data_rd[3] & 0x07) - 1; // warum  - 1 ?, weil wir 0 - 6 benutzen und DS3231 1 - 7 speichert;
    uint8_t mday = BcdToUint8(data_rd[4] & 0x3F);
    uint8_t monthRaw = data_rd[5];
    uint16_t year_rd = BcdToUint8(data_rd[6]) + 2000;
    uint8_t isdst = 0; // timezone

    if (monthRaw & _BV(7)) // century wrap flag
    {
        year += 100;
    }
    uint8_t month = BcdToUint8(monthRaw & 0x7f);

    // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
    // applyTZ(time);

    ESP_LOGI(TAG, "DS3231 data_rd bytes: sec:%d min:%d hour:%d wday:%d mday:%d month:%d year:%d", second, minute, hour, wday, mday, month, year_rd);
    // uint8_t hour = BcdToBin24Hour(_wire.read());

    /*     if (!IsDateTimeValid())
        {
            ESP_LOGD(TAG, "DS3231 Date Time is valid ");
        }
        else
        {
            ESP_LOGD(TAG, "DS3231 Date Time is NOT !! valid ");
        } */

    /* Now PCF8574  */

    // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-PCF8574B_C206010.pdf
    // PCF8574B Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
    // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-PCF8574B_C206010.html
    // A0/A1/A2 are LOW, so i2c address is 0x38

    // PINS
    // P0= BLUE
    // P1= RED
    // P2= GREEN
    // P3= DISPLAY BACKLIGHT LED
    // P4= SPARE on J13
    // P5= Canbus RS
    // P6= SPARE on J13
    // P7= ESTOP (pull to ground to trigger)
    // INTERRUPT PIN = ESP32 IO34

    // BIT  76543210
    // PORT 76543210
    // MASK=10000000

    // search for PCF8574A
    ret = -1;
    if (ESP_OK == test4i2cDevice(I2C_NUM_0, PCF8574A_ADDRESS))
    {

        ret = writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_INPUTMASK);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PCF8574A Error");
            // Halt(RGBLED::Purple);
        }
        else
        {
            ESP_LOGI(TAG, "wrote INPUT MASK at PCF8574A");
            //attachInterrupt(PCF8574A_INTERRUPT_PIN, PCF8574AInterrupt, FALLING);
            // attachInterrupt(32, PCF8574AInterrupt, FALLING);
            //  attachInterrupt(33, PCF8574AInterrupt, FALLING);
            //   attachInterrupt(35, PCF8574AInterrupt, FALLING);
            ESP_LOGI(TAG, "PCF8574A interrupt attached");
        }
        // toggle pins 0 to 2 to test the routine
        /*     delay(1000);
            writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000111 );
                delay(1000);
            writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000000 );
                delay(1000);
            writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000111 );
                delay(1000);
            writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10001000 ); */

        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        ESP_LOGI(TAG, "PCF8574A read value: %i", PCF8574A_Value);

        ESP_LOGD(TAG, "ESP_OK = 0, I2C reply %i", ret);
    }
    else
    {
        ESP_LOGE(TAG, "PCF8574A Error not found");
    }
    /*
Now for the PCF8574B
*/ // P0=EXT_IO_A
    // P1=EXT_IO_B
    // P2=EXT_IO_C
    // P3=EXT_IO_D
    // P4=RELAY 1
    // P5=RELAY 2
    // P6=RELAY 3 (SSR)
    // P7=EXT_IO_E

    if (ESP_OK == test4i2cDevice(I2C_NUM_0, PCF8574B_ADDRESS))
    {
        // if PCF8574B is found set Input mask else abort
        ret = writeBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUTMASK);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PCF8574B Error");
            // Halt(RGBLED::Purple);
        }
        else
        {
            ESP_LOGI(TAG, "wrote INPUT MASK at PCF8574B");
            attachInterrupt(PCF8574B_INTERRUPT_PIN, PCF8574BInterrupt, FALLING);
            ESP_LOGI(TAG, "PCF8574B interrupt attached");
        }
        ESP_LOGD(TAG, "ESP_OK = 0, I2C reply %i", ret);
    }
    else
    {
        ESP_LOGE(TAG, "PCF8574B Error not found");
    }
}
