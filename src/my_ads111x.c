/* ESP native i2c communication */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "my_ads111x.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp32

#define REG_CONVERSION 0
#define REG_CONFIG 1
#define REG_THRESH_L 2
#define REG_THRESH_H 3

#define COMP_QUE_OFFSET 1
#define COMP_QUE_MASK 0x03
#define COMP_LAT_OFFSET 2
#define COMP_LAT_MASK 0x01
#define COMP_POL_OFFSET 3
#define COMP_POL_MASK 0x01
#define COMP_MODE_OFFSET 4
#define COMP_MODE_MASK 0x01
#define DR_OFFSET 5
#define DR_MASK 0x07
#define MODE_OFFSET 8
#define MODE_MASK 0x01
#define PGA_OFFSET 9
#define PGA_MASK 0x07
#define MUX_OFFSET 12
#define MUX_MASK 0x07
#define OS_OFFSET 15
#define OS_MASK 0x01

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

static const char *TAG = "ads111x";

const float ads111x_gain_values[] = {
    [ADS111X_GAIN_6V144] = 6.144,
    [ADS111X_GAIN_4V096] = 4.096,
    [ADS111X_GAIN_2V048] = 2.048,
    [ADS111X_GAIN_1V024] = 1.024,
    [ADS111X_GAIN_0V512] = 0.512,
    [ADS111X_GAIN_0V256] = 0.256,
    [ADS111X_GAIN_0V256_2] = 0.256,
    [ADS111X_GAIN_0V256_3] = 0.256};

// static functions
static esp_err_t ads111x_read_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, uint16_t *val);
static esp_err_t ads111x_write_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, uint16_t val);
esp_err_t read_conf_bits(i2c_port_t i2c_num, uint8_t devaddr, uint8_t offs, uint16_t mask, uint16_t *bits);
esp_err_t write_conf_bits(i2c_port_t i2c_num, uint8_t devaddr, uint16_t val, uint8_t offs, uint16_t mask);

esp_err_t hello_stupid(int i)
{
    i *= 3;
    ESP_LOGE(TAG, "Hello Stupid i %d", i);
    return ESP_OK;
}

static esp_err_t ads111x_read_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t res;
    if ((res = i2c_ads111x_read_reg(i2c_num, devaddr, reg, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read from register 0x%02x", reg);
        return res;
    } /* else {
        ESP_LOGI(TAG, "Did read from register 0x%02x", reg);
    } */
    *val = (buf[0] << 8) | buf[1];

    return ESP_OK;
}

esp_err_t i2c_ads111x_read_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, void *in_data, size_t in_size)
{
    return i2c_ads111x_read(i2c_num, devaddr, &reg, 1, in_data, in_size);
}

esp_err_t i2c_ads111x_read(i2c_port_t i2c_num, uint8_t devaddr, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!in_data || !in_size)
        return ESP_ERR_INVALID_ARG;

    // SEMAPHORE_TAKE(i2c_num);     // To Do fix this Semaphore
    esp_err_t res = -1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (out_data && out_size)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, devaddr << 1, true);
        i2c_master_write(cmd, (void *)out_data, out_size, true);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devaddr << 1) | 1, true);
    i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", devaddr, i2c_num, res);
    /*     else
            ESP_LOGI(TAG, "Did read from device [0x%02x at %d]: %d", devaddr, i2c_num, res); */

    i2c_cmd_link_delete(cmd);
    // SEMAPHORE_GIVE(i2c_num);
    return res;
}
// write reg
static esp_err_t ads111x_write_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = {val >> 8, val};
    esp_err_t res;
    if ((res = i2c_ads111x_write_reg(i2c_num, devaddr, reg, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write 0x%04x to register 0x%02x", val, reg);
        return res;
    }
    return ESP_OK;
}

esp_err_t i2c_ads111x_write_reg(i2c_port_t i2c_num, uint8_t devaddr, uint8_t reg, const void *out_data, size_t out_size)
{
    return i2c_ads111x_write(i2c_num, devaddr, &reg, 1, out_data, out_size);
}

esp_err_t i2c_ads111x_write(i2c_port_t i2c_num, uint8_t devaddr, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!out_data || !out_size)
        return ESP_ERR_INVALID_ARG;

    // SEMAPHORE_TAKE(i2c_num);         // To Do fix this Semaphore
    esp_err_t res = -1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, devaddr << 1, true);
    if (out_reg && out_reg_size)
        i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
    i2c_master_write(cmd, (void *)out_data, out_size, true);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d", devaddr, i2c_num, res);
    i2c_cmd_link_delete(cmd);
    // SEMAPHORE_GIVE(dev->porti2c);
    return res;
}

// My read config bits
esp_err_t read_conf_bits(i2c_port_t i2c_num, uint8_t devaddr, uint8_t offs, uint16_t mask, uint16_t *bits)
{
    uint16_t val;
    esp_err_t res = -1;

    //  if (Geti2cMutex())        // To Do fix this Semaphore
    //  {
    if ((res = ads111x_read_reg(i2c_num, devaddr, REG_CONFIG, &val)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read from register 0x%02x", REG_CONFIG);
        return res;
    }
    //   Releasei2cMutex();

    // ESP_LOGD(TAG, "Got config value: 0x%04x", val);

    *bits = (val >> offs) & mask;

    return ESP_OK;
    //  }
}

// My write config bits
esp_err_t write_conf_bits(i2c_port_t i2c_num, uint8_t devaddr, uint16_t val, uint8_t offs, uint16_t mask)
{
    // ESP_LOGI(TAG, " enter write_conf_bits ");
    uint16_t old;

    esp_err_t res = -1;

    // if (Geti2cMutex())        // To Do fix this Semaphore
    //{
    if ((res = ads111x_read_reg(i2c_num, devaddr, REG_CONFIG, &old)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read from register 0x%02x", REG_CONFIG);
        return res;
    }
    /*     else
        {
            ESP_LOGI(TAG, " Did read Config register 0x%02x  ; old = %d", REG_CONFIG, old);
        } */
    if ((res = ads111x_write_reg(i2c_num, devaddr, REG_CONFIG, ((old & ~(mask << offs)) | (val << offs)))) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write 0x%04x to register 0x%02x", val, REG_CONFIG);
        return res;
    }
    /*         else
            {
                ESP_LOGI(TAG, " Did write to Config register 0x%02x ; new value = %d ", REG_CONFIG, ((old & ~(mask << offs)) | (val << offs)));
            }  */
    //    Releasei2cMutex();
    //}
    return ESP_OK;
}

#define READ_CONFIG(OFFS, MASK, VAR)                               \
    do                                                             \
    {                                                              \
        CHECK_ARG(VAR);                                            \
        uint16_t bits;                                             \
        CHECK(read_conf_bits(I2C_NUM_0, ADDR, OFFS, MASK, &bits)); \
        *VAR = bits;                                               \
        return ESP_OK;                                             \
    } while (0)

///////////////////////////////////////////////////////////////////////////////

// I2C_NUM_0, ADDR, I2C_MODE_MASTER

esp_err_t ads111x_is_busy(i2c_port_t i2c_num, uint8_t devaddr, bool *busy)
{
    READ_CONFIG(OS_OFFSET, OS_MASK, busy);
}

esp_err_t ads111x_start_conversion(i2c_port_t i2c_num, uint8_t devaddr)
{
    return write_conf_bits(i2c_num, devaddr, 1, OS_OFFSET, OS_MASK);
}

esp_err_t ads111x_get_value(i2c_port_t i2c_num, uint8_t devaddr, int16_t *value)
{
    ads111x_read_reg(i2c_num, devaddr, REG_CONVERSION, (uint16_t *)value);

    return ESP_OK;
}

esp_err_t ads101x_get_value(i2c_port_t i2c_num, uint8_t devaddr, int16_t *value)
{

    ads111x_read_reg(i2c_num, devaddr, REG_CONVERSION, (uint16_t *)value);

    *value = *value >> 4;
    if (*value > 0x07FF)
    {
        // negative number - extend the sign to 16th bit
        *value |= 0xF000;
    }
    return ESP_OK;
}

esp_err_t ads111x_get_gain(i2c_port_t i2c_num, uint8_t devaddr, ads111x_gain_t *gain)
{
    READ_CONFIG(PGA_OFFSET, PGA_MASK, gain);
}

esp_err_t ads111x_set_gain(i2c_port_t i2c_num, uint8_t devaddr, ads111x_gain_t gain)
{
    return write_conf_bits(i2c_num, devaddr, gain, PGA_OFFSET, PGA_MASK);
}

esp_err_t ads111x_get_input_mux(i2c_port_t i2c_num, uint8_t devaddr, ads111x_mux_t *mux)
{
    READ_CONFIG(MUX_OFFSET, MUX_MASK, mux);
}

esp_err_t ads111x_set_input_mux(i2c_port_t i2c_num, uint8_t devaddr, ads111x_mux_t mux)
{
    return write_conf_bits(i2c_num, devaddr, mux, MUX_OFFSET, MUX_MASK);
}

esp_err_t ads111x_get_mode(i2c_port_t i2c_num, uint8_t devaddr, ads111x_mode_t *mode)
{
    READ_CONFIG(MODE_OFFSET, MODE_MASK, mode);
}

esp_err_t ads111x_set_mode(i2c_port_t i2c_num, uint8_t devaddr, ads111x_mode_t mode)
{
    esp_err_t ret = -1;
    ret = write_conf_bits(i2c_num, devaddr, mode, MODE_OFFSET, MODE_MASK);
    return ret;
}

esp_err_t ads111x_get_data_rate(i2c_port_t i2c_num, uint8_t devaddr, ads111x_data_rate_t *rate)
{
    READ_CONFIG(DR_OFFSET, DR_MASK, rate);
}

esp_err_t ads111x_set_data_rate(i2c_port_t i2c_num, uint8_t devaddr, ads111x_data_rate_t rate)
{
    return write_conf_bits(i2c_num, devaddr, rate, DR_OFFSET, DR_MASK);
}

esp_err_t ads111x_get_comp_mode(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_mode_t *mode)
{
    READ_CONFIG(COMP_MODE_OFFSET, COMP_MODE_MASK, mode);
}

esp_err_t ads111x_set_comp_mode(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_mode_t mode)
{
    return write_conf_bits(i2c_num, devaddr, mode, COMP_MODE_OFFSET, COMP_MODE_MASK);
}

esp_err_t ads111x_get_comp_polarity(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_polarity_t *polarity)
{
    READ_CONFIG(COMP_POL_OFFSET, COMP_POL_MASK, polarity);
}

esp_err_t ads111x_set_comp_polarity(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_polarity_t polarity)
{
    return write_conf_bits(i2c_num, devaddr, polarity, COMP_POL_OFFSET, COMP_POL_MASK);
}

esp_err_t ads111x_get_comp_latch(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_latch_t *latch)
{
    READ_CONFIG(COMP_LAT_OFFSET, COMP_LAT_MASK, latch);
}

esp_err_t ads111x_set_comp_latch(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_latch_t latch)
{
    return write_conf_bits(i2c_num, devaddr, latch, COMP_LAT_OFFSET, COMP_LAT_MASK);
}

esp_err_t ads111x_get_comp_queue(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_queue_t *queue)
{
    READ_CONFIG(COMP_QUE_OFFSET, COMP_QUE_MASK, queue);
}

esp_err_t ads111x_set_comp_queue(i2c_port_t i2c_num, uint8_t devaddr, ads111x_comp_queue_t queue)
{
    return write_conf_bits(i2c_num, devaddr, queue, COMP_QUE_OFFSET, COMP_QUE_MASK);
}

esp_err_t ads111x_get_comp_low_thresh(i2c_port_t i2c_num, uint8_t devaddr, int16_t *th)
{

    ads111x_read_reg(i2c_num, devaddr, REG_THRESH_L, (uint16_t *)th);

    return ESP_OK;
}

esp_err_t ads111x_set_comp_low_thresh(i2c_port_t i2c_num, uint8_t devaddr, int16_t th)
{

    ads111x_write_reg(i2c_num, devaddr, REG_THRESH_L, th);

    return ESP_OK;
}

esp_err_t ads111x_get_comp_high_thresh(i2c_port_t i2c_num, uint8_t devaddr, int16_t *th)
{

    ads111x_read_reg(i2c_num, devaddr, REG_THRESH_H, (uint16_t *)th);

    return ESP_OK;
}

esp_err_t ads111x_set_comp_high_thresh(i2c_port_t i2c_num, uint8_t devaddr, int16_t th)
{

    ads111x_write_reg(i2c_num, devaddr, REG_THRESH_H, th);

    return ESP_OK;
}
