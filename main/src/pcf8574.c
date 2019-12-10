//
// Created by rarvolt on 07.12.2019.
//

#include <stdlib.h>

#include <freertos/FreeRTOS.h>
#include <driver/i2c.h>

#include "pcf8574.h"


#define PCF_WRITE(addr) ((addr) << 1 | I2C_MASTER_WRITE)
#define PCF_READ(addr) ((addr) << 1 | I2C_MASTER_READ)

#define ACK_CHECK_EN  1
#define ACK_CHECK_DIS 0

struct PCF8574
{
    PCF8574_ADDR_t addr;
    uint8_t data;
};


PCF8574_handle_t pcf8574_init(PCF8574_ADDR_t addr)
{
    PCF8574_handle_t pcf = calloc(1, sizeof(struct PCF8574));

    pcf->addr = addr;
    pcf->data = pcf8574_read_byte(pcf);

    return pcf;
}

void pcf8574_destroy(PCF8574_handle_t pcf)
{
    free(pcf);
}

void pcf8574_write_byte(PCF8574_handle_t pcf, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, PCF_WRITE(pcf->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(PCF_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);
}

uint8_t pcf8574_read_byte(PCF8574_handle_t pcf)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, PCF_READ(pcf->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &pcf->data, I2C_MASTER_LAST_NACK))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(PCF_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    return pcf->data;
}

void pcf8574_set_pin(PCF8574_handle_t pcf, PCF8574_PIN_t pin, PCF8574_STATE_t state)
{
    pcf8574_read_byte(pcf);
    if (state == (PCF8574_HIGH & 1))
    {
        pcf->data |= (1 << pin);
    }
    else
    {
        pcf->data &= ~(1 << pin);
    }
    pcf8574_write_byte(pcf, pcf->data);
}

PCF8574_STATE_t pcf8574_get_pin(PCF8574_handle_t pcf, PCF8574_PIN_t pin)
{
    pcf8574_read_byte(pcf);
    return pcf->data & (1 << pin);
}
