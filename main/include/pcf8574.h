//
// Created by rarvolt on 07.12.2019.
//

#ifndef THUNDER_DETECT_PCF8574_H
#define THUNDER_DETECT_PCF8574_H

#include <stdint.h>
#include <stdbool.h>

#ifndef PCF_I2C_PORT
#define PCF_I2C_PORT I2C_NUM_0
#endif

typedef enum
{
    PCF8574_ADDR_000 = (0x40 >> 1),
    PCF8574_ADDR_001 = (0x42 >> 1),
    PCF8574_ADDR_010 = (0x44 >> 1),
    PCF8574_ADDR_011 = (0x46 >> 1),
    PCF8574_ADDR_100 = (0x48 >> 1),
    PCF8574_ADDR_101 = (0x4A >> 1),
    PCF8574_ADDR_110 = (0x4C >> 1),
    PCF8574_ADDR_111 = (0x4E >> 1)
} PCF8574_ADDR_t;

typedef enum
{
    PCF8574_P0 = 0,
    PCF8574_P1,
    PCF8574_P2,
    PCF8574_P3,
    PCF8574_P4,
    PCF8574_P5,
    PCF8574_P6,
    PCF8574_P7,
} PCF8574_PIN_t;

typedef enum
{
    PCF8574_LOW = 0,
    PCF8574_HIGH
} PCF8574_STATE_t;

typedef struct PCF8574* PCF8574_handle_t;


PCF8574_handle_t pcf8574_init(PCF8574_ADDR_t addr);
void pcf8574_destroy(PCF8574_handle_t pcf);
void pcf8574_write_byte(PCF8574_handle_t pcf, uint8_t data);
uint8_t pcf8574_read_byte(PCF8574_handle_t pcf);
void pcf8574_set_pin(PCF8574_handle_t pcf, PCF8574_PIN_t pin, PCF8574_STATE_t state);
PCF8574_STATE_t pcf8574_get_pin(PCF8574_handle_t pcf, PCF8574_PIN_t pin);


#endif //THUNDER_DETECT_PCF8574_H
