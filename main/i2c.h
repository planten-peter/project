#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "driver/i2c.h"

#define AM2320_I2C_ADDR 0x5C
#define STEMMA_I2C_ADDR 0x36

//general i2c functionality

i2c_port_t setup_i2c(int sda_pin, int scl_pin);

//stemma soil sensor functionality

unsigned short read_soil_sensor(i2c_port_t port);
void soil_sensor_example();

//am2320 temperatue and humidity sensor functionality

esp_err_t am2320_read_temp(i2c_port_t port, uint8_t am2320_addr, uint16_t* temp_reading);
void temp_sensor_example();

#endif //I2C_H_