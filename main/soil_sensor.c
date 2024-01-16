#include "soil_sensor.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define STEMMA_ADDR 0x36
#define I2C_BUFSIZE 0
#define I2C_TIMEOUT_MS 1000

#define SOIL_BASE_ADDR 0x0F
#define SOIL_F_REG 0x10

#define RBUF_SIZE 2

#define readCount 5

// Function to set up the soil sensor
i2c_port_t setup_soil_sensor(int sda_pin, int scl_pin) {
  esp_err_t err;

  int i2c_master_port = I2C_MASTER_PORT;
    i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda_pin,
    .scl_io_num = scl_pin,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
  };

  err = i2c_param_config(i2c_master_port, &i2c_conf);
  if (err != ESP_OK) {
    printf("ERROR: Not able to set up i2c soil sensor: %x\n", err);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    return -1;
  }

  err = i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_BUFSIZE, I2C_BUFSIZE, 0);
  if (err != ESP_OK) {
    printf("ERROR: i2c driver install not ok: %x\n", err);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    return -1;
  }
  printf("I2C soil sensor was set up\n");
  return i2c_master_port;
}

/**
 * @brief 
 * 
 * @param port 
 * @return unsigned short
 * max value: 1023
 *  
 */
unsigned short read_soil_sensor(i2c_port_t port) {
  int sum = 0;
  int succes = 0;
  for (int i = 0; i < readCount; i++)
  {
    esp_err_t err;
    uint8_t wbuf[2] = {SOIL_BASE_ADDR, SOIL_F_REG};
    uint8_t rbuf[RBUF_SIZE];

    err = i2c_master_write_read_device(port, STEMMA_ADDR, wbuf, 2, rbuf, RBUF_SIZE, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      continue;
    }
    sum += ((uint16_t) rbuf[0]) << 8 | ((uint16_t) rbuf[1]);
    succes++;
  }
  if(succes == 0) {
    printf("ERROR: failed to read soil sensor %d times\n",readCount);
    return sum;
  }
  return sum / succes;
}