#include "i2c.h"

/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
*/


#define I2C_FREQ_HZ (100000) // 100kHz
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_TIMEOUT_MS 1000
#define I2C_SLV_BUFSIZE 0 //Only operating in master mode, buffer size 0

#define STEMMA_BASE_ADDR 0x0F
#define STEMMA_F_REG 0x10
#define STEMMA_RBUF_SIZE 2

#define AM2320_READ_CMD 0x03
#define AM2320_HUM_H 0x00
#define AM2320_HUM_L 0x01
#define AM2320_TEMP_H 0x02
#define AM2320_TEMP_L 0x03

#define AM2320_DELAY_T1 (2) //ms
#define AM2320_DELAY_T2 (3) //ms

/**
 * Wake up the AM2320 temperature and humidity sensor.
 * This is required *every time* befor reading the sensor, as the device is
 * otherwise sleeping to avoid using power (and skewing temperature readings)
 * @param port The port handle for the i2c port. See function `setup_i2c`
 * @param am2320_addr The I2C address of the AM2320 device
 * @return ESP_OK if everything was good, an error value otherwise
*/
esp_err_t wake_am2320(i2c_port_t port, uint8_t am2320_addr) {
  esp_err_t res;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create a command buffer for sending commands
  i2c_master_start(cmd); //Enqueue a start transition
  i2c_master_write_byte(cmd, am2320_addr << 1, true); //Write byte, include ack
  i2c_master_stop(cmd); //Enqueue stop command

  //Perform command
  res = i2c_master_cmd_begin(port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return res;
}

/**
 * Read the temperature from the am2320 sensor
 * @param port The port handle for the i2c port. See function `setup_i2c`
 * @param The address of the AM2320 device. Should generally be `AM2320_I2C_ADDR`
 * @param temp_reading Pointer to a uint16_t (unsigned short) where the result is stored
 * @return An ESP error code. ESP_OK if reading was successful, something else if not.
*/
esp_err_t am2320_read_temp(i2c_port_t port, uint8_t am2320_addr, uint16_t* temp_reading) {
  esp_err_t err;
  uint8_t read_cmd[] = {AM2320_READ_CMD, AM2320_TEMP_H, 2}; //2 because we're reading two items
  uint8_t rbuf[6]; //function code, data len, data high, data low, crc1, crc2

  //Try waking up the device.
  // May be necessary to request wakeup multiple times, but more than 5x indicates failure
  for(int i=0; i<5; i++) {
    err = wake_am2320(port, am2320_addr);
    if (err == ESP_OK) {
      break;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  if (err != ESP_OK) {
    ESP_OK("I2C", "am2320 did not respond to wake command: %s", esp_err_to_name(err));
    return err;
  }
  //Delay a little to allow the device to come up
  vTaskDelay(AM2320_DELAY_T1 / portTICK_PERIOD_MS);

  //Send command to the device to read out data
  //port, address, command buffer, size of command buffer, timeout
  err = i2c_master_write_to_device(port, am2320_addr, read_cmd, 3, I2C_TIMEOUT_MS);
  if (err != ESP_OK) {
    ESP_LOGI("I2C", "am2320 did not respond to read command: %s", esp_err_to_name(err));
    return err;
  }
  //Delay before getting back data
  vTaskDelay(AM2320_DELAY_T2 / portTICK_PERIOD_MS);

  //port, address, read buffer, size of read buffer, timeout
  err = i2c_master_read_from_device(port, am2320_addr, rbuf, 6, I2C_TIMEOUT_MS);
  if (err != ESP_OK) {
    ESP_LOGI("I2C", "am2320 did not return read data: %s", esp_err_to_name(err));
    return err;
  }

  //Check validity of returned data
  if (rbuf[0] != AM2320_READ_CMD) {
    ESP_LOGI("I2C", "am2320 did not return expected data function (got %d)", rbuf[0]);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (rbuf[1] != 2) {
    ESP_LOGI("I2C", "am2320 did not return expected number of bytes (got %d)", rbuf[1]);
  }
  *temp_reading = (((uint16_t) rbuf[2]) << 8) | ((uint16_t) rbuf[3]);
  return ESP_OK;
}

/**
 * Configure the soil sensor, setting up the required data structures on the ESP32-C3
 * @param sda_pin The GPIO pin on which the SDA signal of the I2C module is connected (white wire)
 * @param scl_pin The GPIO pin on which the SCL signal of the I2C module is connected (green wire)
 *
 * @return An i2c_port_t that must be given to read_soil_sensor to read the value. Returns -1 if something went wrong
*/
i2c_port_t setup_i2c(int sda_pin, int scl_pin) {
  esp_err_t err;

  int i2c_master_port = I2C_MASTER_PORT;
    i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda_pin,
    .scl_io_num = scl_pin,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_FREQ_HZ
  };

  err = i2c_param_config(i2c_master_port, &i2c_conf);
  if (err != ESP_OK) {
    printf("ERROR: Not able to set up i2c soil sensor: %x\n", err);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    return -1;
  }

  err = i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_SLV_BUFSIZE, I2C_SLV_BUFSIZE, 0);
  if (err != ESP_OK) {
    printf("ERROR: i2c driver install not ok: %x\n", err);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    return -1;
  }
  printf("I2C port was set up\n");
  return i2c_master_port;
}

/**
 * Read a value from the soil sensor
 * @param port The i2c_port_t returned from setup_soil_sensor
 * @return An unsigned short value, representing the moisture level of the soil sensor

unsigned short read_soil_sensor(i2c_port_t port) {
  esp_err_t err;
  uint8_t wbuf[2] = {STEMMA_BASE_ADDR, STEMMA_F_REG};
  uint8_t rbuf[STEMMA_RBUF_SIZE];

  err = i2c_master_write_read_device(port, STEMMA_I2C_ADDR, wbuf, 2, rbuf, STEMMA_RBUF_SIZE, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  if (err != ESP_OK) {
    printf("ERROR: Unable to read soil sensor: %x\n", err);
  }
  unsigned short r = ((uint16_t) rbuf[0]) << 8 | ((uint16_t) rbuf[1]);
  return r;
}
*/

/**
 * An example of how to use the soil sensor

void soil_sensor_example(void) {
  i2c_port_t port = setup_i2c(GPIO_NUM_6, GPIO_NUM_7);
  while(true) {
    printf("Read value: %d\n", read_soil_sensor(port));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
*/

/**
 * An example of how to use the temperature sensor
*/
void temp_sensor_example(void) {
  i2c_port_t port = setup_i2c(GPIO_NUM_18, GPIO_NUM_19);
  esp_err_t err;
  uint16_t temp_reading;
  float real_temp;

  while(true) {
    err = am2320_read_temp(port, AM2320_I2C_ADDR, &temp_reading);
    if (err == ESP_OK) {
      real_temp = (float) temp_reading / 10.0;
      printf("temp=%.2f\n", real_temp);
    } else {
      printf("ERR: Unable to read temperature sensor: %x (%s)\n", err, esp_err_to_name(err));
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}
