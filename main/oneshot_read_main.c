/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_0 //GPIO 0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11 //0-2.5v

#define LED_RED GPIO_NUM_5
#define LED_BLUE GPIO_NUM_6
#define LED_GREEN GPIO_NUM_7
#define setPin(pin, state) gpio_set_level(pin, state)

static uint8_t s_led_state = 0b1;
static int adc_raw = 0; //int created to store data from photosensor

static void configure_OUTPUT_pin(gpio_num_t pin, int state){
    ESP_LOGI("Pin", "setting pin:%d to OUT", pin);
    gpio_reset_pin(pin);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, state);
}

static void init(void){
    //set LED pins to  output, this allows writing to the pin
    configure_OUTPUT_pin(LED_BLUE,0);
    configure_OUTPUT_pin(LED_RED,1);
    configure_OUTPUT_pin(LED_GREEN,1);
}

void app_main(void)
{
    init();
    //-------------ADC1 Init---------------// Analog Digital Converter
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,//PIN 0 on the on ESP32
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    setPin(LED_RED,1);
    setPin(LED_GREEN,1);
    setPin(LED_BLUE,1);
    while (1) {
        //-------------ADC1 Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw));
        ESP_LOGI(TAG, "ADC2 CH1 Raw: %d", adc_raw);
            if(adc_raw < 2000 ){
                  setPin(LED_RED,1);
            setPin(LED_GREEN,0);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","GREEN");
            s_led_state= s_led_state << 1;
          
            }
            else{
                 setPin(LED_RED,0);
            setPin(LED_GREEN,1);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","RED");
            s_led_state= s_led_state << 1;

            }

      /*  switch (s_led_state){
        case 0b001:
            setPin(LED_RED,0);
            setPin(LED_GREEN,1);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","RED");
            s_led_state= s_led_state << 1;
            break;
        case 0b010:
            setPin(LED_RED,1);
            setPin(LED_GREEN,0);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","GREEN");
            s_led_state= s_led_state << 1;
            break;
        case 0b100:
            setPin(LED_RED,1);
            setPin(LED_GREEN,1);
            setPin(LED_BLUE,0);
            ESP_LOGI("Color","BLUE");
            s_led_state= s_led_state << 1;
            break;
        default:
            s_led_state = 1;
            setPin(LED_RED,1);
            setPin(LED_GREEN,1);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","OFF");
        }*/
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}