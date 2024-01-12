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
#include "soil_sensor.c"
#include "driver/gptimer.h"


const static char *TAG = "EXAMPLE";

volatile bool timer_expired = false;

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

static int adc_raw = 0; //int created to store data from photosensor
static int led_red_state = 1; //int created to store the state of the red-led (1 off / 0 on)

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
void IRAM_ATTR timer_isr_handler(void* arg) {
    timer_expired = true;
}

static void timer_setup(gptimer_handle_t* timer){
    // Static can call from anywhere
    // Void no input
    // Void 2nd no output
    gptimer_config_t timer_config = {//setup the timer
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // default clock source
        .direction = GPTIMER_COUNT_UP, // count up
        .resolution_hz = 1 * 1000 * 1000, // 1mHz, 1 tick = 1us
    };
    gptimer_alarm_config_t alarm_config = {//choose how long to wait for the alarm
        .alarm_count = 5 * 1000 * 1000ULL, // 5s, use unsigned long long type
    };
    gptimer_event_callbacks_t call = {//choose callback of the timer (on interrupt)
        .on_alarm = timer_isr_handler,
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, timer));//check for error when making timer
    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, &alarm_config));//check for error when setting alarm
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, &call,NULL));//check for error when registering callback
    ESP_ERROR_CHECK(gptimer_enable(*timer));//check for error when registering callback  
    ESP_LOGI("","Timer created\n");
}

static void ADC_setup(adc_oneshot_unit_handle_t* adc1_handle){
    //-------------ADC1 Init---------------// Analog Digital Converter
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,//PIN 0 on the on ESP32
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc1_handle));
    
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc1_handle, EXAMPLE_ADC1_CHAN1, &config));
}

void app_main(void){
    i2c_port_t port = setup_soil_sensor(GPIO_NUM_18, GPIO_NUM_19);
    
    gptimer_handle_t timer = NULL;
    adc_oneshot_unit_handle_t adc1_handle = NULL;
    init();
    timer_setup(&timer);
    ADC_setup(&adc1_handle);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_start(timer));
    while (1) {
        //-------------ADC1 Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw));
        // ESP_LOGI(TAG, "ADC2 CH1 Raw: %d", adc_raw);
        bool allGood = true;
        if(adc_raw < 2000){
            timer_expired = false;
            gptimer_stop(timer);          
            gptimer_set_raw_count(timer,0);
            gptimer_start(timer);
        }
        if(timer_expired){
            allGood = false;
            ESP_LOGI("Light-condition" , "Too dark");
        }else{
            ESP_LOGI("Light-condition", "All good");
        }
        if(read_soil_sensor(port) < 600){
            allGood = false;
            ESP_LOGI("Soil-condition","too Dry");
        }else{
            ESP_LOGI("Soil-condition","all good");
        }
        if(!allGood) {
            led_red_state = !led_red_state;
            setPin(LED_RED,led_red_state);
            setPin(LED_GREEN,1);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","RED");
        }else{
            setPin(LED_RED,1);
            setPin(LED_GREEN,0);
            setPin(LED_BLUE,1);
            ESP_LOGI("Color","GREEN");
        }
        vTaskDelay((timer_expired ? 100 : 1000) / portTICK_PERIOD_MS); //delaying the while loop. If timer_expired = true, 
                                                                     //we are in red alert, and the while loop will run faster. If timer_expired false, 
                                                                     //less frequently
    }
}