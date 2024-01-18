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
#include "i2c_oled_example_main.c"
#include "lvgl_demo_ui.c"

// const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_0 //GPIO 0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11 //0-2.5v

#define LED_RED GPIO_NUM_7
#define LED_BLUE GPIO_NUM_6
#define LED_GREEN GPIO_NUM_5
#define IC2_SDA GPIO_NUM_18
#define IC2_SCL GPIO_NUM_19
#define setPin(pin, state) gpio_set_level(pin, state)

#define minimumLight 3500
#define timerInSeconds 5

static uint8_t s_led_state = 0b1;
static int adc_raw = 0; //int created to store data from photosensor

static void configure_OUTPUT_pin(gpio_num_t pin, int state){
    ESP_LOGI("Pin", "setting pin%d to OUT", pin);
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

bool IRAM_ATTR timer_isr_handler(struct gptimer_t *, const gptimer_alarm_event_data_t *, void * arg) {
    timer_expired = true;
    return true;
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
        .alarm_count = timerInSeconds * 1000 * 1000ULL, // use unsigned long long type
    };
    gptimer_event_callbacks_t call = {//choose callback of the timer (on interrupt)
        .on_alarm =  timer_isr_handler,
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, timer));//check for error when making timer
    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, &alarm_config));//check for error when setting alarm
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, &call,NULL));//check for error when registering callback
    ESP_ERROR_CHECK(gptimer_enable(*timer));//check for error when registering callback  
    ESP_LOGI("","Timer created\n");
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

static void green_LED(){
    setPin(LED_RED,1);
    setPin(LED_GREEN,0);
    setPin(LED_BLUE,1);
}
static void red_LED(){
    setPin(LED_RED,led_red_state);
    setPin(LED_GREEN,1);
    setPin(LED_BLUE,1);
}

void app_main(void){
    init(); // setup LED pins
    i2c_port_t port = I2C_setup(IC2_SDA, IC2_SCL);// setup I2C to the soil sensor and screen (via the relevand GPIO ports)
    gptimer_handle_t timer = NULL; // timer handle
    adc_oneshot_unit_handle_t adc1_handle = NULL; // adc handle
    timer_setup(&timer); // setup timer
    ADC_setup(&adc1_handle); // setup ADC
    lv_disp_t* disp = generateDisp(); // setup screen
    lv_obj_t* old = NULL; //the text displayed on the screen
    vTaskDelay(3000 / portTICK_PERIOD_MS); // delay for 3 seconds
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_start(timer)); // start timer
    while (1) { // Nu sker der ting og sager
        //-------------ADC1 Read---------------//
        adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw);
        int soil = read_soil_sensor(port);
        uint8_t problems = (adc_raw < minimumLight) | (timer_expired << 1) | ((soil < 700) << 2); // contains all errors
        ESP_LOGI("Result", "0b%d%d%d", (problems&0b100) == 0b100,(problems&0b010) == 0b010,(problems&0b001) == 0b001);

        if(adc_raw > minimumLight){
            timer_expired = false;
            gptimer_stop(timer);          
            gptimer_set_raw_count(timer,0);
            gptimer_start(timer);
            // ESP_LOGI("light","%d", adc_raw);
        }

        switch (problems){
        case 0b000:
            green_LED();
            vTaskDelay(100/portTICK_PERIOD_MS);
            old = example_lvgl_demo_ui(disp,"All good",old);
            break;
        case 0b001:
            old = example_lvgl_demo_ui(disp, "too dark", old);
            green_LED();
            break;
        case 0b011:
            old = example_lvgl_demo_ui(disp, "too dark", old);
            red_LED();
            break;
        case 0b100:
            old = example_lvgl_demo_ui(disp, "too dry", old);
            red_LED();
            break;
        case 0b101:
        case 0b111:
            old = example_lvgl_demo_ui(disp, "too dark\ntoo dry", old);
            red_LED();
            break;
        default:
            ESP_LOGI("ERROR", "Something went wrong");
        }
        led_red_state = !led_red_state;
        vTaskDelay(100 / portTICK_PERIOD_MS); //delaying the while loop.

    }
}