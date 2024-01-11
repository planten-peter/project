#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h" // Include the ESP timer header
#include "driver/gptimer.h"   // Include the general-purpose timer header

#define LED_PIN GPIO_NUM_5
#define BUTTON_PIN GPIO_NUM_0

volatile bool timer_expired = false;
//gptimer_handle_t timer;
volatile bool button_pressed = false;

void IRAM_ATTR button_isr_handler(void* arg) {
  button_pressed = true;
}

void IRAM_ATTR timer_isr_handler(void* arg) {
  timer_expired = true;
}


void app_main(void) {
  //Setup LED and set initial level to low (no output)
  // Your code here
  printf("LED set up\n");



  // Your code here
  printf("Timer created\n");

  //Create button handler
  //Setup interrupt on button negative edge
  //Your code here

  // Attach ISR to button
  gpio_install_isr_service(0);
  esp_err_t err = gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

  if(err != ESP_OK) {
    printf("Unable to set up GPIO ISR handler\n");
  } else {
    printf("GPIO ISR handler was set up\n");
    while(true) {
      //ISR's should set flags, while-loop in app_main should respond to flags and revert their state
      if(button_pressed) {
        printf("Button pressed\n");

      }
      if(timer_expired) {
        printf("Timer expired\n");
        //Your code here
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}
