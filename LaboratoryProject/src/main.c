#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Define the GPIO pin for the onboard LED
#define BLINK_GPIO GPIO_NUM_2  // Replace with the correct pin number for your ESP32-S3 board

void app_main(void) {
    // Configure the GPIO pin as output
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // print in serial
        printf("blink LED\n");
        // Blink the LED
        gpio_set_level(BLINK_GPIO, 1);  // Turn LED on
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay for 500 ms

        printf("blink LED\n");
        gpio_set_level(BLINK_GPIO, 0);  // Turn LED off
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay for 500 ms
    }
}
