#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define LDR_ADC_CHANNEL ADC_CHANNEL_1 // GPIO2 -> ADC1_CHANNEL_1
#define LDR_ADC_UNIT ADC_UNIT_1
#define THRESHOLD_VALUE 5

#define DIP_SWITCH_1 GPIO_NUM_39
#define DIP_SWITCH_2 GPIO_NUM_40
#define DIP_SWITCH_3 GPIO_NUM_41

#define LED_1 GPIO_NUM_19
#define LED_2 GPIO_NUM_21

// Function prototypes
bool key_check(void *arg);
int get_dip_switch_state(void);
void configure_gpio(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down);

void app_main(void)
{
    // Configure DIP switch GPIOs
    configure_gpio(DIP_SWITCH_1, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_2, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_3, GPIO_MODE_INPUT, true, false);

    // Configure LED GPIOs
    configure_gpio(LED_1, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(LED_2, GPIO_MODE_OUTPUT, false, false);

    int last_state = -1;

    while (1) {
        if (key_check(NULL)) {
            printf("Key detected. Checking DIP switch state...\n");

            int current_state = get_dip_switch_state();
            if (current_state != last_state) {
                last_state = current_state;
                printf("DIP switch selected: %d\n", current_state);

                if (current_state == 1) {
                    printf("Car is in park\n");
                    // Turn on both leds 10 times rapidly
                    for (int i = 0; i < 10; i++) {
                        gpio_set_level(LED_1, 1);
                        gpio_set_level(LED_2, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(LED_1, 0);
                        gpio_set_level(LED_2, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                } else if (current_state == 2) {
                    printf("Car is in drive\n");
                    printf("Motor rotates clockwise brrrrr");
                } else if (current_state == 3) {
                    printf("Car is in reverse\n");
                    printf('Motor rotates counter-clockwise brrrrr');
                } else {
                    printf("Car is off\n");
                }
            }
        } else {
            printf("No key detected. Waiting...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Main task polling interval
    }
}

/**
 * @brief Configures a GPIO pin with the specified settings.
 *
 * This function sets up a GPIO pin with the desired mode, pull-up, and pull-down configurations.
 *
 * @param pin The GPIO pin number to configure (e.g., GPIO_NUM_19).
 * @param mode The mode of the GPIO pin (e.g., GPIO_MODE_INPUT, GPIO_MODE_OUTPUT).
 * @param pull_up Enable or disable the internal pull-up resistor (true to enable, false to disable).
 * @param pull_down Enable or disable the internal pull-down resistor (true to enable, false to disable).
 */
void configure_gpio(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = mode,
        .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * @brief Task to check the presence of a key (finger detection) using an LDR sensor.
 *
 * This function initializes the ADC in oneshot mode, reads values from the LDR,
 * and checks if the ADC value is below a predefined threshold. If the value
 * is below the threshold, it indicates the presence of a key (finger detected).
 *
 * @param arg Optional parameter for task arguments (not used).
 * @return true if the key is present, false otherwise.
 */
bool key_check(void *arg)
{
    // Initialize ADC in oneshot mode
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LDR_ADC_UNIT,
    };

    // Configure ADC channel
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &channel_config);

    // Read raw ADC value from LDR
    int raw_adc_value;
    adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw_adc_value);

    // Deinitialize ADC
    adc_oneshot_del_unit(adc_handle);

    // Return true if the ADC value is below the threshold
    return raw_adc_value < THRESHOLD_VALUE;
}

/**
 * @brief Get the current state of the DIP switches.
 *
 * Reads the GPIO states of the DIP switches (1-3) and returns the selected state.
 *
 * @return int The current DIP switch state (1, 2, 3, or 0 if none are selected).
 */
int get_dip_switch_state(void)
{
    if (gpio_get_level(DIP_SWITCH_1) == 0) {
        return 1;
    } else if (gpio_get_level(DIP_SWITCH_2) == 0) {
        return 2;
    } else if (gpio_get_level(DIP_SWITCH_3) == 0) {
        return 3;
    }
    return 0; // No DIP switch selected
}
