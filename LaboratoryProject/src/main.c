#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

// Pin Definitions
#define LDR_ADC_CHANNEL ADC_CHANNEL_1 // GPIO2 -> ADC1_CHANNEL_1
#define LDR_ADC_UNIT ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_8 // Adjust to your potentiometer pin

#define DIP_SWITCH_1 GPIO_NUM_39
#define DIP_SWITCH_2 GPIO_NUM_40
#define DIP_SWITCH_3 GPIO_NUM_41

#define LED_1 GPIO_NUM_19
#define LED_2 GPIO_NUM_47

#define MOTOR_IN1 GPIO_NUM_5
#define MOTOR_IN2 GPIO_NUM_6
#define MOTOR_EN GPIO_NUM_4

#define THRESHOLD_VALUE 600

#define REVERSE_SPEED 800

// Global Variables
static adc_oneshot_unit_handle_t adc_handle;

// Function Prototypes
bool key_check(void);
int get_dip_switch_state(void);
void configure_gpio(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down);
void configure_pwm(void);
int get_motor_speed(void);

void app_main(void)
{
    // Configure GPIOs for DIP switches, LEDs, and motor control
    configure_gpio(DIP_SWITCH_1, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_2, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_3, GPIO_MODE_INPUT, true, false);

    configure_gpio(LED_1, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(LED_2, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(MOTOR_IN1, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(MOTOR_IN2, GPIO_MODE_OUTPUT, false, false);

    // Initialize PWM for motor speed control
    configure_pwm();

    // Initialize ADC for potentiometer and LDR
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LDR_ADC_UNIT,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t pot_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &pot_config);

    adc_oneshot_chan_cfg_t ldr_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &ldr_config);

    int last_state = -1;

    while (1) {
        if (key_check()) {
            printf("Key detected. Checking DIP switch state...\n");

            int current_state = get_dip_switch_state();
            if (current_state != last_state) {
                last_state = current_state;
                printf("DIP switch selected: %d\n", current_state);

                if (current_state == 1) {
                    printf("Car is in park\n");
                    // Blink both LEDs for 10 cycles rapidly
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

                    // Set motor direction (forwards) and speed based on DIP switch 2
                    gpio_set_level(MOTOR_IN1, 1);
                    gpio_set_level(MOTOR_IN2, 0);
                    
                    // Dynamic motor speed control based on potentiometer value
                    while (current_state == 2) {
                        current_state = get_dip_switch_state();
                        int speed = get_motor_speed();
                        printf("Dynamic motor speed: %d\n", speed);
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
                        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                } else if (current_state == 3) {
                    printf("Car is in reverse\n");

                    // Set motor direction (backwards) and speed based on DIP switch 3
                    gpio_set_level(MOTOR_IN1, 0);
                    gpio_set_level(MOTOR_IN2, 1);

                    // Set motor speed to a predefined value
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, REVERSE_SPEED);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                } else {
                    printf("Car is off\n");

                    // Turn off motor and LEDs when car is off
                    gpio_set_level(MOTOR_IN1, 0);
                    gpio_set_level(MOTOR_IN2, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                }
            }
        } else {
            printf("No key detected. Waiting...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
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
 * @brief Configures the PWM settings for the motor.
 *
 * This function initializes the PWM timer and channel for the motor control.
 */
void configure_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = MOTOR_EN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Task to check the presence of a key (finger detection) using an LDR sensor.
 *
 * This function reads values from the LDR using ADC and checks if the ADC value is below a predefined threshold.
 * If the value is below the threshold, it indicates the presence of a key (finger detected).
 *
 * @return true if the key is present, false otherwise.
 */
bool key_check(void)
{
    int raw_adc_value;
    adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw_adc_value);
    printf("LDR ADC value: %d\n", raw_adc_value);
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
    return 0;
}

/**
 * @brief Get the current speed of the motor.
 *
 * Reads the raw ADC value from the potentiometer and maps it to a 10-bit duty cycle.
 *
 * @return int The current speed of the motor (0-1023).
 */
int get_motor_speed(void)
{
    int raw_value = 0;
    adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw_value);
    return (raw_value * 1023) / 4095; // Map ADC value to 10-bit duty cycle
}
