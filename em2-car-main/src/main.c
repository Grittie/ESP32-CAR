#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

// Pin Definitions

// ADC Definitions
#define LDR_ADC_CHANNEL ADC_CHANNEL_1
#define LDR_ADC_UNIT ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_9

// PWM Definitions
#define DIP_SWITCH_1 GPIO_NUM_39
#define DIP_SWITCH_2 GPIO_NUM_40
#define DIP_SWITCH_3 GPIO_NUM_41
#define DIP_SWITCH_9 GPIO_NUM_38
#define DIP_SWITCH_10 GPIO_NUM_20

// I2C Definitions
#define I2C_DISPLAY_NUM I2C_NUM_1
#define I2C_DISPLAY_FREQ_HZ 100000
#define I2C_DISPLAY_SDA_IO GPIO_NUM_8
#define I2C_DISPLAY_SCL_IO GPIO_NUM_9 

// OLED Display Definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// LEDC Definitions
typedef enum SCREEN_ALIGNMENT{
  FREE,
  CENTER,
  RIGHT,
  LEFT
} SCREEN_ALIGNMENT;

// OLED Display Handle
static ssd1306_handle_t ssd1306_dev = NULL;

// LED Definitions
#define LED_1 GPIO_NUM_19
#define LED_2 GPIO_NUM_47
#define LED_3 GPIO_NUM_35
#define LED_4 GPIO_NUM_37

// Motor Definitions
#define MOTOR_IN1 GPIO_NUM_5
#define MOTOR_IN2 GPIO_NUM_6
#define MOTOR_EN GPIO_NUM_4

// Button Definitions
#define BREAK_BTN GPIO_NUM_15
#define HORN_BTN GPIO_NUM_16

// Buzzer Definitions
#define BZR GPIO_NUM_17

// LDR Threshhold Value
#define THRESHOLD_VALUE 100

// Motor Speed Definitions
#define REVERSE_SPEED 800

// Hall Effect Sensor Definitions
#define HALL_SENSOR_PIN GPIO_NUM_7
#define PULSE_COUNT_PERIOD_MS 1000

// Button Debounce Delay
#define DEBOUNCE_DELAY_MS 150

// Global Variables

// ADC Handle
static adc_oneshot_unit_handle_t adc_handle;

// ISR Variables
volatile bool brake_pressed = false;  
volatile bool horn_pressed = false;

// Hall Sensor Variables
volatile int pulse_count = 0;
volatile bool is_motor_active = false;

// Global variables for debouncing
volatile uint32_t last_brake_press_time = 0;
volatile uint32_t last_horn_press_time = 0;

// RIP global variable
volatile double current_rpm = 0.0; 

// Function Prototypes
bool key_check(void);
int get_dip_switch_state(void);
void configure_gpio(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down);
void configure_pwm(void);
int get_motor_speed(void);
void indicator_light_task(void *param);
static void IRAM_ATTR brake_button_isr_handler(void* arg);
void configure_button_with_interrupt(gpio_num_t pin);
static void IRAM_ATTR horn_button_isr_handler(void* arg);
void configure_horn_button_with_interrupt(void);
void configure_buzzer(void);
void handle_brake(void);
void handle_horn(void);
static void IRAM_ATTR hall_sensor_isr_handler(void* arg);
void calculate_rpm_task(void* param);
void configure_hall_sensor(void);
void i2c_master_init(void);
void init_display(void);
void add_text_to_display(const char *text, int x, int y, SCREEN_ALIGNMENT alignment, bool clear);
void clear_display(void);

void app_main(void)
{
    // Configure the button GPIO with interrupt
    configure_button_with_interrupt(BREAK_BTN);

    // Configure the horn button and buzzer
    configure_horn_button_with_interrupt();
    configure_buzzer();

    // Configure GPIOs for DIP switches, LEDs, and motor control
    configure_gpio(DIP_SWITCH_1, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_2, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_3, GPIO_MODE_INPUT, true, false);

    configure_gpio(LED_1, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(LED_2, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(MOTOR_IN1, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(MOTOR_IN2, GPIO_MODE_OUTPUT, false, false);

    configure_gpio(DIP_SWITCH_9, GPIO_MODE_INPUT, true, false);
    configure_gpio(DIP_SWITCH_10, GPIO_MODE_INPUT, true, false);
    configure_gpio(LED_3, GPIO_MODE_OUTPUT, false, false);
    configure_gpio(LED_4, GPIO_MODE_OUTPUT, false, false);

    // Create a task to control the indicator lights
    xTaskCreate(indicator_light_task, "Indicator Light Task", 2048, NULL, 1, NULL);


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

    // Configure Hall Effect Sensor and create task to calculate RPM
    configure_hall_sensor();
    xTaskCreate(calculate_rpm_task, "Calculate RPM Task", 2048, NULL, 1, NULL);

    // Initialize I2C and OLED display
    init_display();

    // Main loop to check the key presence and DIP switch states
    while (1) {

        if (brake_pressed) {
            handle_brake();
            is_motor_active = false;
        }

        if (horn_pressed) {
            handle_horn();
        }

        if (key_check()) {
            printf("Key detected. Checking DIP switch state...\n");

            int current_state = get_dip_switch_state();
            if (current_state != last_state) {
                last_state = current_state;
                printf("DIP switch selected: %d\n", current_state);

                if (current_state == 1) {
                    printf("Car is in park\n");
                    add_text_to_display("Car is parked", 0, 0, CENTER, true);

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
                    add_text_to_display("Car is driving", 0, 0, CENTER, true);
                    is_motor_active = true;

                    // Set motor direction (forwards) and speed based on DIP switch 2
                    gpio_set_level(MOTOR_IN1, 1);
                    gpio_set_level(MOTOR_IN2, 0);
                    
                    // Dynamic motor speed control based on potentiometer value
                    while (current_state == 2) {
                        current_state = get_dip_switch_state();

                        
                        brake_pressed = false;
                        horn_pressed = false;

                        if (brake_pressed) {
                            handle_brake();
                            is_motor_active = false;
                            break;
                        }

                        if (horn_pressed) {
                            handle_horn();
                        }

                        int speed = get_motor_speed();
                        printf("Dynamic motor speed: %d\n", speed);

                        // Display RPM on the OLED
                        char rpm_text[32];
                        snprintf(rpm_text, sizeof(rpm_text), "RPM: %.2f", current_rpm);
                        add_text_to_display(rpm_text, 0, 16, LEFT, false);

                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
                        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                } else if (current_state == 3) {
                    printf("Car is in reverse\n");
                    add_text_to_display("Car is driving in reverse", 0, 0, CENTER, true);

                    is_motor_active = true;

                    // Set motor direction (backwards) and speed based on DIP switch 3
                    gpio_set_level(MOTOR_IN1, 0);
                    gpio_set_level(MOTOR_IN2, 1);

                    // Set motor speed to a predefined value
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, REVERSE_SPEED);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                } else {
                    printf("Car is off\n");
                    add_text_to_display("Car is off", 0, 0, CENTER, true);
                    is_motor_active = false;

                    // Turn off motor and LEDs when car is off
                    gpio_set_level(MOTOR_IN1, 0);
                    gpio_set_level(MOTOR_IN2, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                }
            }
        } else {
            printf("No key detected. Waiting...\n");
            add_text_to_display("No key detected", 0, 0, CENTER, true);
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

/**
 * @brief Task to control the indicator lights based on the DIP switch states.
 *
 * This function checks the states of DIP_SWITCH_9 and DIP_SWITCH_10 and turns on the corresponding LEDs.
 */
void indicator_light_task(void *param) {
    while (1) {
        // Check DIP_SWITCH_10 and key presence
        if (gpio_get_level(DIP_SWITCH_9) == 0 && key_check()) {
            // Start blinking LED_3 until DIP_SWITCH_9 is turned off
            add_text_to_display("Indicating left", 0, 0, CENTER, true);
            while (gpio_get_level(DIP_SWITCH_9) == 0 && key_check()) {
                gpio_set_level(LED_3, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_3, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else {
            gpio_set_level(LED_3, 0);
        }

        // Check DIP_SWITCH_10 and key presence
        if (gpio_get_level(DIP_SWITCH_10) == 0  && key_check()) {
            add_text_to_display("Indicating right", 0, 0, CENTER, true);
            // Start blinking LED_4 until DIP_SWITCH_10 is turned off
            while (gpio_get_level(DIP_SWITCH_10) == 0 && key_check()) {
                gpio_set_level(LED_4, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_4, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else {
            gpio_set_level(LED_4, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Interrupt handler for the brake button.
 *
 * This function is called when the brake button is pressed.
 *
 * @param arg The argument passed to the ISR handler.
 */
static void IRAM_ATTR brake_button_isr_handler(void* arg) {
    uint32_t current_time = esp_timer_get_time() / 1000;  // Get time in milliseconds
    if (current_time - last_brake_press_time > DEBOUNCE_DELAY_MS) {
        brake_pressed = true;  // Set the flag when the button is pressed
        last_brake_press_time = current_time;  // Update last press time
    }
}

/**
 * @brief Configure a button with an interrupt handler.
 *
 * This function configures a button with an interrupt handler that triggers on a falling edge.
 *
 * @param pin The GPIO pin number to configure with an interrupt.
 */
void configure_button_with_interrupt(gpio_num_t pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),       // Configure the specific pin
        .mode = GPIO_MODE_INPUT,            // Set as input
        .pull_up_en = GPIO_PULLUP_ENABLE,   // Enable pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE      // Trigger on falling edge
    };
    gpio_config(&io_conf);

    // Install the ISR service and attach the handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin, brake_button_isr_handler, NULL);
}

static void IRAM_ATTR horn_button_isr_handler(void* arg) {
    uint32_t current_time = esp_timer_get_time() / 1000;  // Get time in milliseconds
    if (current_time - last_horn_press_time > DEBOUNCE_DELAY_MS) {
        horn_pressed = true;  // Set the flag
        last_horn_press_time = current_time;  // Update last press time
    }
}

/**
 * @brief Configure the horn button with an interrupt handler.
 *
 * This function configures the horn button with an interrupt handler that triggers on a falling edge.
 */
void configure_horn_button_with_interrupt(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HORN_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on button press (falling edge)
    };
    gpio_config(&io_conf);

    // Install ISR service and attach handler
    gpio_install_isr_service(0);  // Use default interrupt flags
    gpio_isr_handler_add(HORN_BTN, horn_button_isr_handler, NULL);
}

/**
 * @brief Configure the buzzer GPIO pin.
 *
 * This function configures the buzzer GPIO pin as an output with a pull-up resistor.
 */
void configure_buzzer(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BZR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * @brief Handle the brake button press event.
 *
 * This function is called when the brake button is pressed.
 */
void handle_brake() {
    printf("Brake pressed! Stopping the car.\n");
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    brake_pressed = false;  // Reset the flag
}

/**
 * @brief Handle the horn button press event.
 *
 * This function is called when the horn button is pressed.
 */
void handle_horn() {
    printf("Horn pressed! Beeping.\n");
    gpio_set_level(BZR, 1);  // Turn on the buzzer
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second
    gpio_set_level(BZR, 0);  // Turn off the buzzer
    horn_pressed = false;  // Reset the flag
}

/**
 * @brief Interrupt handler for the Hall Effect Sensor.
 *
 * This function is called when the Hall Effect Sensor detects a pulse.
 *
 * @param arg The argument passed to the ISR handler.
 */
static void IRAM_ATTR hall_sensor_isr_handler(void* arg) {
    pulse_count++;
}

/**
 * @brief Task to calculate the motor speed using the Hall Effect Sensor.
 *
 * This function calculates the motor speed in RPM using the Hall Effect Sensor.
 */
void calculate_rpm_task(void* param) {
    while (1) {
        if (is_motor_active) {
            int pulses;
            uint32_t start_time = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(PULSE_COUNT_PERIOD_MS));
            uint32_t end_time = esp_timer_get_time();

            pulses = pulse_count;
            pulse_count = 0;

            double time_seconds = (end_time - start_time) / 1e6;  // Convert time to seconds
            double rpm = (pulses / time_seconds) * 60;           // Calculate RPM
            current_rpm = rpm;  // Update global variable
            printf("Motor Speed: %.2f RPM\n", rpm);
        } else {
            current_rpm = 0.0;  // Set RPM to 0 if motor is inactive
            vTaskDelay(pdMS_TO_TICKS(500)); // Sleep to save CPU cycles if the motor is inactive
        }
    }
}

/**
 * @brief Configure the Hall Effect Sensor GPIO pin.
 *
 * This function configures the Hall Effect Sensor GPIO pin as an input with a pull-up resistor.
 */
void configure_hall_sensor() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(HALL_SENSOR_PIN, hall_sensor_isr_handler, NULL);
}

/**
 * @brief Initialize the I2C master interface.
 *
 * This function initializes the I2C master interface for the OLED display.
 */
void init_display()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_DISPLAY_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_DISPLAY_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_DISPLAY_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_DISPLAY_NUM, &conf);
    i2c_driver_install(I2C_DISPLAY_NUM, conf.mode, 0, 0, 0);

    // Initialize the SSD1306 display
    ssd1306_dev = ssd1306_create(I2C_DISPLAY_NUM, SSD1306_I2C_ADDRESS);
    if (ssd1306_dev == NULL) {
        printf("Failed to initialize SSD1306\n");
        return;
    }

    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    // Add startup text to the display
    add_text_to_display("Startup...", 0, 0, CENTER, true);
}

/**
 * @brief Add text to the OLED display.
 *
 * This function adds text to the OLED display at the specified position and alignment.
 *
 * @param text The text to display.
 * @param x The x-coordinate of the text.
 * @param y The y-coordinate of the text.
 * @param alignment The alignment of the text (FREE, CENTER, RIGHT, LEFT).
 * @param clear Clear the screen before adding the text.
 */
void add_text_to_display(const char *text, int x, int y, SCREEN_ALIGNMENT alignment, bool clear)
{
    switch (alignment)
    {
    default:
    case FREE:
        break;
    case CENTER:
        // Calculate the x pos based on the length of the string and the width of chars
        x = (SCREEN_WIDTH - strlen(text) * 8) / 2;
        break;
    case LEFT:
        x = 0;
        break;
    case RIGHT:
        x = (SCREEN_WIDTH - strlen(text) * 8);
        break;
    }

    if (clear) {
        clear_display();
    }

    ssd1306_draw_string(ssd1306_dev, x, y, (const uint8_t *)text, 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
}

/**
 * @brief Clear the OLED display.
 *
 * This function clears the OLED display by filling the screen with zeros.
 */
void clear_display()
{
    ssd1306_clear_screen(ssd1306_dev, 0x00);
    ssd1306_refresh_gram(ssd1306_dev);
}