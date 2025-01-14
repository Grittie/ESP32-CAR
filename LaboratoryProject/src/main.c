#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

#define LDR_ADC_CHANNEL ADC_CHANNEL_1 // GPIO2 -> ADC1_CHANNEL_1
#define LDR_ADC_UNIT ADC_UNIT_1
#define THRESHOLD_VALUE 10

// Function prototypes
bool key_check(void *arg);

void app_main(void)
{
    if (key_check(NULL)) {
        printf("Car can start\n");
    } else {
        printf("Car cannot start\n");
    }
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
