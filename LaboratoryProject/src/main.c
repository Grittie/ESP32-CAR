#include "driver/i2c.h"
#include <stdio.h>
#include <string.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ 100000
#define OLED_I2C_ADDR 0x3C

// Basic 8x8 font for ASCII characters (space to ~)
const uint8_t font8x8_basic[96][8] = {
    // Add your font data here, or use an external library.
    // For simplicity, you can use font arrays from open-source SSD1306 libraries.
};

// Initialize I2C master
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Send a command to the OLED
void oled_send_command(uint8_t cmd) {
    uint8_t data[] = {0x00, cmd}; // Co=0, D/C#=0
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_I2C_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

// Send data (pixel buffer) to OLED
void oled_send_data(uint8_t *data, size_t length) {
    uint8_t *buffer = malloc(length + 1);
    buffer[0] = 0x40; // Co=0, D/C#=1
    memcpy(buffer + 1, data, length);
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_I2C_ADDR, buffer, length + 1, 1000 / portTICK_PERIOD_MS);
    free(buffer);
}

// Initialize OLED display
void oled_init(void) {
    oled_send_command(0xAE); // Display OFF
    oled_send_command(0x20); oled_send_command(0x00); // Horizontal addressing mode
    oled_send_command(0xB0); // Set page start address
    oled_send_command(0xC8); // COM output scan direction
    oled_send_command(0x00); // Lower column address
    oled_send_command(0x10); // Higher column address
    oled_send_command(0x40); // Display start line
    oled_send_command(0x81); oled_send_command(0x7F); // Contrast control
    oled_send_command(0xA1); // Segment remap
    oled_send_command(0xA6); // Normal display
    oled_send_command(0xA8); oled_send_command(0x3F); // Multiplex ratio (1/64)
    oled_send_command(0xA4); // Resume to RAM content
    oled_send_command(0xD3); oled_send_command(0x00); // Display offset
    oled_send_command(0xD5); oled_send_command(0x80); // Clock divide ratio
    oled_send_command(0xD9); oled_send_command(0xF1); // Pre-charge period
    oled_send_command(0xDA); oled_send_command(0x12); // COM pins config
    oled_send_command(0xDB); oled_send_command(0x40); // Vcomh deselect level
    oled_send_command(0x8D); oled_send_command(0x14); // Charge pump enable
    oled_send_command(0xAF); // Display ON
}

// Set cursor position
void oled_set_cursor(uint8_t page, uint8_t column) {
    oled_send_command(0xB0 + page);                // Set page start address
    oled_send_command(0x00 + (column & 0x0F));    // Set lower column start address
    oled_send_command(0x10 + ((column >> 4) & 0x0F)); // Set higher column start address
}

// Write a string to the OLED
void oled_write_string(const char *text) {
    while (*text) {
        uint8_t char_index = *text - ' '; // Offset for ASCII space
        oled_send_data((uint8_t *)font8x8_basic[char_index], 8);
        text++;
    }
}

// Clear the screen
void oled_clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        oled_set_cursor(page, 0);
        uint8_t empty[128] = {0};
        oled_send_data(empty, 128);
    }
}

void app_main(void) {
    i2c_master_init();
    oled_init();

    oled_clear(); // Clear the screen
    oled_set_cursor(0, 0); // Set cursor to top-left
    oled_write_string("Hello, World!"); // Write the message
}
