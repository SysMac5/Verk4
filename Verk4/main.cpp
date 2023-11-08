/*
 * TODO:
    - Decide how to send data from core 1 to core 0 (see helpful tips below)
 *  - Complete the functions core_1_entry and loop
 * 
 * Helpful tips:
 *  - https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#pico_multicore
 *  - https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#multicore_fifo
 *  - https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_irq
 *  - https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#queue
 */

// Includes
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/i2c.h>
#include <BME280.h>

// Defines

// Pinouts
const uint8_t PIN_QT_SDA = 2;
const uint8_t PIN_QT_SCL = 3;

// Constructors
BME280 bme;

// Global variables and data structures
BME280::BME_Measurement_t bme_measurement_in;

// Forward declarations

void core_1_entry() {
    i2c_init(i2c1, 400000);
    gpio_set_function(PIN_QT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_QT_SCL, GPIO_FUNC_I2C);
    bme = BME280(i2c1, BME280_DEFAULT_I2CADDR); // Change to BME280_ALTERNATE_I2CADDR for 0x77
    sleep_ms(1000);
    if(!bme.init()) {
        //printf("Failed to initialize!\n"); // Uncomment for debugging
        while(true);
    } else {
        //printf("Initialized.\n"); // Uncomment for debugging
    }
    while (1) {
        if (bme.read(&bme_measurement_in)) {
            // Push the measurement to core 0 for displaying
            /*ADD YOUR CODE HERE*/
        }
        sleep_ms(10);
    }
}

void init() {
    stdio_init_all();
    sleep_ms(5000);
    printf("Launching core 1... ");
    multicore_launch_core1(core_1_entry);
    printf("Launched.\n");
}

void loop() {
    // Receive data from core 1 and display
    /*ADD YOUR CODE HERE*/
    sleep_ms(10);
}

int main() {
    init();
    while(1) {
        loop();
    }
    return 0;
}