/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Höf.:  S. Maggi Snorrason
 *  Netf.:  sms70@hi.is
 *  Dags.:  8. nóvember 2023
 *
 *     Lýsing:  Sækir gögn í BME280 skynjara í kjarna 1
 *              Raspberry Pi Pico 2040 örtölvu og sendir gögnin
 *              yfir á kjarna 0 með biðröð (e. queue). Kjarni 0
 *              prentar svo gildin út.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <hardware/i2c.h>
#include <BME280.h>

#define HUMI_DIFF 0.5 // Typical RMS :    0.02-0.07  %RH
#define TEMP_DIFF 0.1 //    noise at :  0.002-0.005  °C
#define PRES_DIFF 1.5 //        25°C :      0.2-1.3  Pa

const uint8_t PIN_QT_SDA = 2;
const uint8_t PIN_QT_SCL = 3;

BME280 bme;
queue_t bme_queue;

BME280::BME_Measurement_t bme_measurement_bme280;
BME280::BME_Measurement_t bme_measurement_core1;
BME280::BME_Measurement_t bme_measurement_core0;

/**
 * Athugar hvort nýjasta mæligildið frá skynjaranum er mikið
 * öðruvísi en síðasta gildið sem búið var að senda á kjarna 0.
 * 
 * @return hvort gildið sé búið að breytast mikið
 */
bool has_changed() {
  return (
    abs(bme_measurement_bme280.humidity 
      - bme_measurement_core1.humidity) > HUMI_DIFF ||
    abs(bme_measurement_bme280.temperature 
      - bme_measurement_core1.temperature) > TEMP_DIFF ||
    abs(bme_measurement_bme280.pressure 
      - bme_measurement_core1.pressure) > PRES_DIFF
  );
}

void core_1_entry() {
  i2c_init(i2c1, 400000);
  gpio_set_function(PIN_QT_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_QT_SCL, GPIO_FUNC_I2C);
  bme = BME280(i2c1, BME280_DEFAULT_I2CADDR);
  sleep_ms(1000);
  if(!bme.init()) {
    while(true);
  }
  while (1) {
    if (bme.read(&bme_measurement_bme280)) {
      if (has_changed()) {
        bme_measurement_core1 = bme_measurement_bme280;
        queue_add_blocking(
          &bme_queue, 
          &bme_measurement_core1
        );
      }
    }
    sleep_ms(10);
  }
}

void init() {
  stdio_init_all();
  sleep_ms(5000);
  queue_init(
    &bme_queue, 
    sizeof(BME280::BME_Measurement_t), 
    2
  );
  printf("Launching core 1... ");
  multicore_launch_core1(core_1_entry);
  printf("Launched.\n");
}

void loop() {
  queue_remove_blocking(
    &bme_queue, 
    &bme_measurement_core0
  );
  printf(
    "        Hitastig: %.2f °C\n", 
    bme_measurement_core0.temperature
  );
  printf(
    "  Loftþrýstingur: %.2f hPa\n", 
    bme_measurement_core0.pressure
  );
  printf(
    "        Rakastig: %.2f%%\n\n", 
    bme_measurement_core0.humidity
  );
  sleep_ms(10);
}

int main() {
  init();
  while(1) {
    loop();
  }
  return 0;
}
