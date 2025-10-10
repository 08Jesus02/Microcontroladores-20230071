#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// === AJUSTA AQUÍ EL PIN DEL LED ===
// En muchas placas ESP32-DevKitC el LED integrado usa el GPIO2.
// Cámbialo si tu placa utiliza otro (ej.: GPIO8 en algunos ESP32-C3).
#define LED_PIN              GPIO_NUM_2

// Coloca 1 si tu LED es activo en bajo (se enciende con nivel lógico 0)
#define LED_IS_ACTIVE_LOW    0

static const char *TAG = "BLINK_PARAPHRASED";

void app_main(void)
{
    // Configuración básica del GPIO como salida
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);

    bool led_state = false;

    while (true) {
        // Conmutar estado lógico del LED
        led_state = !led_state;

        // Ajustar el nivel según sea activo-bajo o activo-alto
        int out_level = LED_IS_ACTIVE_LOW ? !led_state : led_state;
        gpio_set_level(LED_PIN, out_level);

        ESP_LOGI(TAG, "LED %s", led_state ? "ENCENDIDO" : "APAGADO");

        // Pausa de 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
