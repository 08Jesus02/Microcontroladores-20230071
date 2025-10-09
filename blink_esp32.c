#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// === CONFIGURA AQUÍ TU PIN LED ===
// En muchas ESP32-DevKitC el LED está en GPIO2.
// Cambia a tu pin si usas otro (p.ej. GPIO8 en algunos ESP32-C3).
#define LED_GPIO        GPIO_NUM_2

// Pon 1 si tu LED es activo-bajo (se enciende con nivel 0)
#define LED_ACTIVE_LOW  0

static const char *TAG = "BLINK";

void app_main(void)
{
    // Configurar pin como salida
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    bool on = false;

    while (1) {
        on = !on;
        int level = LED_ACTIVE_LOW ? !on : on;
        gpio_set_level(LED_GPIO, level);
        ESP_LOGI(TAG, "LED %s", on ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms
    }
}
