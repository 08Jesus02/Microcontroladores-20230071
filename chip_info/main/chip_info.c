#include <stdio.h>
#include "esp_chip_info.h"
#include "esp_system.h"

void app_main(void) {
    esp_chip_info_t info;
    esp_chip_info(&info);

    printf("\n==== CHIP INFO ====\n");
    printf("Model: ESP32 (D0WD-V3 probable)\n");
    printf("Revision (efuse): %d\n", info.revision);
    printf("Cores: %d\n", info.cores);
    printf("Features: WiFi:%s BT:%s BLE:%s\n",
           (info.features & CHIP_FEATURE_WIFI_BGN) ? "yes" : "no",
           (info.features & CHIP_FEATURE_BT) ? "yes" : "no",
           (info.features & CHIP_FEATURE_BLE) ? "yes" : "no");
    printf("===================\n\n");
}
