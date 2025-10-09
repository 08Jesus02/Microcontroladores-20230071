#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mac.h"  // <-- NECESARIO para MACSTR y MAC2STR

static const char *TAG = "softap";

/* ====== CONFIGURA TU AP AQUÍ ====== */
#define AP_SSID         "ESP32_AP"
#define AP_PASSWORD     "12345678"        // mín. 8 chars para WPA2/WPA3
#define AP_CHANNEL      6
#define AP_MAX_CONN     4
// WIFI_AUTH_OPEN (sin clave), WIFI_AUTH_WPA2_PSK, WIFI_AUTH_WPA3_PSK, WIFI_AUTH_WPA2_WPA3_PSK
#define AP_AUTHMODE     WIFI_AUTH_WPA2_PSK
/* =================================== */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "AP iniciado. SSID:%s Canal:%d Auth:%s",
                     AP_SSID, AP_CHANNEL, (AP_AUTHMODE == WIFI_AUTH_OPEN) ? "OPEN" : "PSK");
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
            ESP_LOGI(TAG, "STA conectada: " MACSTR " AID=%d", MAC2STR(e->mac), e->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGI(TAG, "STA desconectada: " MACSTR " AID=%d (reason=%d)",
                     MAC2STR(e->mac), e->aid, e->reason);
            break;
        }

        default:
            break;
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_AP_STAIPASSIGNED) {
            ip_event_ap_staipassigned_t *e = (ip_event_ap_staipassigned_t *)data;
            ESP_LOGI(TAG, "IP asignada a STA: " IPSTR, IP2STR(&e->ip));
        }
    }
}

static void wifi_init_softap(void)
{
    // Crear interfaz AP y obtener el handle
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Detener DHCP antes de cambiar IP
    esp_netif_dhcps_stop(ap_netif);

    // Configurar nueva IP, máscara y gateway
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr      = esp_ip4addr_aton("10.0.0.1");
    ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
    ip_info.gw.addr      = esp_ip4addr_aton("10.0.0.1");
    esp_netif_set_ip_info(ap_netif, &ip_info);

    // Reiniciar DHCP
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    // Registrar manejadores para WIFI_EVENT e IP_EVENT
    ESP_ERROR_CHECK( esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_event_handler, NULL, NULL) );

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.ap.ssid, AP_SSID, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, AP_PASSWORD, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len       = strlen((const char *)wifi_config.ap.ssid);
    wifi_config.ap.channel        = AP_CHANNEL;
    wifi_config.ap.max_connection = AP_MAX_CONN;
    wifi_config.ap.authmode       = AP_AUTHMODE;
    wifi_config.ap.ssid_hidden    = 0;       // 1 para ocultar SSID
    wifi_config.ap.beacon_interval= 100;     // ms

    if (wifi_config.ap.authmode == WIFI_AUTH_OPEN) {
        wifi_config.ap.password[0] = '\0';   // sin clave si es OPEN
    }

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI(TAG, "SoftAP listo. Conéctate a \"%s\". IP del AP: 192.168.10.1", AP_SSID);
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    wifi_init_softap();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
