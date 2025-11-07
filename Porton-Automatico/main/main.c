// main.c - Portón automático con ESP32 + ESP-IDF + MQTT
// Máquina de estados con "bucles infinitos" por estado y muchos IF
#define ESTADO_INICIAL     0
#define ESTADO_ERROR       1
#define ESTADO_ABRIENDO    2
#define ESTADO_ABIERTO     3
#define ESTADO_CERRANDO    4
#define ESTADO_CERRADO     5
#define ESTADO_DETENIDO    6
#define ESTADO_DESCONOCIDO 7

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>    // strcasecmp
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include "cJSON.h"

// ================= CONFIGURACIÓN AJUSTABLE ==================
// --- Pines ---
#define PIN_LSC        GPIO_NUM_35   // Limit switch CERRADO (solo entrada, sin pulls internos)
#define PIN_LSA        GPIO_NUM_34   // Limit switch ABIERTO (solo entrada, sin pulls internos)
#define PIN_MOTOR_A    GPIO_NUM_14   // Relay motor: ABRIR
#define PIN_MOTOR_C    GPIO_NUM_27   // Relay motor: CERRAR
#define PIN_LAMP       GPIO_NUM_2    // LED/lámpara de estado (opcional)

// --- Lógica de entradas (NC recomendado con pull-up EXTERNO si usas GPIO 34/35) ---
#define LM_ACTIVO      0   // 0: activo en BAJO (NC + pull-up). 1: activo en ALTO (NO + pull-down)
#define LM_NOACTIVO    (!LM_ACTIVO)

// --- Tiempos (ms) ---
#define T_OPEN_MS      15000
#define T_CLOSE_MS     15000
#define DEBOUNCE_MS    20
#define PUB_PERIOD_MS  30000  // telemetría periódica

// --- Wi-Fi / MQTT ---
#define WIFI_SSID      "Docentes_Administrativos"
#define WIFI_PASS      "Adm1N2584km"
#define MQTT_URI       "mqtt://broker.emqx.io"   // incluye "mqtt://"
#define TOPIC_CMD      "gate/cmd"
#define TOPIC_STATUS   "gate/status"
#define TOPIC_TELE     "gate/tele"

// ================== GLOBALES ==================
static const char *TAG = "GATE";
static esp_mqtt_client_handle_t g_client = NULL;

static volatile int g_estado = ESTADO_INICIAL;
static int g_estado_prev = -1;
static int g_motorA = 0, g_motorC = 0; // flags salidas
static int g_lsa = 0, g_lsc = 0;       // entradas (con antirrebote)
static int g_error_code = 0;

typedef enum {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_TOGGLE,
    CMD_LAMP_ON,
    CMD_LAMP_OFF
} gate_cmd_t;

static QueueHandle_t q_cmd;
static uint64_t g_last_pub_us = 0;

// ================== UTILIDADES ==================
static int debounce_read(gpio_num_t pin, int ms) {
    const int step = 5;
    int stable = gpio_get_level(pin);
    int elapsed = 0;
    while (elapsed < ms) {
        vTaskDelay(pdMS_TO_TICKS(step));
        int now = gpio_get_level(pin);
        if (now != stable) {
            stable = now;
            elapsed = 0;
        } else {
            elapsed += step;
        }
    }
    return stable;
}

static inline void leer_sensores(void) {
    g_lsa = (debounce_read(PIN_LSA, DEBOUNCE_MS) == LM_ACTIVO);
    g_lsc = (debounce_read(PIN_LSC, DEBOUNCE_MS) == LM_ACTIVO);
}

static const char* estado_str(int e) {
    switch (e) {
        case ESTADO_INICIAL: return "INICIAL";
        case ESTADO_ERROR: return "ERROR";
        case ESTADO_ABRIENDO: return "ABRIENDO";
        case ESTADO_ABIERTO: return "ABIERTO";
        case ESTADO_CERRANDO: return "CERRANDO";
        case ESTADO_CERRADO: return "CERRADO";
        case ESTADO_DETENIDO: return "DETENIDO";
        case ESTADO_DESCONOCIDO: return "DESCONOCIDO";
        default: return "???";
    }
}

static inline void motor_stop(void) {
    gpio_set_level(PIN_MOTOR_A, 0);
    gpio_set_level(PIN_MOTOR_C, 0);
    g_motorA = g_motorC = 0;
}

static inline void motor_abrir(void) {
    gpio_set_level(PIN_MOTOR_C, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_MOTOR_A, 1);
    g_motorA = 1; g_motorC = 0;
}

static inline void motor_cerrar(void) {
    gpio_set_level(PIN_MOTOR_A, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_MOTOR_C, 1);
    g_motorA = 0; g_motorC = 1;
}

static inline void lamp_on(bool on) { gpio_set_level(PIN_LAMP, on ? 1 : 0); }

static void publicar_json(const char *topic, bool include_mot, bool include_err) {
    if (!g_client) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", estado_str(g_estado));
    cJSON_AddBoolToObject(root, "lsa_open", g_lsa);
    cJSON_AddBoolToObject(root, "lsc_closed", g_lsc);
    if (include_mot) {
        cJSON_AddBoolToObject(root, "motor_open", g_motorA);
        cJSON_AddBoolToObject(root, "motor_close", g_motorC);
    }
    if (include_err) cJSON_AddNumberToObject(root, "err", g_error_code);

    char *js = cJSON_PrintUnformatted(root);
    if (js) {
        // QoS 1, retain 1: último estado se queda en el broker
        esp_mqtt_client_publish(g_client, topic, js, 0, 1, 1);
        free(js);
    }
    cJSON_Delete(root);
}

static inline void publicar_estado_si_cambia(void) {
    if (g_estado != g_estado_prev) {
        g_estado_prev = g_estado;
        publicar_json(TOPIC_STATUS, true, true);
        ESP_LOGI(TAG, "Estado => %s", estado_str(g_estado));
    }
}

static inline void tick_telemetria(void) {
    uint64_t now = esp_timer_get_time();
    if ((now - g_last_pub_us) > (uint64_t)PUB_PERIOD_MS * 1000ULL) {
        publicar_json(TOPIC_TELE, true, true);
        g_last_pub_us = now;
    }
}

static inline bool fetch_cmd(gate_cmd_t *out_cmd) {
    return xQueueReceive(q_cmd, out_cmd, 0) == pdTRUE;
}

// ================== Wi-Fi ==================
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = { 0 };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    int backoff = 1000;
    while (true) {
        esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(backoff));
        backoff = backoff < 30000 ? backoff * 2 : 30000;
        if (backoff > 8000) break;
    }
}

// ================== MQTT ==================
static gate_cmd_t parse_cmd_json(const char *data, int len) {
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return CMD_NONE;
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    gate_cmd_t out = CMD_NONE;
    if (cJSON_IsString(cmd) && cmd->valuestring) {
        if (!strcasecmp(cmd->valuestring, "OPEN")) out = CMD_OPEN;
        else if (!strcasecmp(cmd->valuestring, "CLOSE")) out = CMD_CLOSE;
        else if (!strcasecmp(cmd->valuestring, "STOP")) out = CMD_STOP;
        else if (!strcasecmp(cmd->valuestring, "TOGGLE")) out = CMD_TOGGLE;
        else if (!strcasecmp(cmd->valuestring, "LAMP_ON")) out = CMD_LAMP_ON;
        else if (!strcasecmp(cmd->valuestring, "LAMP_OFF")) out = CMD_LAMP_OFF;
    }
    cJSON_Delete(root);
    return out;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t e = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            esp_mqtt_client_subscribe(g_client, TOPIC_CMD, 1);
            publicar_json(TOPIC_STATUS, true, false);
            break;
        case MQTT_EVENT_DATA: {
            char *buf = calloc(1, e->data_len + 1);
            if (!buf) break;
            memcpy(buf, e->data, e->data_len);
            gate_cmd_t cmd = parse_cmd_json(buf, e->data_len);
            free(buf);
            if (cmd != CMD_NONE && q_cmd) {
                xQueueSend(q_cmd, &cmd, 0);
            }
            break;
        }
        default: break;
    }
}

static void mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address.uri = MQTT_URI,
        },
        .session = {
            .keepalive = 30,
            .disable_clean_session = false,
        },
    };
    g_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(g_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_client);
}

// ================== ESTADOS (cada uno con su while(1)) ==================
static int loop_error(void) {
    ESP_LOGW(TAG, "Entrando a ERROR (code=%d).", g_error_code);
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        // Si ambos LS activos => sigue siendo error cableado
        if (g_lsa && g_lsc) {
            // permanece en ERROR
        } else {
            // si se resolvió la inconsistencia, puede pasar a estado conocido por LS
            if (g_lsc && !g_lsa) return ESTADO_CERRADO;
            if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
            if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;
        }

        // Comandos MQTT
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            if (cmd == CMD_STOP) {
                // ya estamos detenidos; nada
            }
            if (cmd == CMD_OPEN)  return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE) return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) {
                // por defecto intenta abrir si está en error
                return ESTADO_ABRIENDO;
            }
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int loop_abierto(void) {
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        // Verificación de coherencia de LS
        if (g_lsa && g_lsc) { g_error_code = 3; return ESTADO_ERROR; }
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;        // cayó a cerrado por hardware
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;   // se movió entre finales

        // Comandos
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_CLOSE) return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) return ESTADO_CERRANDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            // OPEN en ABIERTO no hace nada
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int loop_cerrado(void) {
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = 3; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)   return ESTADO_ABRIENDO;
            if (cmd == CMD_TOGGLE) return ESTADO_ABRIENDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int loop_detenido(void) {
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = 3; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)   return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)  return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) {
                // si está más cerca de cerrado, abrimos; si no, cerramos
                return g_lsc ? ESTADO_ABRIENDO : ESTADO_CERRANDO;
            }
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            // STOP mantiene DETENIDO
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int loop_desconocido(void) {
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = 3; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)   return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)  return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) return ESTADO_ABRIENDO; // preferencia abrir
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            // STOP mantiene DESCONOCIDO
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int loop_abriendo(void) {
    // Al entrar a ABRIENDO: arrancar motor y fijar deadline
    motor_abrir();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_OPEN_MS * 1000ULL;
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        // Seguridad por inconsistencia
        if (g_lsa && g_lsc) { motor_stop(); g_error_code = 3; return ESTADO_ERROR; }

        // Llegó al final de apertura
        if (g_lsa && !g_lsc) { motor_stop(); return ESTADO_ABIERTO; }

        // Timeout de apertura
        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = 1; return ESTADO_ERROR; }

        // Comandos
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_STOP)   { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_CLOSE)  { motor_cerrar(); deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL; g_estado = ESTADO_CERRANDO; publicar_estado_si_cambia(); return ESTADO_CERRANDO; }
            if (cmd == CMD_TOGGLE) { motor_stop(); return ESTADO_DETENIDO; } // o invertir si prefieres
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            // OPEN mientras abre: sin efecto
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int loop_cerrando(void) {
    motor_cerrar();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL;
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { motor_stop(); g_error_code = 3; return ESTADO_ERROR; }

        if (g_lsc && !g_lsa) { motor_stop(); return ESTADO_CERRADO; }

        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = 2; return ESTADO_ERROR; }

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_STOP)   { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_OPEN)   { motor_abrir(); deadline_us = esp_timer_get_time() + (uint64_t)T_OPEN_MS * 1000ULL; g_estado = ESTADO_ABRIENDO; publicar_estado_si_cambia(); return ESTADO_ABRIENDO; }
            if (cmd == CMD_TOGGLE) { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int loop_inicial(void) {
    // Revisa LS para saber dónde está
    leer_sensores();
    if (g_lsa && g_lsc) { g_error_code = 3; return ESTADO_ERROR; }
    if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
    if (g_lsc && !g_lsa) return ESTADO_CERRADO;
    return ESTADO_DESCONOCIDO;
}

// ================== DISPATCHER ==================
static void state_machine_task(void *arg) {
    g_last_pub_us = esp_timer_get_time();
    lamp_on(false);
    motor_stop();

    while (1) {
        switch (g_estado) {
            case ESTADO_INICIAL:     g_estado = loop_inicial();   publicar_estado_si_cambia(); break;
            case ESTADO_ABIERTO:     g_estado = loop_abierto();   break;
            case ESTADO_CERRADO:     g_estado = loop_cerrado();   break;
            case ESTADO_ABRIENDO:    g_estado = loop_abriendo();  break;
            case ESTADO_CERRANDO:    g_estado = loop_cerrando();  break;
            case ESTADO_DETENIDO:    g_estado = loop_detenido();  break;
            case ESTADO_DESCONOCIDO: g_estado = loop_desconocido(); break;
            case ESTADO_ERROR:       g_estado = loop_error();     break;
            default:                 g_estado = ESTADO_ERROR;     g_error_code = 99; break;
        }
        // Cada función de estado retorna el siguiente estado.
        // El while(1) del dispatcher vuelve a llamar el bucle del nuevo estado.
    }
}

// ================== INICIALIZACIÓN ==================
static void gpio_init_all(void) {
    // Entradas (nota: 34/35 no tienen pulls internos; usa resistencias externas si las necesitas)
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_LSA) | (1ULL << PIN_LSC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // ignorado por 34/35
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    // Salidas
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_A) | (1ULL << PIN_MOTOR_C) | (1ULL << PIN_LAMP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    motor_stop();
    lamp_on(false);
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    gpio_init_all();
    wifi_init_sta();

    // Cola de comandos ANTES de MQTT para evitar carreras
    q_cmd = xQueueCreate(16, sizeof(gate_cmd_t));

    mqtt_init();

    xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Sistema iniciado.");
}
