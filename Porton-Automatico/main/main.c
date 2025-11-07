/**
 * @file main.c
 * @brief Portón automático con ESP32 + ESP-IDF + MQTT
 * @version 1.0
 *
 * @details
 * - Arquitectura: Máquina de estados con *un bucle infinito por estado* (while(1))
 *   y múltiples verificaciones por `if` (LSA/LSC, temporizadores, comandos MQTT).
 * - Conectividad: Wi-Fi STA + MQTT. Publicación con QoS 1 y retain=1 en estado/telemetría.
 * - Seguridad: Interlock en relés (nunca abrir y cerrar a la vez) + timeouts.
 * - Telemetría: Publicación periódica y cada vez que cambia el estado.
 *
 * ----------------------------- CABLEADO (RESUMEN) -----------------------------
 * Pines usados por este firmware:
 *  - PIN_LSA  = GPIO34  → Fin de carrera ABIERTO (entrada; SIN pull interno)
 *  - PIN_LSC  = GPIO35  → Fin de carrera CERRADO (entrada; SIN pull interno)
 *  - PIN_MOTOR_A = GPIO13 → Relé para ABRIR
 *  - PIN_MOTOR_C = GPIO12 → Relé para CERRAR   (*ver nota de strapping abajo*)
 *  - PIN_LAMP    = GPIO2  → Lámpara/LED de estado (*ver nota de strapping abajo*)
 *
 * IMPORTANTE (GPIO34/35):
 *  - No poseen pull-up/pull-down internos. Debes usar resistencias externas.
 *  - Esquema recomendado (LS tipo NC, activo en BAJO):
 *        3V3 ──[10kΩ pull-up]──┬── GPIO34/35
 *                              └── LS (NC) ── GND
 *    En el código: #define LM_ACTIVO 0  (activo en bajo)
 *
 * NOTA SOBRE STRAPPING PINS:
 *  - GPIO12 y GPIO2 son pines de arranque (strapping). Evita mantenerlos
 *    forzados a niveles que impidan el boot. Con módulos de relé, asegúrate
 *    de que al encender la placa el relé esté *inactivo* por defecto.
 *    (Si tienes problemas de arranque, cambia PIN_MOTOR_C y/o PIN_LAMP
 *    a GPIOs no-strapping como 4, 14, 27, 26, 25, 23, 22, 21, 19, 18, 17, 16, 5, 33, 32).
 *
 * Estados (g_estado):
 *  - ESTADO_INICIAL, ESTADO_ERROR, ESTADO_ABRIENDO, ESTADO_ABIERTO,
 *    ESTADO_CERRANDO, ESTADO_CERRADO, ESTADO_DETENIDO, ESTADO_DESCONOCIDO.
 *  - Al iniciar: se leen los LS para clasificar posición; si ambos activos → ERROR,
 *    si ninguno activo → DESCONOCIDO.
 *
 * Códigos de error (g_error_code):
 *  - 0  (ERR_OK)              = Sin error
 *  - 1  (ERR_TIMEOUT_OPEN)    = Timeout abriendo (no llegó a LSA dentro de T_OPEN_MS)
 *  - 2  (ERR_TIMEOUT_CLOSE)   = Timeout cerrando (no llegó a LSC dentro de T_CLOSE_MS)
 *  - 3  (ERR_LS_INCONSISTENT) = Inconsistencia de LS (LSA y LSC activos a la vez)
 *  - 99 (ERR_STATE_GUARDRAIL) = Guardrail: estado inválido en el dispatcher
 *
 * Comandos MQTT (JSON en TOPIC_CMD):
 *  - {"cmd":"OPEN"}      → iniciar apertura
 *  - {"cmd":"CLOSE"}     → iniciar cierre
 *  - {"cmd":"STOP"}      → detener motor; pasa a DETENIDO
 *  - {"cmd":"TOGGLE"}    → alterna sentido según estado (ver lógica)
 *  - {"cmd":"LAMP_ON"}   → enciende lámpara
 *  - {"cmd":"LAMP_OFF"}  → apaga lámpara
 *
 * Tópicos MQTT:
 *  - TOPIC_CMD    = "gate/cmd"     (suscripción)
 *  - TOPIC_STATUS = "gate/status"  (publicación on-change; QoS1 + retain)
 *  - TOPIC_TELE   = "gate/tele"    (publicación periódica; QoS1 + retain)
 *
 * Ejemplo de JSON publicado:
 *  {"state":"CERRANDO","lsa_open":false,"lsc_closed":false,
 *   "motor_open":false,"motor_close":true,"err":0}
 *
 * Parámetros de tiempo:
 *  - T_OPEN_MS   = 15000 ms (timeout al abrir)
 *  - T_CLOSE_MS  = 15000 ms (timeout al cerrar)
 *  - DEBOUNCE_MS = 20 ms    (antirrebote por software en LS)
 *  - PUB_PERIOD_MS = 30000 ms (telemetría periódica)
 *
 * Wi-Fi / MQTT (en este código):
 *  - WIFI_SSID = "Pagalo"
 *  - WIFI_PASS = "notetocanet"
 *  - MQTT_URI  = "mqtt://broker.emqx.io"
 *  - Keepalive 30 s; clean session deshabilitado (retiene último estado).
 *
 * Construcción (ESP-IDF >= 5.x):
 *  - La configuración de MQTT usa campos anidados:
 *      .broker.address.uri, .session.keepalive, .session.disable_clean_session
 *  - CMakeLists.txt de referencia:
 *      idf_component_register(SRCS "main.c"
 *          REQUIRES driver esp_wifi mqtt esp_netif esp_event cjson)
 */


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

// ------------------------------ ESTADOS ---------------------------------------
#define ESTADO_INICIAL     0
#define ESTADO_ERROR       1
#define ESTADO_ABRIENDO    2
#define ESTADO_ABIERTO     3
#define ESTADO_CERRANDO    4
#define ESTADO_CERRADO     5
#define ESTADO_DETENIDO    6
#define ESTADO_DESCONOCIDO 7

// (Opcional) Etiquetas simbólicas de errores para legibilidad:
#define ERR_OK                 0
#define ERR_TIMEOUT_OPEN       1
#define ERR_TIMEOUT_CLOSE      2
#define ERR_LS_INCONSISTENT    3
#define ERR_STATE_GUARDRAIL   99

// ----------------------- CONFIGURACIÓN AJUSTABLE ------------------------------
// --- Pines ---
#define PIN_LSC        GPIO_NUM_35   // Limit switch CERRADO (solo entrada, sin pulls internos)
#define PIN_LSA        GPIO_NUM_34   // Limit switch ABIERTO (solo entrada, sin pulls internos)
#define PIN_MOTOR_A    GPIO_NUM_13   // Relay motor: ABRIR
#define PIN_MOTOR_C    GPIO_NUM_12   // Relay motor: CERRAR
#define PIN_LAMP       GPIO_NUM_2    // LED/lámpara de estado (opcional)

// --- Lógica de entradas ---
// 0: activo en BAJO (NC + pull-up externo recomendado con GPIO34/35)
// 1: activo en ALTO (NO + pull-down externo)
#define LM_ACTIVO      0
#define LM_NOACTIVO    (!LM_ACTIVO)

// --- Tiempos (ms) ---
#define T_OPEN_MS      15000         ///< Timeout máximo para apertura
#define T_CLOSE_MS     15000         ///< Timeout máximo para cierre
#define DEBOUNCE_MS    20            ///< Antirrebote por software
#define PUB_PERIOD_MS  30000         ///< Periodo de telemetría

// --- Wi-Fi / MQTT ---
#define WIFI_SSID      "Pagalo"
#define WIFI_PASS      "notetocanet"
#define MQTT_URI       "mqtt://broker.emqx.io"   ///< Debe incluir esquema mqtt:// o mqtts://
#define TOPIC_CMD      "gate/cmd"
#define TOPIC_STATUS   "gate/status"
#define TOPIC_TELE     "gate/tele"

// ------------------------------- GLOBALES -------------------------------------
static const char *TAG = "GATE";
static esp_mqtt_client_handle_t g_client = NULL;

static volatile int g_estado = ESTADO_INICIAL; ///< Estado actual de la FSM
static int g_estado_prev = -1;                 ///< Último estado publicado
static int g_motorA = 0, g_motorC = 0;        ///< Flags de salidas de motor
static int g_lsa = 0, g_lsc = 0;              ///< Lecturas de LS (interpretadas vs LM_ACTIVO)
static int g_error_code = ERR_OK;             ///< Código de error actual

/// Cola de comandos MQTT (productor: handler MQTT; consumidor: FSM)
typedef enum {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_TOGGLE,
    CMD_LAMP_ON,
    CMD_LAMP_OFF
} gate_cmd_t;

static QueueHandle_t q_cmd;         ///< Cola de comandos entrantes
static uint64_t g_last_pub_us = 0;  ///< Timestamp última telemetría publicada

// ------------------------------ UTILIDADES ------------------------------------
/**
 * @brief Lectura con antirrebote por software.
 *
 * @param pin  GPIO a leer.
 * @param ms   Tiempo mínimo estable (ms) para aceptar el nivel.
 * @return int Nivel estable leído (0/1) al finalizar el periodo.
 *
 * @details
 * Se muestrea cada 5 ms. Si el nivel cambia dentro de la ventana,
 * reinicia el conteo de estabilidad.
 */
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

/**
 * @brief Actualiza g_lsa / g_lsc aplicando antirrebote y comparando contra LM_ACTIVO.
 */
static inline void leer_sensores(void) {
    g_lsa = (debounce_read(PIN_LSA, DEBOUNCE_MS) == LM_ACTIVO);
    g_lsc = (debounce_read(PIN_LSC, DEBOUNCE_MS) == LM_ACTIVO);
}

/**
 * @brief Devuelve cadena legible del estado.
 */
static const char* estado_str(int e) {
    switch (e) {
        case ESTADO_INICIAL:     return "INICIAL";
        case ESTADO_ERROR:       return "ERROR";
        case ESTADO_ABRIENDO:    return "ABRIENDO";
        case ESTADO_ABIERTO:     return "ABIERTO";
        case ESTADO_CERRANDO:    return "CERRANDO";
        case ESTADO_CERRADO:     return "CERRADO";
        case ESTADO_DETENIDO:    return "DETENIDO";
        case ESTADO_DESCONOCIDO: return "DESCONOCIDO";
        default:                 return "???";
    }
}

/**
 * @brief Apaga ambos relés (estado seguro).
 */
static inline void motor_stop(void) {
    gpio_set_level(PIN_MOTOR_A, 0);
    gpio_set_level(PIN_MOTOR_C, 0);
    g_motorA = g_motorC = 0;
}

/**
 * @brief Activa relé de apertura garantizando interlock.
 */
static inline void motor_abrir(void) {
    gpio_set_level(PIN_MOTOR_C, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_MOTOR_A, 1);
    g_motorA = 1; g_motorC = 0;
}

/**
 * @brief Activa relé de cierre garantizando interlock.
 */
static inline void motor_cerrar(void) {
    gpio_set_level(PIN_MOTOR_A, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_MOTOR_C, 1);
    g_motorA = 0; g_motorC = 1;
}

/**
 * @brief Control de lámpara/LED de estado.
 */
static inline void lamp_on(bool on) { gpio_set_level(PIN_LAMP, on ? 1 : 0); }

/**
 * @brief Publica JSON a un tópico MQTT (QoS1, retain 1).
 *
 * @param topic        Tópico destino.
 * @param include_mot  Si true, incluye flags de motores.
 * @param include_err  Si true, incluye g_error_code.
 *
 * @note Requiere g_client inicializado.
 */
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
        esp_mqtt_client_publish(g_client, topic, js, 0, 1, 1);
        free(js);
    }
    cJSON_Delete(root);
}

/**
 * @brief Publica estado solo si cambió (y traza por log).
 */
static inline void publicar_estado_si_cambia(void) {
    if (g_estado != g_estado_prev) {
        g_estado_prev = g_estado;
        publicar_json(TOPIC_STATUS, true, true);
        ESP_LOGI(TAG, "Estado => %s", estado_str(g_estado));
    }
}

/**
 * @brief Publica telemetría cada PUB_PERIOD_MS.
 */
static inline void tick_telemetria(void) {
    uint64_t now = esp_timer_get_time();
    if ((now - g_last_pub_us) > (uint64_t)PUB_PERIOD_MS * 1000ULL) {
        publicar_json(TOPIC_TELE, true, true);
        g_last_pub_us = now;
    }
}

/**
 * @brief Obtiene un comando de la cola sin bloquear.
 *
 * @param out_cmd Comando de salida.
 * @return true si se obtuvo un comando; false en caso contrario.
 */
static inline bool fetch_cmd(gate_cmd_t *out_cmd) {
    return xQueueReceive(q_cmd, out_cmd, 0) == pdTRUE;
}

// ------------------------------- Wi-Fi ----------------------------------------
/**
 * @brief Inicializa Wi-Fi en modo estación y lanza un intento de conexión con backoff.
 *
 * @details
 * No bloquea indefinidamente; tras unos intentos, confía en la lógica de reconexión
 * del cliente MQTT/eventos del stack de red.
 */
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

// ------------------------------- MQTT -----------------------------------------
/**
 * @brief Parsea JSON de comando (del tópico gate/cmd).
 *
 * @param data Buffer JSON.
 * @param len  Longitud del buffer.
 * @return gate_cmd_t Comando reconocido o CMD_NONE.
 */
static gate_cmd_t parse_cmd_json(const char *data, int len) {
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return CMD_NONE;
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    gate_cmd_t out = CMD_NONE;
    if (cJSON_IsString(cmd) && cmd->valuestring) {
        if (!strcasecmp(cmd->valuestring, "OPEN"))      out = CMD_OPEN;
        else if (!strcasecmp(cmd->valuestring, "CLOSE"))out = CMD_CLOSE;
        else if (!strcasecmp(cmd->valuestring, "STOP")) out = CMD_STOP;
        else if (!strcasecmp(cmd->valuestring, "TOGGLE")) out = CMD_TOGGLE;
        else if (!strcasecmp(cmd->valuestring, "LAMP_ON"))  out = CMD_LAMP_ON;
        else if (!strcasecmp(cmd->valuestring, "LAMP_OFF")) out = CMD_LAMP_OFF;
    }
    cJSON_Delete(root);
    return out;
}

/**
 * @brief Manejador de eventos MQTT (conexión, datos, etc.).
 *
 * @details
 * - Al conectar: se suscribe a TOPIC_CMD y publica estado actual.
 * - Al recibir datos: parsea JSON y encola un comando si es válido.
 */
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

/**
 * @brief Inicializa el cliente MQTT y arranca la sesión.
 *
 * @note ESP-IDF 5.x usa estructura anidada para configuración.
 */
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

// --------------------------- BUCLES DE ESTADO ---------------------------------
/**
 * @brief Bucle del estado ERROR.
 *
 * @return Siguiente estado a ejecutar.
 *
 * @details
 * Permanece en ERROR mientras LSA y LSC estén ambos activos (inconsistencia).
 * Si se corrige el cableado/lectura, deja ERROR y va al estado conocido por LS.
 * Comandos: OPEN/CLOSE/TOGGLE fuerzan salida de ERROR hacia movimiento.
 */
static int loop_error(void) {
    ESP_LOGW(TAG, "Entrando a ERROR (code=%d).", g_error_code);
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        // Si ambos LS activos => sigue inconsistente
        if (!(g_lsa && g_lsc)) {
            // Si se resolvió, escoger estado basado en LS
            if (g_lsc && !g_lsa) return ESTADO_CERRADO;
            if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
            if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;
        }

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            if (cmd == CMD_OPEN)     return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)    return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE)   return ESTADO_ABRIENDO; // preferencia abrir
            // STOP en ERROR: no hace nada especial
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Bucle del estado ABIERTO (motor parado).
 *
 * @return Próximo estado.
 *
 * @note (OPCIONAL) Para limpiar error al entrar a estado sano:
 *   g_error_code = ERR_OK;
 */
static int loop_abierto(void) {
    // g_error_code = ERR_OK; // <-- descomenta si deseas limpiar error al entrar
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_CLOSE || cmd == CMD_TOGGLE) return ESTADO_CERRANDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Bucle del estado CERRADO (motor parado).
 */
static int loop_cerrado(void) {
    // g_error_code = ERR_OK; // opcional
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN || cmd == CMD_TOGGLE) return ESTADO_ABRIENDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Bucle del estado DETENIDO (motor parado a mitad de carrera).
 */
static int loop_detenido(void) {
    // g_error_code = ERR_OK; // opcional
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)   return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)  return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) return g_lsc ? ESTADO_ABRIENDO : ESTADO_CERRANDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Bucle del estado DESCONOCIDO (no está ni en LSA ni en LSC).
 */
static int loop_desconocido(void) {
    // g_error_code = ERR_OK; // opcional
    motor_stop();
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)     return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)    return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE)   return ESTADO_ABRIENDO; // preferencia abrir
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Bucle del estado ABRIENDO (motor en marcha hacia abierto).
 *
 * @details
 * - Fija un deadline en T_OPEN_MS.
 * - Termina en ABIERTO si LSA activo; en ERROR si vence el timeout.
 * - Comandos STOP/TOGGLE detienen; CLOSE invierte hacia CERRANDO.
 */
static int loop_abriendo(void) {
    motor_abrir();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_OPEN_MS * 1000ULL;
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { motor_stop(); g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) { motor_stop(); return ESTADO_ABIERTO; }
        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = ERR_TIMEOUT_OPEN; return ESTADO_ERROR; }

        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_STOP)   { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_CLOSE)  { motor_cerrar(); deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL; g_estado = ESTADO_CERRANDO; publicar_estado_si_cambia(); return ESTADO_CERRANDO; }
            if (cmd == CMD_TOGGLE) { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }

        tick_telemetria();
        publicar_estado_si_cambia();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Bucle del estado CERRANDO (motor en marcha hacia cerrado).
 */
static int loop_cerrando(void) {
    motor_cerrar();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL;
    publicar_estado_si_cambia();

    while (1) {
        leer_sensores();

        if (g_lsa && g_lsc) { motor_stop(); g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsc && !g_lsa) { motor_stop(); return ESTADO_CERRADO; }
        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = ERR_TIMEOUT_CLOSE; return ESTADO_ERROR; }

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

/**
 * @brief Estado inicial: clasifica posición según LS.
 */
static int loop_inicial(void) {
    leer_sensores();
    if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
    if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
    if (g_lsc && !g_lsa) return ESTADO_CERRADO;
    return ESTADO_DESCONOCIDO;
}

// ----------------------------- DISPATCHER FSM ---------------------------------
/**
 * @brief Tarea principal de la FSM: llama al bucle del estado actual y cambia
 *        al estado que retorne cada bucle.
 */
static void state_machine_task(void *arg) {
    g_last_pub_us = esp_timer_get_time();
    lamp_on(false);
    motor_stop();

    while (1) {
        switch (g_estado) {
            case ESTADO_INICIAL:     g_estado = loop_inicial();     publicar_estado_si_cambia(); break;
            case ESTADO_ABIERTO:     g_estado = loop_abierto();     break;
            case ESTADO_CERRADO:     g_estado = loop_cerrado();     break;
            case ESTADO_ABRIENDO:    g_estado = loop_abriendo();    break;
            case ESTADO_CERRANDO:    g_estado = loop_cerrando();    break;
            case ESTADO_DETENIDO:    g_estado = loop_detenido();    break;
            case ESTADO_DESCONOCIDO: g_estado = loop_desconocido(); break;
            case ESTADO_ERROR:       g_estado = loop_error();       break;
            default:                 g_estado = ESTADO_ERROR;       g_error_code = ERR_STATE_GUARDRAIL; break;
        }
        // El while(1) repite y ejecuta el nuevo estado.
    }
}

// ------------------------------ INICIALIZACIÓN --------------------------------
/**
 * @brief Configura GPIOs de entradas y salidas.
 *
 * @note GPIO34/35 no admiten pull internos. Si requieres nivel definido,
 *       usa resistencias externas (ver encabezado).
 */
static void gpio_init_all(void) {
    // Entradas (GPIO34/35: pulls internos deshabilitados por hardware)
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

/**
 * @brief Punto de entrada: NVS, GPIO, Wi-Fi, cola de comandos, MQTT y FSM.
 */
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    gpio_init_all();
    wifi_init_sta();

    // Crear cola de comandos ANTES de arrancar MQTT para evitar carreras
    q_cmd = xQueueCreate(16, sizeof(gate_cmd_t));

    mqtt_init();

    xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Sistema iniciado.");
}