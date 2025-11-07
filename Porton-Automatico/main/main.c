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
