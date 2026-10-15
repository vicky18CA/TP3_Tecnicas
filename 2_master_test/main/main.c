/* ======================================================================
 * TP3 - Técnicas Digitales III
 * MAESTRO Modbus RTU sobre RS485  (ESP-IDF + FreeRTOS, sin Arduino)
 *
 * Funciones implementadas:
 *   0x03  Read Holding Registers        (1 o N registros consecutivos)
 *   0x06  Write Single Holding Register
 *   0x10  Write Multiple Holding Registers (FC16)
 *
 * Tareas FreeRTOS:
 *   uart_rx_task        : recibe tramas completas (fin = silencio T3.5)
 *                         y las manda a la cola + semáforo
 *   modbus_master_task  : ciclo automático opcional (consulta a los
 *                         esclavos 1 y 2), se activa/desactiva desde el menú
 *   consola_menu_task   : MENÚ por consola. Es la interfaz principal:
 *                         el usuario elige qué ver y qué comandar.
 *
 * Por defecto NO se imprime nada solo: ni tramas, ni ciclo automático.
 * Todo se activa a pedido desde el menú.
 * ====================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_chip_info.h"

#define TAG "MODBUS_MASTER"

// ======================================================
// CONFIGURACIÓN
// ======================================================

// UART / RS485
#define PORT_MAESTRO        UART_NUM_1
#define MAESTRO_UART_TXD    GPIO_NUM_17   // -> DI del MAX485
#define MAESTRO_UART_RXD    GPIO_NUM_16   // <- RO del MAX485
#define MAESTRO_UART_RTS    GPIO_NUM_4    // -> DE y RE (puenteados) del MAX485

#define BAUD_RATE           9600          // 9600, 8E1
#define RX_BUFFER_SIZE      256
#define MODBUS_FRAME_SIZE   256

// Silencio que marca el fin de trama (T3.5 = 4.01 ms a 9600 8E1)
#define T35_MS              5

// Tiempo máximo de espera de una respuesta del esclavo
#define RESPUESTA_TIMEOUT_MS  500

// Esclavos
#define SLAVE_ID1           1
#define SLAVE_ID2           2

// Códigos de función
#define FC_READ_HOLDING_REGISTERS    0x03
#define FC_WRITE_SINGLE_REGISTER     0x06
#define FC_WRITE_MULTIPLE_REGISTERS  0x10

// Registros (dirección PDU base 0: 40001 -> 0)
#define REG_ADC1        0
#define REG_ADC2        1
#define REG_CONTADOR    2
#define REG_ESTADO      3
#define REG_SETPOINT    4
#define REG_TIEMPO      5
#define REG_MODO        6
#define REG_ERRORES     7

// ADC del esclavo: 12 bits, atenuación 12 dB (~0 a 3.3 V)
#define ADC_MAX_RAW         4095
#define ADC_VREF_MV         3300

// Código propio para "el esclavo respondió con una excepción Modbus"
#define MB_ERR_EXCEPCION    ESP_ERR_INVALID_STATE

// ======================================================
// OPCIONES ACTIVABLES DESDE EL MENÚ
// ======================================================

static volatile bool mostrar_tramas = false;  // [T] ver TX/RX en hexadecimal
static volatile bool ciclo_auto     = false;  // [C] consulta automática de esclavos 1 y 2

// ======================================================
// ESTRUCTURAS Y OBJETOS DE SINCRONIZACIÓN
// ======================================================

// Trama recibida (uart_rx_task -> modbus_master_task / consola)
typedef struct
{
    uint8_t  data[MODBUS_FRAME_SIZE];
    uint16_t length;
} modbus_frame_t;

static QueueHandle_t     modbus_queue;        // cola de tramas recibidas
static SemaphoreHandle_t response_semaphore;  // avisa que llegó una respuesta
static SemaphoreHandle_t bus_mutex;           // una sola transacción a la vez en el bus

// ======================================================
// CRC-16 MODBUS (poly 0xA001, init 0xFFFF)
//
// El CRC se calcula sobre dirección + función + datos y se
// transmite primero el byte BAJO y después el ALTO.
// Quien recibe una trama recalcula el CRC sobre todo menos
// los 2 últimos bytes y lo compara con el recibido (o, lo que
// es equivalente, el CRC de la trama completa da 0).
// ======================================================

uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;                      // registro CRC con todos los bits en 1

    for (uint16_t pos = 0; pos < length; pos++) // cada byte de la trama
    {
        crc ^= (uint16_t)data[pos];             // XOR con el byte actual

        for (int i = 0; i < 8; i++)             // procesar sus 8 bits
        {
            if (crc & 0x0001)                   // bit menos significativo en 1:
            {
                crc >>= 1;                      //   desplazar y XOR con el polinomio
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;                      // si no, solo desplazar
            }
        }
    }
    return crc;
}

// Agrega el CRC al final de la trama y devuelve la nueva longitud
static int agregar_crc(uint8_t *trama, int n)
{
    uint16_t crc = modbus_crc16(trama, n);
    trama[n]     = crc & 0xFF;          // CRC bajo primero
    trama[n + 1] = (crc >> 8) & 0xFF;   // CRC alto después
    return n + 2;
}

// ======================================================
// UART + RS485
// ======================================================

void rs485_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(PORT_MAESTRO, RX_BUFFER_SIZE, RX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT_MAESTRO, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PORT_MAESTRO, MAESTRO_UART_TXD, MAESTRO_UART_RXD,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // GPIO que controla DE/RE del MAX485
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAESTRO_UART_RTS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(MAESTRO_UART_RTS, 0);        // por defecto: recepción
}

void rs485_send(const uint8_t *data, size_t length)
{
    gpio_set_level(MAESTRO_UART_RTS, 1);                          // 1. modo transmisión
    uart_write_bytes(PORT_MAESTRO, (const char *)data, length);   // 2. enviar
    uart_wait_tx_done(PORT_MAESTRO, pdMS_TO_TICKS(100));          // 3. esperar fin de TX
    gpio_set_level(MAESTRO_UART_RTS, 0);                          // 4. volver a recepción
}

// ======================================================
// TAREA DE RECEPCIÓN UART
//
// Una trama Modbus RTU termina cuando el bus queda en silencio
// por lo menos T3.5. Por eso se lee el primer byte bloqueando
// y después se sigue leyendo con timeout de T3.5.
// ======================================================

void uart_rx_task(void *pvParameters)
{
    TickType_t t35 = pdMS_TO_TICKS(T35_MS);
    if (t35 < 1) t35 = 1;               // si el tick fuera de 10 ms

    modbus_frame_t frame;

    while (1)
    {
        uint8_t b;
        int len = 0;
        bool desbordado = false;

        // 1) Esperar el primer byte de la trama
        if (uart_read_bytes(PORT_MAESTRO, &b, 1, portMAX_DELAY) != 1) continue;
        frame.data[len++] = b;

        // 2) Seguir leyendo hasta que haya un silencio >= T3.5
        while (uart_read_bytes(PORT_MAESTRO, &b, 1, t35) == 1)
        {
            if (len < MODBUS_FRAME_SIZE) frame.data[len++] = b;
            else desbordado = true;
        }

        if (desbordado)
        {
            ESP_LOGW(TAG, "Trama demasiado larga -> descartada");
            continue;
        }

        frame.length = len;

        // 3) Entregar la trama a la cola y avisar con el semáforo
        if (xQueueSend(modbus_queue, &frame, 0) == pdTRUE)
        {
            xSemaphoreGive(response_semaphore);
        }
        else
        {
            ESP_LOGW(TAG, "Cola llena: trama descartada");
        }
    }
}

// ======================================================
// TRANSACCIÓN: envía un pedido y espera la respuesta
//
// Se protege con un mutex porque hay DOS tareas que usan el bus
// (modbus_master_task y consola_menu_task).
// Las tramas TX/RX solo se imprimen si el usuario activó [T].
// ======================================================

// Imprime una trama en hexadecimal en una sola línea (solo con [T] activo)
static void imprimir_trama(const char *etiqueta, const uint8_t *buf, int len)
{
    printf("   %s (%d bytes):", etiqueta, len);
    for (int i = 0; i < len; i++)
        printf(" %02X", buf[i]);
    printf("\n");
}

static esp_err_t modbus_transaccion_locked(const uint8_t *req, int req_len, modbus_frame_t *resp)
{
    // Limpiar restos de respuestas anteriores (tardías, ruido, etc.)
    xQueueReset(modbus_queue);
    xSemaphoreTake(response_semaphore, 0);

    if (mostrar_tramas) imprimir_trama("TX", req, req_len);

    rs485_send(req, req_len);

    // Esperar a que uart_rx_task avise que llegó una trama
    if (xSemaphoreTake(response_semaphore, pdMS_TO_TICKS(RESPUESTA_TIMEOUT_MS)) != pdTRUE)
    {
        if (mostrar_tramas) printf("   RX: (sin respuesta, timeout)\n");
        return ESP_ERR_TIMEOUT;
    }

    if (xQueueReceive(modbus_queue, resp, 0) != pdTRUE)
        return ESP_FAIL;

    if (mostrar_tramas) imprimir_trama("RX", resp->data, resp->length);

    return ESP_OK;
}

static esp_err_t modbus_transaccion(const uint8_t *req, int req_len, modbus_frame_t *resp)
{
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    esp_err_t e = modbus_transaccion_locked(req, req_len, resp);
    xSemaphoreGive(bus_mutex);
    return e;
}

// ======================================================
// VALIDACIÓN COMÚN DE RESPUESTAS
// Orden: largo mínimo -> CRC -> ID -> excepción -> función
// ======================================================

static esp_err_t validar_respuesta(const modbus_frame_t *r, uint8_t slave_id, uint8_t fc)
{
    // Mínimo: ID + función + 1 byte + CRC (una excepción mide 5 bytes)
    if (r->length < 5)
    {
        ESP_LOGW(TAG, "Respuesta demasiado corta (%d bytes)", r->length);
        return ESP_ERR_INVALID_SIZE;
    }

    // CRC calculado sobre toda la trama (con su CRC incluido) debe dar 0
    if (modbus_crc16(r->data, r->length) != 0)
    {
        ESP_LOGW(TAG, "CRC incorrecto en la respuesta");
        return ESP_ERR_INVALID_CRC;
    }

    if (r->data[0] != slave_id)
    {
        ESP_LOGW(TAG, "Respuesta de otro esclavo (esperado %d, llegó %d)", slave_id, r->data[0]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Excepción Modbus: función | 0x80 + código de excepción
    if (r->data[1] == (fc | 0x80))
    {
        ESP_LOGW(TAG, "Esclavo %d respondió con excepción 0x%02X", slave_id, r->data[2]);
        return MB_ERR_EXCEPCION;
    }

    if (r->data[1] != fc)
    {
        ESP_LOGW(TAG, "Function code inesperado: 0x%02X", r->data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

// ======================================================
// FC03: LEER HOLDING REGISTERS (1 a 125 registros consecutivos)
// Pedido:    ID FC START(2) QTY(2) CRC(2)
// Respuesta: ID FC BYTECOUNT DATOS(2*QTY) CRC(2)
// ======================================================

esp_err_t modbus_read_holding_registers(uint8_t slave_id, uint16_t start,
                                        uint16_t qty, uint16_t *dest)
{
    if (qty < 1 || qty > 125) return ESP_ERR_INVALID_ARG;

    uint8_t req[8];
    req[0] = slave_id;
    req[1] = FC_READ_HOLDING_REGISTERS;
    req[2] = (start >> 8) & 0xFF;
    req[3] = start & 0xFF;
    req[4] = (qty >> 8) & 0xFF;
    req[5] = qty & 0xFF;
    agregar_crc(req, 6);

    modbus_frame_t r;
    esp_err_t e = modbus_transaccion(req, sizeof(req), &r);
    if (e != ESP_OK) return e;

    e = validar_respuesta(&r, slave_id, FC_READ_HOLDING_REGISTERS);
    if (e != ESP_OK) return e;

    // ID + FC + BYTECOUNT + datos + CRC(2)
    if (r.length != 5 + 2 * qty || r.data[2] != 2 * qty)
    {
        ESP_LOGW(TAG, "Largo o byte count inesperado");
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < qty; i++)
        dest[i] = ((uint16_t)r.data[3 + 2 * i] << 8) | r.data[4 + 2 * i];

    return ESP_OK;
}

// ======================================================
// FC06: ESCRIBIR UN HOLDING REGISTER
// Pedido:    ID FC REG(2) VALOR(2) CRC(2)
// Respuesta: eco exacto del pedido
// ======================================================

esp_err_t modbus_write_single_register(uint8_t slave_id, uint16_t reg, uint16_t valor)
{
    uint8_t req[8];
    req[0] = slave_id;
    req[1] = FC_WRITE_SINGLE_REGISTER;
    req[2] = (reg >> 8) & 0xFF;
    req[3] = reg & 0xFF;
    req[4] = (valor >> 8) & 0xFF;
    req[5] = valor & 0xFF;
    agregar_crc(req, 6);

    modbus_frame_t r;
    esp_err_t e = modbus_transaccion(req, sizeof(req), &r);
    if (e != ESP_OK) return e;

    e = validar_respuesta(&r, slave_id, FC_WRITE_SINGLE_REGISTER);
    if (e != ESP_OK) return e;

    // La respuesta debe ser un eco del pedido
    if (r.length != 8 || memcmp(r.data, req, 8) != 0)
    {
        ESP_LOGW(TAG, "El eco de FC06 no coincide con el pedido");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

// ======================================================
// FC16: ESCRIBIR MULTIPLES HOLDING REGISTERS (1 a 123)
// Pedido:    ID FC START(2) QTY(2) BYTECOUNT DATOS(2*QTY) CRC(2)
// Respuesta: ID FC START(2) QTY(2) CRC(2)
// ======================================================

esp_err_t modbus_write_multiple_registers(uint8_t slave_id, uint16_t start,
                                          uint16_t qty, const uint16_t *valores)
{
    if (qty < 1 || qty > 123) return ESP_ERR_INVALID_ARG;

    uint8_t req[MODBUS_FRAME_SIZE];
    req[0] = slave_id;
    req[1] = FC_WRITE_MULTIPLE_REGISTERS;
    req[2] = (start >> 8) & 0xFF;
    req[3] = start & 0xFF;
    req[4] = (qty >> 8) & 0xFF;
    req[5] = qty & 0xFF;
    req[6] = qty * 2;                               // cantidad de bytes de datos

    for (int i = 0; i < qty; i++)
    {
        req[7 + 2 * i] = (valores[i] >> 8) & 0xFF;  // byte alto primero
        req[8 + 2 * i] = valores[i] & 0xFF;
    }
    int req_len = agregar_crc(req, 7 + 2 * qty);

    modbus_frame_t r;
    esp_err_t e = modbus_transaccion(req, req_len, &r);
    if (e != ESP_OK) return e;

    e = validar_respuesta(&r, slave_id, FC_WRITE_MULTIPLE_REGISTERS);
    if (e != ESP_OK) return e;

    // Debe repetir START y QTY del pedido
    if (r.length != 8 || memcmp(&r.data[2], &req[2], 4) != 0)
    {
        ESP_LOGW(TAG, "Respuesta de FC16 inconsistente");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

// ======================================================
// AUXILIARES DE PRUEBA / REPORTE DE ERRORES
// ======================================================

static const char *mb_err_str(esp_err_t e)
{
    switch (e)
    {
        case ESP_OK:                   return "OK";
        case ESP_ERR_TIMEOUT:          return "TIMEOUT (el esclavo no respondio)";
        case ESP_ERR_INVALID_CRC:      return "CRC incorrecto";
        case ESP_ERR_INVALID_SIZE:     return "largo de respuesta invalido";
        case ESP_ERR_INVALID_RESPONSE: return "respuesta invalida";
        case MB_ERR_EXCEPCION:         return "excepcion Modbus";
        default:                       return "error de comunicacion";
    }
}

// Informa el resultado de una operación. Devuelve true si salió bien.
static bool mb_check(const char *operacion, uint8_t slave_id, esp_err_t e)
{
    if (e == ESP_OK) return true;
    ESP_LOGE(TAG, "Esclavo %d - %s: %s", slave_id, operacion, mb_err_str(e));
    return false;
}

// Muestra el valor del potenciómetro (ADC1) en crudo, voltios y porcentaje
static void mostrar_adc1(uint8_t slave_id, uint16_t raw)
{
    uint32_t mv  = ((uint32_t)raw * ADC_VREF_MV) / ADC_MAX_RAW;
    uint32_t pct = ((uint32_t)raw * 100) / ADC_MAX_RAW;

    printf(">> Esclavo %d - POTENCIOMETRO (ADC1, 40001): raw = %u | %u.%03u V | %u %%\n",
           slave_id, raw, (unsigned)(mv / 1000), (unsigned)(mv % 1000), (unsigned)pct);
}

// Lee 'qty' registros desde 'start' y los muestra por la UART.
// Si entre ellos está el ADC1 (40001), además lo muestra en V y %.
static void leer_y_mostrar(uint8_t slave_id, uint16_t start, uint16_t qty)
{
    uint16_t regs[16];
    if (qty > 16) qty = 16;

    if (mb_check("FC03", slave_id, modbus_read_holding_registers(slave_id, start, qty, regs)))
    {
        for (int i = 0; i < qty; i++)
        {
            uint16_t dir = 40001 + start + i;
            
            // Si es el Esclavo 2 y el registro es 40002 (REG_BOTON), aclaramos el estado lógico:
            if (slave_id == SLAVE_ID2 && dir == 40002)
            {
                printf(">> Esclavo %d - %d (PULSADOR) = %u -> %s\n", 
                       slave_id, dir, regs[i], regs[i] ? "PRESIONADO" : "SUELTO");
            }
            else
            {
                printf(">> Esclavo %d - %d = %u\n", slave_id, dir, regs[i]);
            }
        }

        if (start == REG_ADC1) // regs[0] es el potenciómetro
            mostrar_adc1(slave_id, regs[0]);
    }
}

// Lee SOLO el registro del potenciómetro (ADC1, 40001) de un esclavo
static void leer_potenciometro(uint8_t slave_id)
{
    uint16_t raw = 0;
    if (mb_check("FC03 (ADC1)", slave_id, modbus_read_holding_registers(slave_id, REG_ADC1, 1, &raw)))
        mostrar_adc1(slave_id, raw);
}

// Escribe Estado (FC06) y lo relee (FC03) para confirmar que el esclavo lo aplicó
static void comandar_led(uint8_t slave_id, uint16_t estado)
{
    // 1) El eco del FC06 ya valida que el esclavo recibió y aceptó el valor
    if (!mb_check("FC06", slave_id, modbus_write_single_register(slave_id, REG_ESTADO, estado)))
        return;

    vTaskDelay(pdMS_TO_TICKS(100));

    // 2) Releer el registro para verificar que quedó guardado
    uint16_t leido = 0xFFFF;
    if (!mb_check("FC03 (verificacion)", slave_id,
                  modbus_read_holding_registers(slave_id, REG_ESTADO, 1, &leido)))
        return;

    if (leido == estado)
        printf(">> OK: esclavo %d confirmo Estado (40004) = %u -> LED %s\n",
               slave_id, leido, leido ? "ON (parpadeando)" : "OFF");
    else
        printf(">> FALLO: esclavo %d tiene Estado = %u, se esperaba %u\n",
               slave_id, leido, estado);
}

// Cambia el período de parpadeo con FC16 (Setpoint, Tiempo, Modo)
static void cambiar_parpadeo(uint8_t slave_id, uint16_t periodo_ms)
{
    const uint16_t params[3] = { 100, periodo_ms, 1 };   // Setpoint, Tiempo (ms), Modo

    if (mb_check("FC16", slave_id,
                 modbus_write_multiple_registers(slave_id, REG_SETPOINT, 3, params)))
    {
        printf(">> OK: esclavo %d - periodo de parpadeo = %u ms\n", slave_id, periodo_ms);
    }
}

// Pruebas obligatorias 1 a 5 del TP, ejecutadas a pedido con [R]
static void ejecutar_pruebas(void)
{
    printf("\n=== Prueba 1: leer UN registro (40001) del esclavo 1 ===\n");
    leer_y_mostrar(SLAVE_ID1, REG_ADC1, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n=== Prueba 2: leer VARIOS registros (40001-40004) del esclavo 1 ===\n");
    leer_y_mostrar(SLAVE_ID1, REG_ADC1, 4);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n=== Prueba 3: FC16, escribir 40005-40007 (Setpoint, Tiempo, Modo) ===\n");
    const uint16_t valores[3] = { 100, 500, 1 };
    if (mb_check("FC16", SLAVE_ID1, modbus_write_multiple_registers(SLAVE_ID1, REG_SETPOINT, 3, valores)))
        leer_y_mostrar(SLAVE_ID1, REG_ADC1, 7);     // releer para verificar
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n=== Prueba 4: activar parpadeo del LED en esclavos 1 y 2 ===\n");
    comandar_led(SLAVE_ID1, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    comandar_led(SLAVE_ID2, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n=== Prueba 5: consultar alternativamente esclavo 1 y esclavo 2 ===\n");
    leer_y_mostrar(SLAVE_ID1, REG_ADC1, 4);
    vTaskDelay(pdMS_TO_TICKS(100));
    leer_y_mostrar(SLAVE_ID2, REG_ADC1, 4);
}

// ======================================================
// TAREA DEL MAESTRO MODBUS (ciclo automático opcional)
//
// Solo consulta a los esclavos cuando el usuario activó [C].
// Ante cualquier error el programa sigue funcionando normalmente.
// ======================================================

void modbus_master_task(void *pvParameters)
{
    while (1)
    {
        if (!ciclo_auto)
        {
            vTaskDelay(pdMS_TO_TICKS(200));     // inactivo: espera sin tocar el bus
            continue;
        }

        printf("\n--- [Ciclo] Consultando esclavo 1 ---\n");
        leer_y_mostrar(SLAVE_ID1, REG_ADC1, 4);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!ciclo_auto) continue;              // por si lo apagaron mientras tanto

        printf("--- [Ciclo] Consultando esclavo 2 ---\n");
        leer_y_mostrar(SLAVE_ID2, REG_ADC1, 4);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ======================================================
// MENÚ DE CONSOLA
// ======================================================

static void mostrar_menu(void)
{
    printf("\n======================================================\n");
    printf("              MENU DE CONTROL MODBUS RTU              \n");
    printf("======================================================\n");
    printf(" LED / ESCRITURA\n");
    printf("   [1] LED Esclavo 1 -> ON  (FC06)\n");
    printf("   [0] LED Esclavo 1 -> OFF (FC06)\n");
    printf("   [3] LED Esclavo 2 -> ON  (FC06)\n");
    printf("   [4] LED Esclavo 2 -> OFF (FC06)\n");
    printf("   [+] Parpadeo rapido  200 ms, Esclavo 1 (FC16)\n");
    printf("   [-] Parpadeo lento  1000 ms, Esclavo 1 (FC16)\n");
    printf("   [*] Parpadeo rapido 200 ms,  Esclavo 2 (FC16)\n");
    printf("   [/] Parpadeo lento 1000 ms,  Esclavo 2 (FC16)\n");
    printf(" LECTURA\n");
    printf("   [P] Potenciometro (ADC1) - Esclavo 1\n");
    printf("   [Q] Potenciometro (ADC1) -Esclavo 2\n");
    printf("   [L] Todos los registros - Esclavo 1 (40001-40007)\n");
    printf("   [2] Registros 40001-40004 - Esclavo 2\n");
    printf(" OPCIONES\n");
    printf("   [T] Ver tramas TX/RX en hex ........ %s\n", mostrar_tramas ? "ACTIVADO" : "desactivado");
    printf("   [C] Ciclo automatico esclavos 1 y 2  %s\n", ciclo_auto ? "ACTIVADO" : "desactivado");
    printf("   [K] Enviar trama con CRC corrupto (Prueba Timeout - Esclavo 1)\n");
    printf("   [O] Enviar trama con CRC corrupto (Prueba Timeout - Esclavo 2)\n");
    printf("   [R] Ejecutar pruebas obligatorias 1 a 5\n");
    printf("   [M] Mostrar este menu\n");
    printf("======================================================\n");
    printf("Elegi una opcion: ");
    fflush(stdout);
}

void consola_menu_task(void *pvParameters)
{
    // Espera inicial para que arranquen las tareas y se estabilice el bus
    vTaskDelay(pdMS_TO_TICKS(1500));

    mostrar_menu();

    while (1)
    {
        int c = fgetc(stdin);

        // Si no hay datos en el buffer de entrada, esperamos un momento
        if (c == EOF)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Se ignoran Enter / espacios: el menú se vuelve a mostrar solo
        // después de cada comando
        if (c == '\n' || c == '\r' || c == ' ') continue;

        printf("%c\n", c);                      // eco de la opción elegida

        switch (c)
        {
            case '1': comandar_led(SLAVE_ID1, 1); break;
            case '0': comandar_led(SLAVE_ID1, 0); break;
            case '3': comandar_led(SLAVE_ID2, 1); break;
            case '4': comandar_led(SLAVE_ID2, 0); break;

            case '+': cambiar_parpadeo(SLAVE_ID1, 200);  break;
            case '-': cambiar_parpadeo(SLAVE_ID1, 1000); break;
            case '*': cambiar_parpadeo(SLAVE_ID2, 200);  break;
            case '/': cambiar_parpadeo(SLAVE_ID2, 1000); break;
            case 'p':
            case 'P': leer_potenciometro(SLAVE_ID1); break;

            case 'q':
            case 'Q': leer_potenciometro(SLAVE_ID2); break;

            case 'l':
            case 'L': leer_y_mostrar(SLAVE_ID1, REG_ADC1, 7); break;

            case '2': leer_y_mostrar(SLAVE_ID2, REG_ADC1, 4); break;

            case 't':
            case 'T':
                mostrar_tramas = !mostrar_tramas;
                printf(">> Tramas TX/RX: %s\n", mostrar_tramas ? "ACTIVADAS" : "desactivadas");
                break;

            case 'c':
            case 'C':
                ciclo_auto = !ciclo_auto;
                printf(">> Ciclo automatico: %s\n", ciclo_auto ? "ACTIVADO" : "desactivado");
                break;

            case 'r':
            case 'R': ejecutar_pruebas(); break;

            case 'k':
            case 'K':
            {
                printf("\n>> [TEST] Enviando lectura con CRC corrupto al Esclavo 1...\n");
                
                // Trama FC03 para leer 1 registro (REG_ADC1) con CRC alterado a 0x0000
                uint8_t req_corrupto[8] = {
                    SLAVE_ID1,                    // ID esclavo (1)
                    FC_READ_HOLDING_REGISTERS,    // Función 0x03
                    0x00, 0x00,                   // Registro inicial 0
                    0x00, 0x01,                   // Cantidad 1
                    0x00, 0x00                    // CRC erróneo forzado
                };
                
                modbus_frame_t resp;
                
                // modbus_transaccion envía por RS-485 y espera hasta RESPUESTA_TIMEOUT_MS (500 ms)
                esp_err_t err = modbus_transaccion(req_corrupto, sizeof(req_corrupto), &resp);
                
                // mb_check evalúa el error e imprime el mensaje correspondiente
                if (!mb_check("Lectura con CRC corrupto", SLAVE_ID1, err))
                {
                    printf(">> COMPORTAMIENTO VERIFICADO: El esclavo ignoro la trama y el maestro dio Timeout.\n");
                }
                break;
            }

            case 'o':
            case 'O':
            {
                printf("\n>> [TEST] Enviando lectura con CRC corrupto al Esclavo 2...\n");
                
                uint8_t req_corrupto[8] = {
                    SLAVE_ID2,                    // ID esclavo (2)
                    FC_READ_HOLDING_REGISTERS,    // Función 0x03
                    0x00, 0x00,                   // Registro inicial 0
                    0x00, 0x01,                   // Cantidad 1
                    0x00, 0x00                    // CRC erróneo forzado
                };
                
                modbus_frame_t resp;
                
                // modbus_transaccion envía por RS-485 y espera hasta RESPUESTA_TIMEOUT_MS (500 ms)
                esp_err_t err = modbus_transaccion(req_corrupto, sizeof(req_corrupto), &resp);
                
                // mb_check evalúa el error e imprime el mensaje correspondiente
                if (!mb_check("Lectura con CRC corrupto", SLAVE_ID2, err))
                {
                    printf(">> COMPORTAMIENTO VERIFICADO: El esclavo ignoro la trama y el maestro dio Timeout.\n");
                }
                break;
            }

            case 'm':
            case 'M': break;                    // solo vuelve a mostrar el menú

            default:
                printf(">> Opcion '%c' no valida\n", c);
                break;
        }

        mostrar_menu();                         // el menú siempre queda a la vista
    }
}

// ======================================================
// APP_MAIN
// ======================================================

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " FIRMWARE GRABADO Y EJECUTANDOSE CORRECTAMENTE");
    ESP_LOGI(TAG, " Compilado : %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, " ESP-IDF   : %s", esp_get_idf_version());
    ESP_LOGI(TAG, " Chip      : ESP32, %d nucleo(s), rev %d", chip.cores, chip.revision);
    ESP_LOGI(TAG, " Maestro   : UART 9600 8E1 | Esclavos: ID %d e ID %d", SLAVE_ID1, SLAVE_ID2);
    ESP_LOGI(TAG, "==================================================");

    rs485_init();

    modbus_queue = xQueueCreate(5, sizeof(modbus_frame_t));
    if (modbus_queue == NULL)
    {
        ESP_LOGE(TAG, "Error creando la cola");
        return;
    }

    response_semaphore = xSemaphoreCreateBinary();
    if (response_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Error creando el semaforo");
        return;
    }

    bus_mutex = xSemaphoreCreateMutex();
    if (bus_mutex == NULL)
    {
        ESP_LOGE(TAG, "Error creando el mutex del bus");
        return;
    }

    // Mayor prioridad en la recepción para no perder bytes
    xTaskCreate(uart_rx_task,       "uart_rx_task",       4096, NULL, 6, NULL);
    xTaskCreate(modbus_master_task, "modbus_master_task", 6144, NULL, 5, NULL);
    xTaskCreate(consola_menu_task,  "consola_menu_task",  6144, NULL, 4, NULL);
}