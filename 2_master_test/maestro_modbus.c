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
 *   modbus_master_task  : genera solicitudes, espera respuestas,
 *                         valida y detecta errores / timeout
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

// Código propio para "el esclavo respondió con una excepción Modbus"
#define MB_ERR_EXCEPCION    ESP_ERR_INVALID_STATE

// ======================================================
// ESTRUCTURAS Y OBJETOS DE SINCRONIZACIÓN
// ======================================================

// Trama recibida (uart_rx_task -> modbus_master_task)
typedef struct
{
    uint8_t  data[MODBUS_FRAME_SIZE];
    uint16_t length;
} modbus_frame_t;

static QueueHandle_t     modbus_queue;        // cola de tramas recibidas
static SemaphoreHandle_t response_semaphore;  // avisa que llegó una respuesta

// ======================================================
// CRC-16 MODBUS (poly 0xA001, init 0xFFFF)
//
// El CRC se calcula sobre dirección + función + datos y se
// transmite primero el byte BAJO y después el ALTO.
// Quien recibe una trama recalcula el CRC sobre todo menos
// los 2 últimos bytes y lo compara con el recibido (o, lo que
// es equivalente, el CRC de la trama completa da 0).
// Cada trama (pedido y respuesta) lleva su propio CRC.
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
// y después se sigue leyendo con timeout de T3.5: apenas hay
// silencio, la trama está completa y se entrega enseguida.
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
// ======================================================

static esp_err_t modbus_transaccion(const uint8_t *req, int req_len, modbus_frame_t *resp)
{
    // Limpiar restos de respuestas anteriores (tardías, ruido, etc.)
    uart_flush_input(PORT_MAESTRO);
    xQueueReset(modbus_queue);
    xSemaphoreTake(response_semaphore, 0);

    ESP_LOGI(TAG, "TX (%d bytes):", req_len);
    ESP_LOG_BUFFER_HEX(TAG, req, req_len);

    rs485_send(req, req_len);

    // Esperar a que uart_rx_task avise que llegó una trama
    if (xSemaphoreTake(response_semaphore, pdMS_TO_TICKS(RESPUESTA_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (xQueueReceive(modbus_queue, resp, 0) != pdTRUE)
        return ESP_FAIL;

    ESP_LOGI(TAG, "RX (%d bytes):", resp->length);
    ESP_LOG_BUFFER_HEX(TAG, resp->data, resp->length);

    return ESP_OK;
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

// Lee 'qty' registros desde 'start' y los muestra por la UART
static void leer_y_mostrar(uint8_t slave_id, uint16_t start, uint16_t qty)
{
    uint16_t regs[16];
    if (qty > 16) qty = 16;

    if (mb_check("FC03", slave_id, modbus_read_holding_registers(slave_id, start, qty, regs)))
    {
        for (int i = 0; i < qty; i++)
            ESP_LOGI(TAG, "Esclavo %d - %d = %u", slave_id, 40001 + start + i, regs[i]);
    }
}

// ======================================================
// TAREA DEL MAESTRO MODBUS
// ======================================================

void modbus_master_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));        // dar tiempo a que arranquen los esclavos

    // ---------------- Pruebas iniciales (una sola vez) ----------------
    ESP_LOGI(TAG, "=== Prueba 1: leer UN registro (40001) del esclavo 1 ===");
    leer_y_mostrar(SLAVE_ID1, REG_ADC1, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "=== Prueba 2: leer VARIOS registros (40001-40004) del esclavo 1 ===");
    leer_y_mostrar(SLAVE_ID1, REG_ADC1, 4);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "=== Prueba 3: FC16, escribir 40005-40007 (Setpoint, Tiempo, Modo) ===");
    const uint16_t valores[3] = { 100, 60, 2 };
    if (mb_check("FC16", SLAVE_ID1, modbus_write_multiple_registers(SLAVE_ID1, REG_SETPOINT, 3, valores)))
        leer_y_mostrar(SLAVE_ID1, REG_ADC1, 7);     // releer para verificar
    vTaskDelay(pdMS_TO_TICKS(100));

    // ---------------- Ciclo normal ----------------
    uint32_t ciclo = 0;
    uint16_t estado = 0;

    while (1)
    {
        // Prueba 5: consultar alternativamente a los esclavos
        ESP_LOGI(TAG, "--- Consultando esclavo 1 ---");
        leer_y_mostrar(SLAVE_ID1, REG_ADC1, 4);
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "--- Consultando esclavo 2 ---");
        leer_y_mostrar(SLAVE_ID2, REG_ADC1, 4);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Prueba 4: cada 5 ciclos alternar el LED de ambos esclavos (FC06)
        if (++ciclo % 5 == 0)
        {
            estado = !estado;
            ESP_LOGI(TAG, "=== FC06: Estado (40004) = %u en esclavos 1 y 2 ===", estado);
            mb_check("FC06", SLAVE_ID1, modbus_write_single_register(SLAVE_ID1, REG_ESTADO, estado));
            vTaskDelay(pdMS_TO_TICKS(100));
            mb_check("FC06", SLAVE_ID2, modbus_write_single_register(SLAVE_ID2, REG_ESTADO, estado));
        }

        // Ante cualquier error el programa sigue funcionando normalmente
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ======================================================
// APP_MAIN
// ======================================================

void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Iniciando Maestro Modbus RTU");
    ESP_LOGI(TAG, "UART: 9600 8E1 | Esclavos: ID %d e ID %d", SLAVE_ID1, SLAVE_ID2);

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

    // Mayor prioridad en la recepción para no perder bytes
    xTaskCreate(uart_rx_task,       "uart_rx_task",       4096, NULL, 6, NULL);
    xTaskCreate(modbus_master_task, "modbus_master_task", 6144, NULL, 5, NULL);
}
