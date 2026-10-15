/* ======================================================================
 * TP3 - Técnicas Digitales III
 * ESCLAVO Modbus RTU sobre RS485  (ESP-IDF + FreeRTOS, sin Arduino)
 *
 * Funciones soportadas:
 *   0x03  Read Holding Registers
 *   0x06  Write Single Holding Register
 *   0x10  Write Multiple Holding Registers (FC16)
 *
 * Tareas FreeRTOS:
 *   modbus_slave_task : recibe tramas, valida CRC/dirección y responde
 *   adquisicion_task  : ADC1 (potenciómetro), ADC2 y contador -> mapa de registros
 *   aplicacion_task   : registro Estado (40004) -> LED
 *
 * POTENCIÓMETRO: extremos a 3.3 V y GND, cursor (pin central) a GPIO34.
 * Su valor (0-4095) queda en el registro 40001 (ADC1).
 *
 * IMPORTANTE: cambiar SLAVE_ADDR (1 o 2) antes de flashear cada esclavo.
 * ====================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_chip_info.h"

#define TAG "MODBUS_SLAVE"

// ======================================================
// CONFIGURACIÓN
// ======================================================

#define SLAVE_ADDR          1            // <<< CAMBIAR: 1 = Esclavo 1, 2 = Esclavo 2

// UART / RS485 (mismos pines que el Maestro)
#define PORT_ESCLAVO        UART_NUM_1
#define ESCLAVO_UART_TXD    GPIO_NUM_17  // -> DI del MAX485
#define ESCLAVO_UART_RXD    GPIO_NUM_16  // <- RO del MAX485
#define ESCLAVO_UART_RTS    GPIO_NUM_4   // -> DE y RE (puenteados) del MAX485

#define BAUD_RATE           9600         // 9600, 8E1
#define RX_BUFFER_SIZE      256
#define MODBUS_FRAME_SIZE   256

// Silencio que marca el fin de trama: T3.5 = 3.5 * 11 bits / 9600 = 4.01 ms
// Con tick de 10 ms (CONFIG_FREERTOS_HZ=100) se redondea a 1 tick (10 ms): sigue funcionando.
#define T35_MS              5

// Periféricos
#define LED_GPIO            GPIO_NUM_2
#define ADC1_CANAL          ADC_CHANNEL_6   // GPIO34  <- POTENCIÓMETRO (registro 40001)
#define ADC2_CANAL          ADC_CHANNEL_7   // GPIO35  (registro 40002)

// Promedio de muestras para estabilizar la lectura del potenciómetro
#define ADC_MUESTRAS        16

// Códigos de función
#define FC_READ_HOLDING_REGISTERS    0x03
#define FC_WRITE_SINGLE_REGISTER     0x06
#define FC_WRITE_MULTIPLE_REGISTERS  0x10

// Códigos de excepción
#define EX_ILLEGAL_FUNCTION   0x01
#define EX_ILLEGAL_ADDRESS    0x02
#define EX_ILLEGAL_VALUE      0x03

// ======================================================
// MAPA DE REGISTROS (dirección PDU base 0: 40001 -> 0)
// ======================================================

#define REG_ADC1        0   // 40001 Valor analógico 1 (potenciómetro en GPIO34)
#define REG_ADC2        1   // 40002 Valor analógico 2
#define REG_CONTADOR    2   // 40003 Contador (+1 por segundo)
#define REG_ESTADO      3   // 40004 Estado: 0 = OFF, 1 = ON (controla el LED)
#define REG_SETPOINT    4   // 40005 Setpoint        (FC16)
#define REG_TIEMPO      5   // 40006 Tiempo          (FC16)
#define REG_MODO        6   // 40007 Modo operación  (FC16)
#define REG_ERRORES     7   // 40008 Contador de errores de CRC (opcional)
#define NUM_REGISTROS   8

static uint16_t registros[NUM_REGISTROS];
static SemaphoreHandle_t registros_mutex;   // protege el mapa entre tareas
static TaskHandle_t aplicacion_handle;      // para avisar cambios de Estado

static uint16_t reg_get(uint16_t idx)
{
    uint16_t v;
    xSemaphoreTake(registros_mutex, portMAX_DELAY);
    v = registros[idx];
    xSemaphoreGive(registros_mutex);
    return v;
}

static void reg_set(uint16_t idx, uint16_t valor)
{
    xSemaphoreTake(registros_mutex, portMAX_DELAY);
    registros[idx] = valor;
    xSemaphoreGive(registros_mutex);
}

// ======================================================
// CRC-16 MODBUS (poly 0xA001, init 0xFFFF)
// Se transmite primero el byte BAJO y después el ALTO.
// ======================================================

uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

// Agrega el CRC al final de la trama y devuelve la nueva longitud
static int agregar_crc(uint8_t *trama, int n)
{
    uint16_t crc = modbus_crc16(trama, n);
    trama[n]     = crc & 0xFF;
    trama[n + 1] = (crc >> 8) & 0xFF;
    return n + 2;
}

// ======================================================
// UART + RS485 (DE/RE controlado por GPIO)
// ======================================================

void rs485_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,          // 8E1
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(PORT_ESCLAVO, RX_BUFFER_SIZE, RX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT_ESCLAVO, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PORT_ESCLAVO, ESCLAVO_UART_TXD, ESCLAVO_UART_RXD,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ESCLAVO_UART_RTS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // El esclavo arranca SIEMPRE en recepción (nunca transmite espontáneamente)
    gpio_set_level(ESCLAVO_UART_RTS, 0);
}

void rs485_send(const uint8_t *data, size_t length)
{
    gpio_set_level(ESCLAVO_UART_RTS, 1);                         // 1. modo transmisión
    uart_write_bytes(PORT_ESCLAVO, (const char *)data, length);  // 2. enviar
    uart_wait_tx_done(PORT_ESCLAVO, pdMS_TO_TICKS(100));         // 3. esperar fin de TX
    gpio_set_level(ESCLAVO_UART_RTS, 0);                         // 4. volver a recepción
}

// ======================================================
// PROCESAMIENTO DE TRAMAS
// ======================================================

// Respuesta de excepción: dirección + (función | 0x80) + código + CRC
static int armar_excepcion(uint8_t fc, uint8_t codigo, uint8_t *tx)
{
    tx[0] = SLAVE_ADDR;
    tx[1] = fc | 0x80;
    tx[2] = codigo;
    return agregar_crc(tx, 3);
}

// Devuelve la longitud de la respuesta (0 = no responder)
static int procesar_trama(const uint8_t *rx, int len, uint8_t *tx)
{
    if (len < 4) return 0;                       // dirección + función + CRC

    // CRC calculado sobre toda la trama (con su CRC incluido) debe dar 0
    if (modbus_crc16(rx, len) != 0)
    {
        ESP_LOGW(TAG, "CRC incorrecto -> trama descartada (sin respuesta)");
        reg_set(REG_ERRORES, reg_get(REG_ERRORES) + 1);
        return 0;
    }

    uint8_t addr = rx[0];
    if (addr != SLAVE_ADDR && addr != 0) return 0;   // no es para mí (0 = broadcast)

    uint8_t fc = rx[1];
    int n = 0;

    switch (fc)
    {
        // ---------------- FC03: Read Holding Registers ----------------
        // Solicitud: ADDR FC START(2) QTY(2) CRC(2)
        case FC_READ_HOLDING_REGISTERS:
        {
            if (len != 8) { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            uint16_t inicio   = ((uint16_t)rx[2] << 8) | rx[3];
            uint16_t cantidad = ((uint16_t)rx[4] << 8) | rx[5];

            if (cantidad < 1 || cantidad > 125)
            { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            if (inicio + cantidad > NUM_REGISTROS)
            { n = armar_excepcion(fc, EX_ILLEGAL_ADDRESS, tx); break; }

            tx[0] = SLAVE_ADDR;
            tx[1] = fc;
            tx[2] = cantidad * 2;                        // bytes de datos

            xSemaphoreTake(registros_mutex, portMAX_DELAY);
            for (int i = 0; i < cantidad; i++)
            {
                tx[3 + 2 * i] = registros[inicio + i] >> 8;     // byte alto primero
                tx[4 + 2 * i] = registros[inicio + i] & 0xFF;
            }
            xSemaphoreGive(registros_mutex);

            n = agregar_crc(tx, 3 + cantidad * 2);
            break;
        }

        // ---------------- FC06: Write Single Holding Register ----------------
        // Solicitud: ADDR FC REG(2) VALOR(2) CRC(2)  -> la respuesta es un eco
        case FC_WRITE_SINGLE_REGISTER:
        {
            if (len != 8) { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            uint16_t reg   = ((uint16_t)rx[2] << 8) | rx[3];
            uint16_t valor = ((uint16_t)rx[4] << 8) | rx[5];

            if (reg >= NUM_REGISTROS)
            { n = armar_excepcion(fc, EX_ILLEGAL_ADDRESS, tx); break; }

            if (reg == REG_ESTADO && valor > 1)          // Estado solo admite 0 o 1
            { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            reg_set(reg, valor);

            if (reg == REG_ESTADO || reg == REG_TIEMPO) xTaskNotifyGive(aplicacion_handle);

            memcpy(tx, rx, 6);
            n = agregar_crc(tx, 6);
            break;
        }

        // ---------------- FC16: Write Multiple Holding Registers ----------------
        // Solicitud: ADDR FC START(2) QTY(2) BYTECOUNT DATOS... CRC(2)
        // Respuesta: ADDR FC START(2) QTY(2) CRC(2)
        case FC_WRITE_MULTIPLE_REGISTERS:
        {
            if (len < 9) { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            uint16_t inicio   = ((uint16_t)rx[2] << 8) | rx[3];
            uint16_t cantidad = ((uint16_t)rx[4] << 8) | rx[5];
            uint8_t  bytes    = rx[6];

            if (cantidad < 1 || cantidad > 123 || bytes != cantidad * 2 || len != 9 + bytes)
            { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            if (inicio + cantidad > NUM_REGISTROS)
            { n = armar_excepcion(fc, EX_ILLEGAL_ADDRESS, tx); break; }

            // Validar todo antes de escribir (todo o nada)
            for (int i = 0; i < cantidad; i++)
            {
                uint16_t v = ((uint16_t)rx[7 + 2 * i] << 8) | rx[8 + 2 * i];
                if ((inicio + i) == REG_ESTADO && v > 1)
                { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); goto fin; }
            }

            xSemaphoreTake(registros_mutex, portMAX_DELAY);
            for (int i = 0; i < cantidad; i++)
                registros[inicio + i] = ((uint16_t)rx[7 + 2 * i] << 8) | rx[8 + 2 * i];
            xSemaphoreGive(registros_mutex);

            if (REG_ESTADO >= inicio && REG_ESTADO < inicio + cantidad)
                xTaskNotifyGive(aplicacion_handle);

            memcpy(tx, rx, 6);                           // ADDR FC START QTY
            n = agregar_crc(tx, 6);
            break;
        }

        default:
            n = armar_excepcion(fc, EX_ILLEGAL_FUNCTION, tx);
            break;
    }

fin:
    // Broadcast (dirección 0): se ejecuta pero NUNCA se responde
    return (addr == 0) ? 0 : n;
}

// ======================================================
// TAREA MODBUS ESCLAVO (comunicación)
// ======================================================

void modbus_slave_task(void *pvParameters)
{
    static uint8_t rx[MODBUS_FRAME_SIZE];
    static uint8_t tx[MODBUS_FRAME_SIZE];

    TickType_t t35 = pdMS_TO_TICKS(T35_MS);
    if (t35 < 1) t35 = 1;

    while (1)
    {
        uint8_t b;
        int len = 0;
        bool desbordado = false;

        // 1) Esperar el primer byte de una trama (bloqueante)
        if (uart_read_bytes(PORT_ESCLAVO, &b, 1, portMAX_DELAY) != 1) continue;
        rx[len++] = b;

        // 2) Leer hasta que haya un silencio >= T3.5 (fin de trama)
        while (uart_read_bytes(PORT_ESCLAVO, &b, 1, t35) == 1)
        {
            if (len < MODBUS_FRAME_SIZE) rx[len++] = b;
            else desbordado = true;
        }

        if (desbordado)
        {
            ESP_LOGW(TAG, "Trama demasiado larga -> descartada");
            continue;
        }

        ESP_LOGI(TAG, "RX (%d bytes):", len);
        ESP_LOG_BUFFER_HEX(TAG, rx, len);

        // 3) Procesar y responder
        int n = procesar_trama(rx, len, tx);

        if (n > 0)
        {
            ESP_LOGI(TAG, "TX (%d bytes):", n);
            ESP_LOG_BUFFER_HEX(TAG, tx, n);
            rs485_send(tx, n);
        }
    }
}

// ======================================================
// TAREA DE ADQUISICIÓN: ADC -> mapa de registros
// ======================================================

// Promedia varias lecturas de un canal para filtrar el ruido del ADC
static uint16_t adc_leer_promedio(adc_oneshot_unit_handle_t adc, adc_channel_t canal)
{
    uint32_t suma = 0;
    int validas = 0;

    for (int i = 0; i < ADC_MUESTRAS; i++)
    {
        int v = 0;
        if (adc_oneshot_read(adc, canal, &v) == ESP_OK)
        {
            suma += (uint32_t)v;
            validas++;
        }
    }
    return (validas > 0) ? (uint16_t)(suma / validas) : 0;
}

void adquisicion_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,        // ~0 a 3.3 V
        .bitwidth = ADC_BITWIDTH_12         // 0 a 4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC1_CANAL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC2_CANAL, &chan_cfg));

    uint32_t ciclos = 0;

    while (1)
    {
        // ADC1 = potenciómetro (GPIO34) -> registro 40001
        uint16_t v1 = adc_leer_promedio(adc, ADC1_CANAL);
        uint16_t v2 = adc_leer_promedio(adc, ADC2_CANAL);

        reg_set(REG_ADC1, v1);
        reg_set(REG_ADC2, v2);

        if (++ciclos % 5 == 0)              // 5 ciclos x 200 ms = 1 s
            reg_set(REG_CONTADOR, reg_get(REG_CONTADOR) + 1);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ======================================================
// TAREA DE APLICACIÓN: registro Estado -> LED
// ======================================================

void aplicacion_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    bool led = false;

    while (1)
    {
        uint16_t estado = reg_get(REG_ESTADO);      // 40004: 0 = apagado, 1 = parpadeando
        uint16_t periodo_ms = reg_get(REG_TIEMPO);  // 40006: período en ms

        // Valor por defecto o seguridad mínima para que no sature la CPU
        if (periodo_ms < 50) periodo_ms = 500;

        TickType_t espera = (estado != 0) ? pdMS_TO_TICKS(periodo_ms) : portMAX_DELAY;

        if (ulTaskNotifyTake(pdTRUE, espera) > 0)
        {
            // El Maestro escribió un registro (se actualizó Estado, Tiempo o Modo)
            estado = reg_get(REG_ESTADO);
            led = (estado != 0);
            gpio_set_level(LED_GPIO, led);
        }
        else
        {
            // Timeout: alternar el LED con el período configurado
            led = !led;
            gpio_set_level(LED_GPIO, led);
        }
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
    //ESP_LOGI(TAG, " Compilado : %s %s", _DATE, __TIME_);
    ESP_LOGI(TAG, " ESP-IDF   : %s", esp_get_idf_version());
    ESP_LOGI(TAG, " Chip      : ESP32, %d nucleo(s), rev %d", chip.cores, chip.revision);
    ESP_LOGI(TAG, " Esclavo   : direccion %d | UART 9600 8E1", SLAVE_ADDR);
    ESP_LOGI(TAG, " Potenciometro: GPIO34 (ADC1_CH6) -> registro 40001");
    ESP_LOGI(TAG, "==================================================");

    registros_mutex = xSemaphoreCreateMutex();
    if (registros_mutex == NULL)
    {
        ESP_LOGE(TAG, "Error creando el mutex");
        return;
    }

    memset(registros, 0, sizeof(registros));
    rs485_init();

    // Primero la tarea de aplicación: su handle lo usa la tarea Modbus
    xTaskCreate(aplicacion_task,   "aplicacion_task",   3072, NULL, 5,  &aplicacion_handle);
    xTaskCreate(adquisicion_task,  "adquisicion_task",  3072, NULL, 5,  NULL);
    xTaskCreate(modbus_slave_task, "modbus_slave_task", 4096, NULL, 10, NULL);
}