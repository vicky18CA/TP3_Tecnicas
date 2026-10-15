/* ======================================================================
 * TP3 - Técnicas Digitales III
 * ESCLAVO 2 Modbus RTU sobre RS485 (ESP-IDF + FreeRTOS)
 *
 * Funciones soportadas:
 *   0x03  Read Holding Registers
 *   0x06  Write Single Holding Register
 *   0x10  Write Multiple Holding Registers (FC16)
 *
 * Periféricos asignados:
 *   - Potenciómetro : GPIO34 (ADC1 Canal 6) -> Registro 40001 (ADC1)
 *   - Pulsador      : GPIO18 (Pull-up interno a GND) -> Registro 40002 (Estado botón: 0 suelto, 1 presionado)
 *   - LED           : GPIO2 -> Registro 40004 (Estado: 0 apagado, 1 destello/encendido)
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

#define TAG "MODBUS_SLAVE_2"

// ======================================================
// CONFIGURACIÓN DE HARDWARE Y PROTOCOLO
// ======================================================

#define SLAVE_ADDR          2              // Dirección Modbus de este esclavo

// UART / RS485
#define PORT_ESCLAVO        UART_NUM_1
#define ESCLAVO_UART_TXD    GPIO_NUM_17    // -> DI del MAX485
#define ESCLAVO_UART_RXD    GPIO_NUM_16    // <- RO del MAX485
#define ESCLAVO_UART_RTS    GPIO_NUM_4     // -> DE y RE puenteados del MAX485

#define BAUD_RATE           9600           // 9600, 8E1
#define RX_BUFFER_SIZE      256
#define MODBUS_FRAME_SIZE   256

#define T35_MS              5              // Silencio T3.5 a 9600 baudios (~4.01 ms)

// Entradas / Salidas
#define LED_GPIO            GPIO_NUM_2
#define BOTON_GPIO          GPIO_NUM_18    // Pulsador conectado entre GPIO18 y GND
#define ADC1_CANAL          ADC_CHANNEL_6  // GPIO34 -> Potenciómetro
#define ADC_MUESTRAS        16

// Códigos de función Modbus
#define FC_READ_HOLDING_REGISTERS    0x03
#define FC_WRITE_SINGLE_REGISTER     0x06
#define FC_WRITE_MULTIPLE_REGISTERS  0x10

// Códigos de excepción
#define EX_ILLEGAL_FUNCTION   0x01
#define EX_ILLEGAL_ADDRESS    0x02
#define EX_ILLEGAL_VALUE      0x03

// ======================================================
// MAPA DE REGISTROS (Dirección PDU base 0)
// ======================================================

#define REG_ADC1        0   // 40001 Potenciómetro (0 - 4095)
#define REG_BOTON       1   // 40002 Pulsador (0 = libre, 1 = presionado)
#define REG_CONTADOR    2   // 40003 Contador incremental de segundos
#define REG_ESTADO      3   // 40004 Estado de salida (0 = OFF, 1 = ON)
#define REG_SETPOINT    4   // 40005 Setpoint
#define REG_TIEMPO      5   // 40006 Tiempo de parpadeo en ms
#define REG_MODO        6   // 40007 Modo operación
#define REG_ERRORES     7   // 40008 Contador de tramas descartadas por CRC
#define NUM_REGISTROS   8

static uint16_t registros[NUM_REGISTROS];
static SemaphoreHandle_t registros_mutex;
static TaskHandle_t aplicacion_handle;

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
// CRC-16 MODBUS (Polinomio 0xA001, Init 0xFFFF)
// ======================================================

uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static int agregar_crc(uint8_t *trama, int n)
{
    uint16_t crc = modbus_crc16(trama, n);
    trama[n]     = crc & 0xFF;         // Byte bajo primero
    trama[n + 1] = (crc >> 8) & 0xFF;  // Byte alto después
    return n + 2;
}

// ======================================================
// COMUNICACIÓN FÍSICA RS485
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
    gpio_set_level(ESCLAVO_UART_RTS, 0); // Modo escucha por defecto
}

void rs485_send(const uint8_t *data, size_t length)
{
    gpio_set_level(ESCLAVO_UART_RTS, 1);
    uart_write_bytes(PORT_ESCLAVO, (const char *)data, length);
    uart_wait_tx_done(PORT_ESCLAVO, pdMS_TO_TICKS(100));
    gpio_set_level(ESCLAVO_UART_RTS, 0);
}

// ======================================================
// PROCESAMIENTO DE TRAMAS MODBUS
// ======================================================

static int armar_excepcion(uint8_t fc, uint8_t codigo, uint8_t *tx)
{
    tx[0] = SLAVE_ADDR;
    tx[1] = fc | 0x80;
    tx[2] = codigo;
    return agregar_crc(tx, 3);
}

static int procesar_trama(const uint8_t *rx, int len, uint8_t *tx)
{
    if (len < 4) return 0;

    // Validación de CRC explícita sobre los datos excluyendo los 2 bytes de checksum
    uint16_t crc_calculado = modbus_crc16(rx, len - 2);
    uint16_t crc_recibido  = rx[len - 2] | (rx[len - 1] << 8);

    if (crc_calculado != crc_recibido)
    {
        ESP_LOGW(TAG, "CRC incorrecto -> trama descartada (sin respuesta)");
        reg_set(REG_ERRORES, reg_get(REG_ERRORES) + 1);
        return 0;
    }

    uint8_t addr = rx[0];
    if (addr != SLAVE_ADDR && addr != 0) return 0; // Trama para otro nodo o broadcast

    uint8_t fc = rx[1];
    int n = 0;

    switch (fc)
    {
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
            tx[2] = cantidad * 2;

            xSemaphoreTake(registros_mutex, portMAX_DELAY);
            for (int i = 0; i < cantidad; i++)
            {
                tx[3 + 2 * i] = registros[inicio + i] >> 8;
                tx[4 + 2 * i] = registros[inicio + i] & 0xFF;
            }
            xSemaphoreGive(registros_mutex);

            n = agregar_crc(tx, 3 + cantidad * 2);
            break;
        }

        case FC_WRITE_SINGLE_REGISTER:
        {
            if (len != 8) { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            uint16_t reg   = ((uint16_t)rx[2] << 8) | rx[3];
            uint16_t valor = ((uint16_t)rx[4] << 8) | rx[5];

            if (reg >= NUM_REGISTROS)
            { n = armar_excepcion(fc, EX_ILLEGAL_ADDRESS, tx); break; }

            if (reg == REG_ESTADO && valor > 1)
            { n = armar_excepcion(fc, EX_ILLEGAL_VALUE, tx); break; }

            reg_set(reg, valor);

            if (reg == REG_ESTADO || reg == REG_TIEMPO) xTaskNotifyGive(aplicacion_handle);

            memcpy(tx, rx, 6);
            n = agregar_crc(tx, 6);
            break;
        }

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

            memcpy(tx, rx, 6);
            n = agregar_crc(tx, 6);
            break;
        }

        default:
            n = armar_excepcion(fc, EX_ILLEGAL_FUNCTION, tx);
            break;
    }

fin:
    return (addr == 0) ? 0 : n;
}

// ======================================================
// TAREA RECEPTORA MODBUS
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

        if (uart_read_bytes(PORT_ESCLAVO, &b, 1, portMAX_DELAY) != 1) continue;
        rx[len++] = b;

        while (uart_read_bytes(PORT_ESCLAVO, &b, 1, t35) == 1)
        {
            if (len < MODBUS_FRAME_SIZE) rx[len++] = b;
            else desbordado = true;
        }

        if (desbordado) continue;

        int n = procesar_trama(rx, len, tx);
        if (n > 0)
        {
            rs485_send(tx, n);
        }
    }
}

// ======================================================
// TAREA DE ADQUISICIÓN: ADC + PULSADOR
// ======================================================

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
    // Configuración ADC para potenciómetro
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC1_CANAL, &chan_cfg));

    // Configuración GPIO para pulsador (Pull-up interno)
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BOTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_conf));

    uint32_t ciclos = 0;

    while (1)
    {
        // 1) Leer potenciómetro -> 40001
        uint16_t v1 = adc_leer_promedio(adc, ADC1_CANAL);
        reg_set(REG_ADC1, v1);

        // 2) Leer pulsador (lógica activa en bajo por el pull-up) -> 40002
        // Si el botón está presionado conecta a GND (lectura 0), se guarda 1 en el registro
        uint16_t estado_boton = (gpio_get_level(BOTON_GPIO) == 0) ? 1 : 0;
        reg_set(REG_BOTON, estado_boton);

        // 3) Contador cada 1 segundo (5 ciclos x 200 ms) -> 40003
        if (++ciclos % 5 == 0)
            reg_set(REG_CONTADOR, reg_get(REG_CONTADOR) + 1);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ======================================================
// TAREA DE APLICACIÓN: Control del LED
// ======================================================

void aplicacion_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    bool led = false;

    while (1)
    {
        uint16_t estado = reg_get(REG_ESTADO);
        uint16_t periodo_ms = reg_get(REG_TIEMPO);

        if (periodo_ms < 50) periodo_ms = 500;

        TickType_t espera = (estado != 0) ? pdMS_TO_TICKS(periodo_ms) : portMAX_DELAY;

        if (ulTaskNotifyTake(pdTRUE, espera) > 0)
        {
            estado = reg_get(REG_ESTADO);
            led = (estado != 0);
            gpio_set_level(LED_GPIO, led);
        }
        else
        {
            led = !led;
            gpio_set_level(LED_GPIO, led);
        }
    }
}

// ======================================================
// PUNTO DE ENTRADA
// ======================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Firmware Esclavo 2");
    ESP_LOGI(TAG, "Direccion Modbus : %d", SLAVE_ADDR);
    ESP_LOGI(TAG, "UART: 9600 8E1");

    registros_mutex = xSemaphoreCreateMutex();
    if (registros_mutex == NULL) return;

    memset(registros, 0, sizeof(registros));
    rs485_init();

    xTaskCreate(aplicacion_task,   "aplicacion_task",   3072, NULL, 5,  &aplicacion_handle);
    xTaskCreate(adquisicion_task, "adquisicion_task", 3072, NULL, 5,  NULL);
    xTaskCreate(modbus_slave_task,"modbus_slave_task",4096, NULL, 10, NULL);
}