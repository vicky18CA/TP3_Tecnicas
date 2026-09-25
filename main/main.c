#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"  //Libreria maestra FreeRToS. Siempre se agrega para este tipo de So
#include "freertos/task.h"       //Libreria que nos permite gestionar la concurrencia
#include "freertos/queue.h"      //Libreria para utilizar las colas
#include "freertos/semphr.h"     //Libreria para utlizar los semaforos binarios y mutex
#include "driver/gpio.h"         // (General Purpose Input/output). Es la que nos permite configurar y gestionar los pines del ESP32.
#include "esp_adc/adc_oneshot.h" //Libreria que nos permite configurar y controlar los ADC
//#include "esp_driver_dac/dac_oneshot.h"//Libreria que nos permite configurar y controlar los DAC
//#include "driver/i2c.h"          //Libreria que nos permite configurar y controlar el I2C
#include "driver/uart.h"         //Libreria que nos permite configurar la UART (por si se quisiera cambiar la velocidad)
#include "esp_log.h"             //Libreria que fue creada para generar Logs del ESP32.
#include "esp_timer.h"
#include "esp_modbus_master.h"
#include "esp_err.h"

//Definimos el master

#define TAG "MODBUS_MASTER"

//Definición de pines de la UART y RS485

#define PORT_MAESTRO   UART_NUM_1  //acá definimos la UART que usamos
#define MAESTRO_UART_TXD  GPIO_NUM_17
#define MAESTRO_UART_RXD  GPIO_NUM_16
#define MAESTRO_UART_RTS  GPIO_NUM_4  //control de dirección del MAX485

//Definición de la UART

#define BAUD_RATE 9600
#define RX_BUFFER_SIZE 256

// Tamaño máximo de una trama Modbus
#define MODBUS_FRAME_SIZE 256

//Definición de los esclavos

#define SLAVE_ID 1
#define SLAVE_ID 2

//en este caso haremos que la trama de modbus sea para leer los HOlding Registers

#define FC_READ_HOLDING_REGISTERS 0x03 

// Estructura para almacenar una trama recibida
typedef struct
{
    uint8_t data[MODBUS_FRAME_SIZE];
    uint16_t length;

} modbus_frame_t;

// Cola para pasar las tramas desde UART RX hacia la tarea Modbus
static QueueHandle_t modbus_queue;


// Semáforo para avisar que llegó una respuesta
static SemaphoreHandle_t response_semaphore;




//la trama de modbus tiene una parte de CRC que es para ver un bug que se haya realizado en un esclavo, son los últimos 2 bytes. Recordar que si el esclavo me duvuelve los mismos 2 bytes que yo le mandé, eso quiere decir que no hay error, pero si no cioncide (generalmente el esclavo pone el bit más significativo en 1) signofoca que hay un error 

uint16_t modbus_crc16(const uint8_t *data, uint16_t lenght) // recibe un puntero a un arreglo de bytes *data y su longuityd y devuelve el crc calculado
{
    uint16_t crc = 0xFFFF; // inicializamos el registro CRC en todos los bits en 1 

    for (uint16_t pos = 0; pos < lenght; pos++) // recorremos cada byte de la trama
    {
        crc ^= (uint16_t)data[pos]; //tomando el byte actual lo convierte en 16 bits y le hacemos una XOR 

        for (int i = 0; i < 8; i++) //por cada byte hay que procesar sus 8 bits individualmente
         {
            if (crc & 0x0001) //miramos el bit menos significativo y si es uno:
            {
                //
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else //solo desplazamos hacia la derecha 
            {
                crc >>= 1;
            }
        }
    }
    return crc; // y todo esto es para calcular CRC :)

}





/*Acá inicializamos la UART y la RS485*/

void rs485_init(void)
{
    //descripción
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //desactiva el control de flujo por hardware
        .source_clk = UART_SCLK_DEFAULT //para generar el baud rate usa el clk por defecto del sistema
    };
    //instalamos el drivwe UART, esto reserva en RAM loa buffers internos que usa el drivwe de UART del ESP-IDF para RX y TX
    ESP_ERROR_CHECK(
        uart_driver_install(
            PORT_MAESTRO,
            RX_BUFFER_SIZE,
            RX_BUFFER_SIZE,
            0,
            NULL,
            0
        )
    );
//configuracion electrica, se aplica la descripción que hicimos anteriormente
    ESP_ERROR_CHECK(
        uart_param_config(PORT_MAESTRO, &uart_config)
    );
    //asignación de pines
    ESP_ERROR_CHECK(
        uart_set_pin(
            PORT_MAESTRO,
            MAESTRO_UART_TXD,
            MAESTRO_UART_RXD,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    //configuramos los pines a gpio de propósito general para el MAX485

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAESTRO_UART_RTS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Comenzar en modo recepción por defecto
    gpio_set_level(MAESTRO_UART_RTS, 0);
}
//una vez que ya inicializamos todo, ahora vamos a transmitir 

void rs485_send(const uint8_t *data, size_t length)
{
    // Habilitar el transmisor del MAX485
    gpio_set_level(MAESTRO_UART_RTS, 1);

    // Enviar bytes por UART
    uart_write_bytes(
        PORT_MAESTRO,
        (const char *)data,
        length
    );

    // Esperar a que termine la transmisión
    uart_wait_tx_done(
        PORT_MAESTRO,
        pdMS_TO_TICKS(100)
    );

    // Volver a modo recepción
    gpio_set_level(MAESTRO_UART_RTS, 0);
}


//Para la transmisión del RS485 

void rs485_send(const uint8_t *data, size_t length)
{
    // --------------------------------------------------
    // 1. Activar transmisión del MAX485
    // --------------------------------------------------

    gpio_set_level(MAESTRO_UART_RTS, 1);


    // --------------------------------------------------
    // 2. Enviar los bytes
    // --------------------------------------------------

    uart_write_bytes(
        PORT_MAESTRO,
        (const char *)data,
        length
    );


    // --------------------------------------------------
    // 3. Esperar que termine la transmisión
    // --------------------------------------------------

    uart_wait_tx_done(
        PORT_MAESTRO,
        pdMS_TO_TICKS(100)
    );


    // --------------------------------------------------
    // 4. Volver a recepción
    // --------------------------------------------------

    gpio_set_level(MAESTRO_UART_RTS, 0);
}

/*Para agregar más cosas al TP, voy a agregar una tarea para la recepción de UART, como hizo Gonza*/

void recepcion_UART(void *pvParameters)
{
    uint8_t rx_buffer[MODBUS_FRAME_SIZE];

    while (1)
    {
        // --------------------------------------------------
        // Indicamos la cantidad de bytes que fueros recibidos
        // --------------------------------------------------

        int len = uart_read_bytes(
            PORT_MAESTRO,
            rx_buffer, //dónde se van a guardar los bytes recibidos
            sizeof(rx_buffer), //leemos como máximo 256 bytes
            pdMS_TO_TICKS(500) //esperamos 500ms por dato
        );


        // --------------------------------------------------
        // Verificar si recibimos datos
        // --------------------------------------------------

        if (len > 0)
        {
            modbus_frame_t frame; //creamos una variable para guardar cuántos datos fueron recibidos y cuántos bytes

            // Limitar tamaño por seguridad
            if (len > MODBUS_FRAME_SIZE)
            {
                len = MODBUS_FRAME_SIZE;
            }


            // --------------------------------------------------
            // Copiar los datos recibidos a la estructura
            // --------------------------------------------------

            memcpy(
                frame.data, //destino
                rx_buffer, //origen
                len //cantidad
            );

            frame.length = len; //guardamos la cantidad de bytes


            // --------------------------------------------------
            // Enviar la trama a la cola 
            // --------------------------------------------------

            if (xQueueSend(
                    modbus_queue,
                    &frame, //toma el contenido de frame y lo mete en la cola
                    pdMS_TO_TICKS(100)
                ) == pdTRUE)

            //todo lo que esta en frame se pasa a la cola
            {
                ESP_LOGI(
                    TAG,
                    "Trama recibida y enviada a la cola"
                );


                // --------------------------------------------------
                // Avisar mediante el semáforo que llegó una respuesta
                // --------------------------------------------------

                xSemaphoreGive(response_semaphore);
            }
            //en caso de que la cola este llena
            else
            {
                ESP_LOGW(
                    TAG,
                    "No se pudo enviar la trama a la cola"
                );
            }
        }
    }
}


// ======================================================
// FC03 - READ HOLDING REGISTERS
// ======================================================

void modbus_read_holding_registers(
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity
)





void app_main(void)
{
    printf("Inicio de TP3_Tecnicas en ESP32!\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
