#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    printf("Inicio de TP3_Tecnicas en ESP32!\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
