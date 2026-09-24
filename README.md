# TP3_Tecnicas
Lo que tenemos que hacer en este trabajo tiene la siguiente arquitectura:
                       BUS RS485
              ┌─────────────────────────┐
              │                         │
PC             │       ESP32             │
│              │      MAESTRO            │
│              │          │              │
USB-RS485 ─────┴──────────┼──────────────┤
                           │
                     ┌─────┴─────┐
                     │           │
                  ESP32        ESP32
                ESCLAVO 1    ESCLAVO 2

cabe aclarar que cada esp32 lleva un MAX485, para ver cómo es la conexión se recomienda el siguiente video: "https://www.youtube.com/watch?v=kXjPUu9jZOw&list=WL&index=89&t=1s"

Presentado como un mapa de memoria, las "tareas" que tenemos que realizar son las siguientes: 

┌─────────────────────────────┐
│       Aplicación            │
│ ADC / LED / contador / etc. │
├─────────────────────────────┤
│        Modbus RTU            │
│ FC03 / FC06 / FC16 / CRC     │
├─────────────────────────────┤
│           UART              │
│       9600 8E1              │
├─────────────────────────────┤
│           RS485             │
│        MAX485 + bus         │
└─────────────────────────────┘


