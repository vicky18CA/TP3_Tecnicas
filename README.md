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

Para la parte de comunicación física 485 cada esp32 tiene que manejar:
TX
RX
DE
RE
A
B

El ESP32 controla DE/RE para decidir si el MAX485 está transmitiendo o recibiendo.



Para la configuración de la UART, las 3 esp32 deben usar: 
Baud rate = 9600
Data bits = 8
Parity    = Even
Stop bits = 1

Para la marte de MOSBUS, el maestro siempre inicia la comunicación y luego responden los esclavos.
Cada esclavo tiene que tener un mapa de registros.

El mínimo pedido es:

Registro	Contenido
40001	Valor analógico 1 (ADC1)
40002	Valor analógico 2 (ADC2)
40003	Contador
40004	Estado

Aparte el maestro debe poder:
Leer registros
Escribir un registro 
Escritura múltiple 

Un diagrama ideal de lo que sería las tareas para cada uno sería el siguiente: 

                    MAESTRO
                       │
          ┌────────────┼────────────┐
          │            │            │
          ▼            ▼            ▼
    Modbus Task    Console Task   Control Task
          │            │            │
          │            │            │
          └────────────┼────────────┘
                       │
                    RS485 BUS
                       │
             ┌─────────┴─────────┐
             │                   │
             ▼                   ▼
          ESCLAVO 1           ESCLAVO 2
             │                   │
       ┌─────┼─────┐       ┌─────┼─────┐
       │     │     │       │     │     │
      ADC  Modbus  App     ADC  Modbus  App
      Task  Task   Task    Task  Task   Task
       │     │     │       │     │     │
       └─────┴─────┘       └─────┴─────┘


       
las tareas de los esclavos haría lo siguiente:

"Modbus Task": la rama principal de comunicación. Se encarga de esperar solicitud del maestro, recibir la trama, verificar CRC, etc. 

"ADC task": se dedica exclusivamente a adquirir información.
ADC2
 │
 ▼
adc_task
 │
 ▼
registro 40002

"App task": 
        Master
           │
           │ FC06
           ▼
       40004 = 1
           │
           ▼
     Modbus Task
           │
           ▼
      registro[3]
           │
           ▼
    Application Task
           │
           ▼
       GPIO = 1
           │
           ▼
       LED ON


Las tareas del maestro harían: 

"Mosbus task": se encarga de consultar a los esclavos 
        Master
          │
          ▼
     Slave 1
          │
          ▼
     esperar respuesta
          │
          ▼
     Slave 2
          │
          ▼
     esperar respuesta
          │
          ▼
       repetir

  "Console task": podría mostrar los datos por la UART del maestro.

  "Control task": podría encargarse de mandar comandos.
  Tecla / comando
      ↓
control_task
      ↓
FC06
      ↓
Slave 1
      ↓
40004 = 1
      ↓
LED ON

NO OLVIDAR TAMBIEN QUE EL MAESTRO TIENE QUE DETECTAR ERRORES 

