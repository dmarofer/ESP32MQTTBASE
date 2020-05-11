# RIEGAMATICO MQTT 1.0

# Control de riego con ESP32 y capacidades MQTT desarrollado para HomeAssistant

Desarrollado con Visual Code + PlatformIO + Plataforma Espressif 32 Arduino

Implementa las comunicaciones WIFI + MQTT asi como la configuracion de las mismas via comandos

Implementa el envio de comandos via puerto serie o MQTT

Implementa el uso de tareas para multiproceso y para usar ambos cores

Author: Diego Maroto - BilbaoMakers 2019 - info@bilbaomakers.org - dmarofer@diegomaroto.net

https://github.com/dmarofer/RIEGAMATICO_MQTT

https://bilbaomakers.org/

Licencia: GNU General Public License v3.0 ( mas info en GitHub )


## NOTAS DEL BRANCH

- Afinal el flujo e implementar riego por volumen

- Con Aire me suele dar entre 450 y 1000 ml/s
- Con agua suele andar por 40 - 50 ml/s

O sea si el fujo esta por encima de 100 es Aire. Cuando esta por debajo de 100 parece que mide bastante bien.