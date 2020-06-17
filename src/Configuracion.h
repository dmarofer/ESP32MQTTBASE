
// Conexiones al ESP32

// SALIDAS
#define PINBOMBA 33
#define PINCARGA 32

// ENTRADAS
#define PINVBAT 35
#define PINVCARGA 34
#define PINFLUJO 19
#define ONEWIREBUS 18
#define PINNIVEL 27
#define PINAMBIENTE 12
#define PINHUMEDAD 13
#define PINLED 5

// CARGA DE LA BATERIA
#define VCARGASTART 12.50           // Voltage para iniciar la carga
#define VCARGASTOP 14.20            // Voltage para parar la carga
#define VBATMINRIEGO 12.50          // Voltage minimo de la bateria para regar
#define VBATEMERGENCIA 12.00        // Tension para hibernacion. -1 o negativos deshabilitada
#define TBATEMERGENCIA 120          // Tiempo en bateria de emergencia para hibernar (segundos)
#define THIBERNADO  60              // Tiempo para despertar y comprobar la bateria (minutos)
                                    // El timer del copro es gigantesto(64 bits). Se puede hibernar 584.942 años (2^64 uS o sea 18 trillones y pico)
#define CARGAMINTEMP 15             // Tiempo minimo de carga para evitar el pico de tension cuando arranca, en minutos


// FLUJO
#define TICKSPORLITRO 270			// Segun lo que leo podria ser esto, ya lo calibraremos a ver ....

// LCD (I2C)
#define I2CSDA 21
#define I2CSCL 22

// Hola local con respecto a UTC
#define HORA_LOCAL 2

// Para el nombre del fichero de configuracion de comunicaciones
#define FICHERO_CONFIG_COM "/RiegaMaticoCom.json"

// Para el nombre del fichero de configuracion del proyecto
#define FICHERO_CONFIG_PRJ "/RiegaMaticoCfg.json"
