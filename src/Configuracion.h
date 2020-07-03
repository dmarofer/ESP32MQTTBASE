
// Conexiones al ESP32

// SALIDAS
#define PINBOMBA 33
#define PINCARGA 32

// ENTRADAS
#define PINVBAT 35
#define PINVCARGA 34
#define PINFLUJO 13
#define ONEWIREBUS 18
#define PINNIVEL 27
#define PINAMBIENTE 12
//#define PINHUMEDAD 19
#define PINLED 5

// CARGA DE LA BATERIA
#define VCARGASTART 12.50           // Voltage para iniciar la carga
#define VCARGASTOP 14.50            // Voltage para parar la carga
#define VBATMINRIEGO 12.50          // Voltage minimo de la bateria para regar
#define VBATEMERGENCIA 12.00        // Tension para hibernacion. -1 o negativos deshabilitada
#define TBATEMERGENCIA 1200         // Tiempo en bateria de emergencia para hibernar (segundos)
#define THIBERNADO  60              // Tiempo para despertar y comprobar la bateria (minutos)
                                    // El timer del copro es gigantesto(64 bits). Se puede hibernar 584.942 años (2^64 uS o sea 18 trillones y pico)
#define CARGAMINTEMP 5              // Tiempo minimo de carga para evitar el pico de tension cuando arranca la carga si la bateria esta un poco KO, en minutos


// FLUJO
// Ahora que empieza a funcionar bien al cambiar al pin 13 en vez del 19 unas notas.
// Con el ticks por litro a 270 me da un flujo de unos 440ml/sec con la bomba a 250
// Hay que calcular un poco para afinar esto, ese numero es demasiado alto.
// Un poco a ojo de momento. Calculo que puede echar un litro en 20 segundos, y a lo mejor es menos. Eso da un flujo de unos 50ml/sec
// O sea 

#define TICKSPORLITRO 2500

#define FLUJOMIN 200                // Flujo minimo (ms / sec) "normal"
#define FLUJOMAX 1000               // Flujo maximo (ms / sec) "normal"

// LCD (I2C)
#define I2CSDA 21
#define I2CSCL 22

// Hola local con respecto a UTC
#define HORA_LOCAL 2

// Para el nombre del fichero de configuracion de comunicaciones
#define FICHERO_CONFIG_COM "/RiegaMaticoCom.json"

// Para el nombre del fichero de configuracion del proyecto
#define FICHERO_CONFIG_PRJ "/RiegaMaticoCfg.json"
