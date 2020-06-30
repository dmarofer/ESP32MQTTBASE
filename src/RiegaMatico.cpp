#include <RiegaMatico.h>
#include <Arduino.h>
#include "driver/adc.h"
#include <Configuracion.h>
#include <ArduinoJson.h>				// OJO: Version 5. La 6 es totalmente distinta
#include <SPIFFS.h>						// Libreria para sistema de ficheros SPIFFS
#include <WiFi.h>						// Para las comunicaciones WIFI del ESP32
#include <NTPClient.h>					// Para la gestion de la hora por NTP
#include <JLed.h>
#include <esp32-hal.h>
#include <vector>
#include <DHTesp.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Led que se maneja con la libreria JLed (va un poco regular en ESP32, hay cosas tiempos y eso que no van bien)
// Iniciar el led en modo Off
auto LedEstado = JLed(PINLED).Off();

// Para el DHT11 o 22 de ambiente
DHTesp SensorAmbiente;
TempAndHumidity LecturaAmbiente;

// LCD
LiquidCrystal_I2C lcd(0x27,16,2);

// Bus Onewire para Temperatura Tierra DS18B20
OneWire MiOneWireBus(ONEWIREBUS);  // (a 4.7K resistor is necessary). Bueno ya veremos se puede pufear con el pullup .....

// Pasamos el bus al objeto DallasTemperature
DallasTemperature MisTermometrosOnewire(&MiOneWireBus);

// Array con las direcciones de cada uno de los termometros del bus Onewire
DeviceAddress TempMaceta1, TempMaceta2;



// Constructor. Lo que sea que haya que hacer antes de devolver el objeto de esta clase al creador.
RiegaMatico::RiegaMatico(String fich_config_RiegaMatico, NTPClient& ClienteNTP) : ClienteNTP(ClienteNTP) {	

	// Apuntar la instancia creada a sRiegaMatico (de momento para lo de la interrupcion)
	sRiegaMatico = this;
	
	HardwareInfo = "RiegaMatico.ESP32.1.0";
	ComOK = false;
	mificheroconfig = fich_config_RiegaMatico;

	riegoerror = false;												// quitar cuando este implementado cargar desde el fichero de configuracion
    
	// Habilitar un generador PWM
	ledcSetup(1,2000,8);
	// Y asignarlo a un pin
	ledcAttachPin(PINBOMBA,1);
	// Y poner a cero
	ledcWrite(1,0);
	
	// Salida para el rele de carga
	pinMode(PINCARGA,OUTPUT);
	digitalWrite(PINCARGA,LOW);
	
	// Sensor de Nivel del deposito (reserva)
	pinMode(PINNIVEL,INPUT_PULLDOWN);

	// Conversores ADC	
	adc1_config_width(ADC_WIDTH_BIT_12);
	// Atenuacion 11dB es maxima y permite leer 11dB attenuation (ADC_ATTEN_DB_11) between 150 to 2450mV
	// ADC1 channel 7 is GPIO35 - BATERIA
	// ADC1 channel 6 is GPIO34 - CARGADOR
	//adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_11db);
	adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_11db);
	//analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
    //analogSetSamples(5);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
    analogSetClockDiv(255);            	    // Set the divider for the ADC clock, default is 1, range is 1 - 255

	
	// LED (viejo)
	//pinMode(PINLED, OUTPUT);
	//digitalWrite(PINLED, LOW);

	// Contador de flujo
	t_flujotick = 0;
	pinMode(PINFLUJO, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PINFLUJO),RiegaMatico::ISRFlujoTick,FALLING);


	// Sensor Ambiente
	SensorAmbiente.setup(PINAMBIENTE, DHTesp::DHT11);

	// LCD
	lcd.init();
	lcd.noBacklight();
	lcd.off();

	// Variables para el tiempo de carga
	tstart_carga = millis();
	tstop_carga = tstart_carga;

	// Para el tiempo en bateria de emergencia
	tvbat_low = millis();
	
}

// Pasar a esta clase la funcion callback de fuera. Me la pasan desde el programa con el metodo SetRespondeComandoCallback
void RiegaMatico::SetRespondeComandoCallback(RespondeComandoCallback ref) {

	MiRespondeComandos = (RespondeComandoCallback)ref;

}

// Metodo que devuelve un JSON con el estado
String RiegaMatico::MiEstadoJson(int categoria) {

	DynamicJsonBuffer jBuffer;
	JsonObject& jObj = jBuffer.createObject();

	// Dependiendo del numero de categoria en la llamada devolver unas cosas u otras
	switch (categoria)
	{

	// JSON CON LA INFORMACION PRINCIPAL DEL HARDWARE
	case 1:

		// Esto llena de objetos de tipo "pareja propiedad valor"
		jObj.set("TIME", ClienteNTP.getFormattedTime());		// HORA
		jObj.set("UPT", t_uptime);								// Uptime en segundos
		jObj.set("HI", HardwareInfo);							// Info del Hardware
		jObj.set("CS", ComOK);									// Info de la conexion WIFI y MQTT
		jObj.set("RSSI", WiFi.RSSI());							// RSSI de la señal Wifi
		jObj.set("VBAT", t_vbateria);							// Tension de la Bateria
		jObj.set("VCARG", t_vcargador);							// Tension del cargador
		jObj.set("TCARG", (tstop_carga - tstart_carga)/60000);	// Tiempo de ultima carga en minutos
		jObj.set("RESERVA", t_nivel);							// Estado del la reserva del deposito
		jObj.set("CARG", cargando);								// Estado de la carga
        jObj.set("RECW", reconexioneswifi);						// Numero de reconexiones Wifi
		
		break;

	// JSON CON LA INFO DE RIEGOS
	case 2:

		jObj.set("TCICLO", t_ciclo_global);						// Valor duracion de cada riego parcial (en segundos)					
		jObj.set("TPAUSA", t_espera_parciales);					// Tiempo de espera entre los parciales (segundos)
		jObj.set("NCICLOS", t_n_parciales);						// Numero de parciales
		jObj.set("NCICLOSREST", t_n_parciales_count);			// Total parciales que quedan del trabajo de riego
		jObj.set("BOMBACUR", ledcRead(1));						// Valor actual PWM de la bomba
		jObj.set("BOMBASET", fuerzabomba);						// Valor de configuracion de la fuerza de la bomba
		jObj.set("FLUJO",flujoactual);							// flujo en ml/s
		jObj.set("LITROS",(float)t_flujotick*1000/TICKSPORLITRO);// Agua Ultimo riego en ML 
		jObj.set("TICKS",t_flujotick);							// Ticks del medidor de flujo
		jObj.set("RIEGOERR", riegoerror);						// Estado de error del riego
		jObj.set("ULTRIEGO",horaultimoriego);					// Fecha y hora del ultimo riego
		jObj.set("RSTAT", ARegar);								// Estado actual del trabajo de rejar

		break;


	// JSON CON LA INFORMACION DE AMBIENTE
	case 3:

		jObj.set("EXTTEMP", LecturaAmbiente.temperature);	// Temperatura Ambiente
		jObj.set("EXTHUM", LecturaAmbiente.humidity);		// Humedad Ambiente
		jObj.set("TEMPT1", TempTierra1);					// Temperatura de la maceta 1
		jObj.set("TEMPT2", TempTierra2);					// Temperatura de la maceta 1

		break;


	// SI ME LLAMAN CON OTRO PARAMETRO
	default:

		jObj.set("NOINFO", "NOINFO");						// MAL LLAMADO

		break;
	}


	// Crear un buffer (aray de 200 char) donde almacenar la cadena de texto del JSON
	char JSONmessageBuffer[200];

	// Tirar al buffer la cadena de objetos serializada en JSON con la propiedad printTo del objeto de arriba
	// Como va medida al buffer de [200] si te pasas se corta el JSON
	jObj.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

	// devolver el char array con el JSON
	return JSONmessageBuffer;
	
}

void RiegaMatico::Regar(){

	if (t_vbateria >= VBATMINRIEGO){

		ARegar = true;
		t_flujotick = 0;
		t_flujotick_previo = 0;
		t_n_parciales_count = t_n_parciales;
		this->MiRespondeComandos("REGAR","OK");

	}
	
	else {

		riegoerror = true;
		this->MiRespondeComandos("REGAR","ERR:NEB");
		Serial.println("Bateria demasiado baja para regar, cancelando riego.");

	}
		
}

void RiegaMatico::Cancelar(){
	
	ledcWrite(1,0);

	b_activa=false;
	ARegar = false;
	t_n_parciales_count = 0;
			
	// Verificar si realmente hemos echado agua con el medidor de flujo
	if (t_flujotick > 100){

		riegoerror = false;

	}

	else{

		riegoerror = true;

	}

	this->MiRespondeComandos("REGAR","CANCELADO");


}

boolean RiegaMatico::SalvaConfig(){
	

	File mificheroconfig_handler = SPIFFS.open(mificheroconfig, "w");

	if (!mificheroconfig_handler) {
		Serial.println("No se puede abrir el fichero del Riegamatico");
		return false;
	}

	if (mificheroconfig_handler.print(MiEstadoJson(2))){

		Serial.println("Config del Riegamatico Salvada.");
		return true;
		

	}

	else {

		return false;

	}

}

boolean RiegaMatico::LeeConfig(){

	// Sacar del fichero de configuracion, si existe, las configuraciones permanentes
	if (SPIFFS.exists(mificheroconfig)) {

		File mificheroconfig_handler = SPIFFS.open(mificheroconfig, "r");
		if (mificheroconfig_handler) {
			size_t size = mificheroconfig_handler.size();
			// Declarar un buffer para almacenar el contenido del fichero
			std::unique_ptr<char[]> buf(new char[size]);
			// Leer el fichero al buffer
			mificheroconfig_handler.readBytes(buf.get(), size);
			DynamicJsonBuffer jsonBuffer;
			JsonObject& json = jsonBuffer.parseObject(buf.get());
			if (json.success()) {

				Serial.print("Configuracion del Riegamatico Leida: ");
				json.printTo(Serial);
				Serial.println("");

				// Dar valor a las variables desde el JSON de configuracion
				t_ciclo_global = json.get<unsigned long>("TCICLO");
				t_espera_parciales = json.get<unsigned long>("TPAUSA");
				t_n_parciales = json.get<unsigned long>("NCICLOS");
				fuerzabomba = json.get<int>("BOMBASET");

				return true;

			}

			return false;

		}

		return false;

	}

	return false;

}

void RiegaMatico::ConfigTiempoRiego(unsigned long tiempo_riego){

	t_ciclo_global = tiempo_riego;
	HayQueSalvar = true;
	this->MiRespondeComandos("TRIEGO",String(t_ciclo_global));

}	

void RiegaMatico::ConfigEsperaParciales(unsigned long tiempo_espera){

	t_espera_parciales = tiempo_espera;
	HayQueSalvar = true;
	this->MiRespondeComandos("TPAUSA",String(t_espera_parciales));

}	

void RiegaMatico::ConfigNumParciales(int n_parciales){

	t_n_parciales = n_parciales;
	HayQueSalvar = true;
	this->MiRespondeComandos("NPARCIALES",String(t_n_parciales));

}

void RiegaMatico::ConfigPWMBomba(int n_fuerzabomba){

	fuerzabomba = n_fuerzabomba;
	HayQueSalvar = true;
	this->MiRespondeComandos("BOMBASET",String(fuerzabomba));

}

void RiegaMatico::MandaConfig(){

	this->MiRespondeComandos("TRIEGO",String(t_ciclo_global));
	this->MiRespondeComandos("TPAUSA",String(t_espera_parciales));
	this->MiRespondeComandos("NPARCIALES",String(t_n_parciales));
	this->MiRespondeComandos("BOMBASET",String(fuerzabomba));

}

void RiegaMatico::ISRFlujoTick(){			// ISR que SI le puedo pasar al AttachInterrupt (estatica) que llama a una funcion de ESTA instancia (sRiegaMatico = this)

	if (sRiegaMatico != 0){

		sRiegaMatico->FujoTick();			// Y por tanto SI puedo llamar a una funcion publica no estatica (y por ende la llama la interrupcion y la puedo llamar por otro lado si gusto)

	}

}

void RiegaMatico::FujoTick(){				// Funcion Publica que incremanta el contador de flujo

	if (this->ARegar){

		t_flujotick++;

	}
	

}

// Se lanza aqui en el run, que se lanza a su vez a traves de Task cada 1000ms
void RiegaMatico::CalculaFlujo(){

	
	if (b_activa){



		// Sacar la diferencia de tiempo entre la anterior medicion y la actual
		unsigned long tiempo_diff = millis() - tflujo_agua_previo;
				
		// Si esta dentro de un rango "normal" hacer el calculo
		if (tiempo_diff > 0 && tiempo_diff < 1500) {

			// Calcular
			flujoactual = ((t_flujotick - t_flujotick_previo)*1000*1000)/(tiempo_diff*TICKSPORLITRO);
		
		}

		// Si no el flujo a cero
		else {

			flujoactual = 0;

		}

		// Almacenar los valores actuales para la siguiente vez que se calcule y poder hacer las restas, el del tiempo y el numero de ticks
		tflujo_agua_previo = millis();
		t_flujotick_previo = t_flujotick;

	}

	else {

		flujoactual = 0;
		millis_previo = 0;

	}

}

void RiegaMatico::RiegoRun(){

	// Si tengo activo el comando regar y la bomba esta parada o es el inicio del riego o estoy en una pausa 
	if ( ARegar == true && b_activa == false){

		// en este caso pueden ocurrir varias cosas:

		// Que sea el primer riego nada mas lanzar el comando. Ni esperamos ni nada a regar
		if (t_n_parciales_count == t_n_parciales){

			t_init_riego = millis(); // Actualizo la marca de tiempo
			b_activa=true;
			t_n_parciales_count--;
			horaultimoriego = ClienteNTP.getFormattedTime();

			//digitalWrite(PINLED, HIGH);
			ledcWrite(1,fuerzabomba);
			this->MiRespondeComandos("REGAR","BOMBA: " + String(fuerzabomba));

		}

		// Si estoy en algun parcial que no sea el ultimo y se ha superado el tiempo de parcial
		else if (t_n_parciales_count < t_n_parciales && t_n_parciales_count > 0 && (millis() - t_init_riego) >= t_espera_parciales*1000){

			t_init_riego = millis(); 		// Actualizo la marca de tiempo
			b_activa=true;
			t_n_parciales_count--;	

			//digitalWrite(PINLED, HIGH);
			ledcWrite(1,fuerzabomba);
			this->MiRespondeComandos("REGAR","BOMBA: " + String(fuerzabomba));

		}

		// Si estoy en el ultimo parcial ....
		else if (t_n_parciales_count <= 0){

			this->MiRespondeComandos("REGAR","FIN RIEGO");
			this->Cancelar();

		}
				
	}
	
	// Si la bomba esta activa no pensamos mucho. Si el tiempo de riego acaba parar la bomba
	if ( b_activa == true && (millis() - t_init_riego) >= t_ciclo_global*1000){

		ledcWrite(1,0);
		//digitalWrite(PINLED, LOW);
		b_activa=false;

		t_init_riego = millis(); // Empieza la pausa y voy a usar esto tambien para contar

		this->MiRespondeComandos("REGAR","FIN PARCIAL");

	}

}

bool RiegaMatico::EstaRegando(){

	return ARegar;

}

void RiegaMatico::GestionCarga(boolean fuerzacarga){

	
	// Paras almacenar varias lecturas	
	float t_lecturas = 0;


	// Bucle de 10 lecuras de VBATERIA
	for (int i = 1; i <= 10; i++){

		t_lecturas = t_lecturas + analogRead(PINVBAT);
		delay (10);

	}
	
	// Dividir el bloque de lecturas entre 10 y convertir la medida segun dicisor para la lectura OK
	t_vbateria =  (t_lecturas * 12.92f ) / (3440.0f * 10);
	

	t_lecturas = 0;

	// Bucle de 10 lecuras de VCARGA
	for (int i = 1; i <= 10; i++){

		t_lecturas = t_lecturas + analogRead(PINVCARGA);
		delay (10);

	}
	
	// Dividir el bloque de lecturas entre 10 para la lectura OK
	t_vcargador =  (t_lecturas * 17.55f ) / (3440.0f * 10);
	
	
	// CARGAR POR BATERIA BAJA
	if (t_vbateria <= VCARGASTART && !cargando){

		digitalWrite(PINCARGA,HIGH);
		cargando = true;
		tstart_carga = millis();

	}

	// CARGAR POR COMANDO FORZADO

	if (fuerzacarga && !cargando){

		digitalWrite(PINCARGA,HIGH);
		cargando = true;
		tstart_carga = millis();
		MiRespondeComandos("CARGAR","CARGANDO");

	}

	
	// Parar la carga
	if (t_vbateria >= VCARGASTOP && (millis() - tstart_carga) > (CARGAMINTEMP*60000)){

		digitalWrite(PINCARGA,LOW);
		cargando = false;
		tstop_carga = millis();

	}


	// Apagado de Emergencia si la tension cae por debajo del umbral establecido durante un tiempo X
	if (t_vbateria <= VBATEMERGENCIA){

		if ( (millis() - tvbat_low) > TBATEMERGENCIA*1000 ){

			Serial.println("Atencion, Bateria extremadamente baja, hibernando el sistema.");
			this->Adormir(SleepModeApagado);

		}
		
	}

	else {

		tvbat_low = millis();

	}
	


	// Ir actualizando la variable para informar de cuanto lleva cargando en la telemetria durante la carga
	if (cargando){tstop_carga = millis();}

}

void RiegaMatico::LeeAmbiente(){

	LecturaAmbiente = SensorAmbiente.getTempAndHumidity();
	
}

void RiegaMatico::LeeTempTierra(){

	// Que los sensores hagan las lecturas
	MisTermometrosOnewire.requestTemperatures();
	// Lo vamos a pasar a variables locales a ver si va bien (si da tiempo a la conversion o al menos lee la anterioi) para poder usarlas luego sin lios
	TempTierra1 = MisTermometrosOnewire.getTempC(TempMaceta1);
	TempTierra2 = MisTermometrosOnewire.getTempC(TempMaceta2);

	//NOTA: Hay funciones para hacer verificaciones de si el dispositivo esta conectado o no para mejorar posibles fallos.

}

void RiegaMatico::Adormir(SleepModes modo){

	switch (modo){

		case SleepModeApagado:

			lcd.noBacklight();
			lcd.off();
			esp_sleep_enable_timer_wakeup(THIBERNADO * (unsigned long long)60000000);
			Serial.println("Entrando en modo Deep Sleep");
			esp_deep_sleep_start();		

		break;

	}

}

// Ejecutar cada 1s
void RiegaMatico::Run() {
	
	// Si hay que salvar, salvar (facil no?)
	if (HayQueSalvar){

		SalvaConfig();
		HayQueSalvar = false;

	}
	
	// Leer el sensor de Reserva de Nivel de Agua.
	t_nivel = digitalRead(PINNIVEL);


	this->RiegoRun();
	this->GestionCarga(false);
	this->LeeAmbiente();
	this->LeeTempTierra();
	this->CalculaFlujo();
	

	// UpTime Minutos
	t_uptime = esp_timer_get_time() / 1000000;


    // Estados en el LED

    // Si estoiy en un ciclo de riego respiracion rapida
    if (ARegar == true){

       LedEstado.Breathe(1000).Forever();

    }

    // Y si no reflejar el estado de la Wifi (de momento)
    else{

		
        switch (WiFi.status()){

        case WL_CONNECTED:
            //LedEstado.Breathe(5000).Forever();
			LedEstado.Blink(200,4000).Forever();
            break;

        case WL_IDLE_STATUS:
            LedEstado.Blink(2000,2000).Forever();
            break;    
        
        default:
            LedEstado.Blink(200,200).Forever();
            break;

        }
		

    }
    
    

}

// Ejecutar en el loop o lo mas rapido posible.
void RiegaMatico::RunFast() {

	// Actualizar Led
    LedEstado.Update();
	

}

void RiegaMatico::Begin(){

	// Para inicializar los termometros de la tierra
	MisTermometrosOnewire.begin();


}