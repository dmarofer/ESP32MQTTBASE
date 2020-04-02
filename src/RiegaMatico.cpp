#include <RiegaMatico.h>
#include <Arduino.h>
#include "driver/adc.h"
#include <Configuracion.h>
#include <ArduinoJson.h>				// OJO: Tener instalada una version NO BETA (a dia de hoy la estable es la 5.13.4). Alguna pata han metido en la 6
#include <SPIFFS.h>						// Libreria para sistema de ficheros SPIFFS
#include <WiFi.h>						// Para las comunicaciones WIFI del ESP32
#include <NTPClient.h>					// Para la gestion de la hora por NTP
#include <JLed.h>
#include <esp32-hal.h>

// Led que se maneja con la maravillosa libreria JLed
// Iniciar el led en modo Off
auto LedEstado = JLed(PINLED).Off();

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
	//analogSetSamples(10);
	
	// LED (viejo)
	//pinMode(PINLED, OUTPUT);
	//digitalWrite(PINLED, LOW);

	// Contador de flujo
	t_flujotick = 0;
	pinMode(PINFLUJO, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PINFLUJO),RiegaMatico::ISRFlujoTick,FALLING);

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

	// INFO GENERAL
	case 1:

		// Esto llena de objetos de tipo "pareja propiedad valor"
		jObj.set("TIME", ClienteNTP.getFormattedTime());	// HORA
		jObj.set("UPT", t_uptime);							// Uptime en segundos
		jObj.set("HI", HardwareInfo);						// Info del Hardware
		jObj.set("CS", ComOK);								// Info de la conexion WIFI y MQTT
		//jObj.set("RSSI", WiFi.RSSI());						// RSSI de la señal Wifi
		jObj.set("ADCBAT", t_vbatlectura);					// Lectura del ADC Bateria
		jObj.set("ADCCARG", t_vcarglectura);				// Lectura del ADC del Cargador
		jObj.set("VBAT", t_vbateria);						// Tension de la Bateria
		jObj.set("VCARG", t_vcargador);						// Tension del cargador
		jObj.set("RESERVA", t_nivel);						// Estado del la reserva del deposito
		jObj.set("CARG", cargando);							// Estado de la carga
        jObj.set("RECW", reconexioneswifi);					// Numero de reconexiones Wifi

		break;

	// INFO DE RIEGOS
	case 2:

		jObj.set("TCICLO", t_ciclo_global);						// Valor duracion de cada riego parcial (en segundos)					
		jObj.set("TPAUSA", t_espera_parciales);					// Tiempo de espera entre los parciales (segundos)
		jObj.set("NCICLOS", t_n_parciales);						// Numero de parciales
		jObj.set("NCICLOSREST", t_n_parciales_count);			// Total parciales que quedan del trabajo de riego
		jObj.set("PWMBOMBA", ledcRead(1));						// Valor actual PWM de la bomba
		jObj.set("FLUJO",(float) t_flujotick / TICKSPORLITRO);	// Valor del medidor de flujo (ultimo riego)
		jObj.set("RIEGOERR", riegoerror);						// Estado de error del riego
		jObj.set("ULTRIEGO",horaultimoriego);					// Fecha y hora del ultimo riego
		jObj.set("RSTAT", ARegar);								// Estado actual del trabajo de rejar

		break;

	// SI ME LLAMAN CON OTRO PARAMETRO
	default:

		jObj.set("NOINFO", "NOINFO");						// MAL LLAMADO

		break;
	}


	// Crear un buffer (aray de 100 char) donde almacenar la cadena de texto del JSON
	char JSONmessageBuffer[200];

	// Tirar al buffer la cadena de objetos serializada en JSON con la propiedad printTo del objeto de arriba
	jObj.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

	// devolver el char array con el JSON
	return JSONmessageBuffer;
	
}

void RiegaMatico::Regar(){

	ARegar = true;
	t_flujotick = 0;
	t_n_parciales_count = t_n_parciales;
		
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

	this->MiRespondeComandos("REGAR",this->MiEstadoJson(2));


}

boolean RiegaMatico::SalvaConfig(){
	

	File mificheroconfig_handler = SPIFFS.open(mificheroconfig, "w");

	if (!mificheroconfig_handler) {
		Serial.println("No se puede abrir el fichero de configuracion de mi proyecto");
		return false;
	}

	if (mificheroconfig_handler.print(MiEstadoJson(1))){

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

				Serial.print("Configuracion de mi proyecto Leida: ");
				json.printTo(Serial);
				Serial.println("");
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
	this->MiRespondeComandos("TRIEGO",this->MiEstadoJson(2));

}	

void RiegaMatico::ConfigEsperaParciales(unsigned long tiempo_espera){

	t_espera_parciales = tiempo_espera;
	HayQueSalvar = true;
	this->MiRespondeComandos("TPAUSA",this->MiEstadoJson(2));

}	

void RiegaMatico::ConfigNumParciales(int n_parciales){

	t_n_parciales = n_parciales;
	HayQueSalvar = true;
	this->MiRespondeComandos("NPARCIALES",this->MiEstadoJson(2));

}

void RiegaMatico::ISRFlujoTick(){			// ISR que SI le puedo pasar al AttachInterrupt (estatica) que llama a una funcion de ESTA instancia (sRiegaMatico = this)

	if (sRiegaMatico != 0){

		sRiegaMatico->FujoTick();			// Y por tanto SI puedo llamar a una funcion publica no estatica (y por ende la llama la interrupcion y la puedo llamar por otro lado si gusto)

	}

}

void RiegaMatico::FujoTick(){				// Funcion Publica que incremanta el contador de flujo

	t_flujotick++;

}

void RiegaMatico::Run() {
	
	// Si hay que salvar, salvar (facil no?)
	if (HayQueSalvar){

		SalvaConfig();
		HayQueSalvar = false;

	}
	
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
			ledcWrite(1,240);
			this->MiRespondeComandos("REGAR",this->MiEstadoJson(2));

		}

		// Si estoy en algun parcial que no sea el ultimo y se ha superado el tiempo de parcial
		else if (t_n_parciales_count < t_n_parciales && t_n_parciales_count > 0 && (millis() - t_init_riego) >= t_espera_parciales*1000){

			t_init_riego = millis(); 		// Actualizo la marca de tiempo
			b_activa=true;
			t_n_parciales_count--;	

			//digitalWrite(PINLED, HIGH);
			ledcWrite(1,240);
			this->MiRespondeComandos("REGAR",this->MiEstadoJson(2));

		}

		// Si estoy en el ultimo parcial ....
		else if (t_n_parciales_count <= 0){

			this->Cancelar();

		}
				
	}
	
	// Si la bomba esta activa no pensamos mucho. Si el tiempo de riego acaba parar la bomba
	if ( b_activa == true && (millis() - t_init_riego) >= t_ciclo_global*1000){

		ledcWrite(1,0);
		//digitalWrite(PINLED, LOW);
		b_activa=false;

		t_init_riego = millis(); // Empieza la pausa y voy a usar esto tambien para contar

		this->MiRespondeComandos("REGAR",this->MiEstadoJson(2));

	}

	// Lectura de Sensores
	
	//t_vbatlectura = analogRead(PINVBAT);
	//t_vcarglectura = analogRead(PINVCARGA);

	t_vbatlectura = adc1_get_raw(ADC1_CHANNEL_7);
	delay(10);
	//t_vcarglectura = adc1_get_raw(ADC1_CHANNEL_6);
	//delay(10);

	t_vbateria =  (t_vbatlectura * 12.92f ) / 3440.0f;
	//t_vcargador = (t_vcarglectura * 14.56f) / 3480.0f;

	t_nivel = digitalRead(PINNIVEL);

	
	// CARGA
	if (t_vbateria <= VCARGASTART && !cargando){

		digitalWrite(PINCARGA,HIGH);
		cargando = true;

	}

	
	if (t_vbateria >= VCARGASTOP){

		digitalWrite(PINCARGA,LOW);
		cargando = false;

	}

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

void RiegaMatico::RunFast() {

	// Actualizar Led
    LedEstado.Update();

}
