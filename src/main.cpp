#pragma region COMENTARIOS

/*

RIEGAMATICO MQTT 1.0
Control de riego con ESP32 y capacidades MQTT
Desarrollado con Visual Code + PlatformIO + Plataforma Espressif 32 Arduino
Implementa las comunicaciones WIFI y MQTT asi como la configuracion de las mismas via comandos
Implementa el envio de comandos via puerto serie o MQTT
Implementa el uso de tareas para multiproceso y para usar ambos cores


Caracteristicas del control de riego:


Author: Diego Maroto - BilbaoMakers 2019 - info@bilbaomakers.org - dmarofer@diegomaroto.net
https://github.com/dmarofer/RIEGAMATICO_MQTT
https://bilbaomakers.org/
Licencia: GNU General Public License v3.0 ( mas info en GitHub )

*/

#pragma endregion

#pragma region INCLUDES
// Librerias comantadas en proceso de sustitucion por la WiFiMQTTManager

#include <ConfigCom.h>					// La configuracion de las comunicaciones
#include <RiegaMatico.h>				// La clase RiegaMatico
#include <FS.h>							// Libreria Sistema de Ficheros
#include <ArduinoJson.h>				// OJO: Tener instalada una version NO BETA (a dia de hoy la estable es la 5.13.4). Alguna pata han metido en la 6
#include <string>						// Para el manejo de cadenas
#include <NTPClient.h>					// Para la gestion de la hora por NTP
#include <WiFiUdp.h>					// Para la conexion UDP con los servidores de hora.
#include <ArduinoOTA.h>					// Actualizaciones de firmware por red.
#include <Comunicaciones.h>

#pragma endregion

#pragma region Objetos

// Los manejadores para las tareas. El resto de las cosas que hace nuestro controlador que son un poco mas flexibles que la de los pulsos del Stepper
TaskHandle_t THandleTaskRiegaMaticoRun,THandleTaskProcesaComandos,THandleTaskComandosSerieRun,THandleTaskEnviaTelemetria,THandleTaskGestionRed,THandleTaskTX;	

// Manejadores Colas para comunicaciones inter-tareas
QueueHandle_t ColaCMND,ColaTX;

// Timer Run
//hw_timer_t * timer_stp = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Flag para el estado del sistema de ficheros
boolean SPIFFStatus = false;

// Conexion UDP para la hora
WiFiUDP UdpNtp;

// Manejador del NTP. Cliente red, servidor, offset zona horaria, intervalo de actualizacion.
// FALTA IMPLEMENTAR ALGO PARA CONFIGURAR LA ZONA HORARIA
static NTPClient ClienteNTP(UdpNtp, "europe.pool.ntp.org", HORA_LOCAL * 3600, 600000);

// Para el sensor de temperatura de la CPU. Definir aqui asi necesario es por no estar en core Arduino.
extern "C" {uint8_t temprature_sens_read();}

// Objeto de la configuracion en fichero
ConfigCom MiConfig(FICHERO_CONFIG_COM);

// Para las Comunicaciones
Comunicaciones MisComunicaciones = Comunicaciones();

// Objeto riegamtico
RiegaMatico* RiegaMatico::sRiegaMatico	 = 0;						// Instancia vacia de la clase (reserva de mem)
RiegaMatico MiRiegaMatico(FICHERO_CONFIG_PRJ, ClienteNTP);

#pragma endregion

#pragma region Funciones de Eventos, CallBack y Auxiliares

// Callback. Manda a la cola de TX el mensaje de respuesta desde la clase RiegaMatico
void MandaRespuesta(String comando, String payload) {

			String t_topic = MiConfig.statTopic + "/" + comando;

			DynamicJsonBuffer jsonBuffer;
			JsonObject& ObjJson = jsonBuffer.createObject();
			// Tipo de mensaje (MQTT, SERIE, BOTH)
			ObjJson.set("TIPO","BOTH");
			// Comando
			ObjJson.set("CMND",comando);
			// Topic (para MQTT)
			ObjJson.set("MQTTT",t_topic);
			// RESPUESTA
			ObjJson.set("RESP",payload);

			char JSONmessageBuffer[300];
			ObjJson.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
			
			// Mando el comando a la cola de respuestas
			xQueueSend(ColaTX, JSONmessageBuffer, 0); 

}

// Funcion para formar el JSON y enviar a la ColaTX de la telemetria tipo X. Asi para poder mandar otras cosas como las configuraciones leidas desde fichero
void CocinaTelemetria (String TopicDestino, String JsonTelemetria){

		// El JSON para la cola de envio
		DynamicJsonBuffer jsonBuffer;
		JsonObject& ObjJson = jsonBuffer.createObject();
		ObjJson.set("TIPO","MQTT");
		ObjJson.set("CMND","TELE");
		ObjJson.set("MQTTT",TopicDestino);
		ObjJson.set("RESP",JsonTelemetria);
		
		// Buffer para el msg de la cola de envio
		char JSONmessageBuffer[300];
		ObjJson.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

		// Mando el comando a la cola de respuestas
		//Serial.println(JSONmessageBuffer);
		xQueueSend(ColaTX, JSONmessageBuffer, 0); 

}

// Funcion ante un evento de la wifi
void WiFiEventCallBack(WiFiEvent_t event) {
    
	switch(event) {
				
		case SYSTEM_EVENT_STA_START:
			Serial.println("Conexion WiFi: Iniciando ...");
			break;
    	
		case SYSTEM_EVENT_STA_GOT_IP:
     	   	Serial.print("Conexion WiFi: Conetado. IP: ");
      	  	Serial.println(WiFi.localIP());
			ArduinoOTA.begin();
			Serial.println("Proceso OTA arrancado.");
			ClienteNTP.begin();
			MisComunicaciones.Conectar();
			MiRiegaMatico.reconexioneswifi ++;
        	break;

    	case SYSTEM_EVENT_STA_DISCONNECTED:
        	Serial.println("Conexion WiFi: Desconetado");
	       	break;

		default:
			Serial.println("Evento de Wifi no contemplado");
			break;

    }
		
}

// Manejador de eventos de la clase comunicaciones
void EventoComunicaciones (unsigned int Evento_Comunicaciones, char Info[200]){

	
	switch (Evento_Comunicaciones)
	{
	case Comunicaciones::EVENTO_CONECTANDO:
	
		Serial.print("MQTT - CONECTANDO: ");
		Serial.println(String(Info));

	break;
	
	case Comunicaciones::EVENTO_CONECTADO:

		Serial.print("MQTT - CONECTADO: ");
		Serial.println(String(Info));
		MiRiegaMatico.MandaConfig();
		CocinaTelemetria(MiConfig.teleTopic + "/INFO1", MiRiegaMatico.MiEstadoJson(1));
		CocinaTelemetria(MiConfig.teleTopic + "/INFO2", MiRiegaMatico.MiEstadoJson(2));
		CocinaTelemetria(MiConfig.teleTopic + "/INFO3", MiRiegaMatico.MiEstadoJson(3));
		CocinaTelemetria(MiConfig.teleTopic + "/INFO4", MiRiegaMatico.MiEstadoJson(4));
		//LedEstado.Ciclo(100,100,5000,1);		// Led en ciclo conectado

	break;

	case Comunicaciones::EVENTO_CMND_RX:

		Serial.print("MQTT - CMND_RX: ");
		Serial.println(String(Info));
		xQueueSend(ColaCMND, Info, 0);

	break;

	case Comunicaciones::EVENTO_TELE_RX:

		//Serial.print("MQTT - TELE_RX: ");
		//Serial.println(String(Info));
		//ColaTelemetria.push(Info);

	break;

	case Comunicaciones::EVENTO_DESCONECTADO:

		//LedEstado.Ciclo(500,500,0,1);		// Led en ciclo desconectado

	break;

	default:
	break;

	}


}

#pragma endregion

#pragma region TASKS

// Tarea para vigilar la conexion con el MQTT y conectar si no estamos conectados
void TaskGestionRed ( void * parameter ) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 30000;
	xLastWakeTime = xTaskGetTickCount ();


	while(true){

		if (WiFi.isConnected()){
			
			if ( !MisComunicaciones.IsConnected() ){

				MisComunicaciones.Conectar();

			}

			// Esto teoricamente se ejecuta en loop y solo actualiza en el tiempo que le damos al constructor, pero con esto es suficiente
			ClienteNTP.update();
			
		}

		else {

			WiFi.disconnect();
			WiFi.begin();
			//WiFi.reconnect();

		}
				
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}

// Tarea para los comandos que llegan por el puerto serie. Los cocina y manda a la ColaCMND
void TaskComandosSerieRun( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount ();

	char sr_buffer[120];
	int16_t sr_buffer_len(sr_buffer!=NULL && sizeof(sr_buffer) > 0 ? sizeof(sr_buffer) - 1 : 0);
	int16_t sr_buffer_pos = 0;
	char* sr_term = "\r\n";
	char* sr_delim = " ";
	int16_t sr_term_pos = 0;
	char* sr_last_token;
	char* comando = "NA";
	char* parametro1 = "NA";

	while(true){
		
		while (Serial.available()) {

			// leer un caracter del serie (en ASCII)
			int ch = Serial.read();

			// Si es menor de 0 es KAKA
			if (ch <= 0) {
				
				continue;
			
			}

			// Si el buffer no esta lleno, escribir el caracter en el buffer y avanzar el puntero
			if (sr_buffer_pos < sr_buffer_len){
			
				sr_buffer[sr_buffer_pos++] = ch;
				//Serial.println("DEBUG: " + String(sr_buffer));

			}
		
			// Si esta lleno ........
			else { 

				return;

			}

			// Aqui para detectar el retorno de linea
			if (sr_term[sr_term_pos] != ch){
				sr_term_pos = 0;
				continue;
			}

			// Si hemos detectado el retorno de linea .....
			if (sr_term[++sr_term_pos] == 0){

				sr_buffer[sr_buffer_pos - strlen(sr_term)] = '\0';

				// Aqui para sacar cada una de las "palabras" del comando que hemos recibido con la funcion strtok_r (curiosa funcion)
				comando = strtok_r(sr_buffer, sr_delim, &sr_last_token);
				parametro1 = strtok_r(NULL, sr_delim, &sr_last_token);

				// Formatear el JSON del comando y mandarlo a la cola de comandos.
				DynamicJsonBuffer jsonBuffer;
				JsonObject& ObjJson = jsonBuffer.createObject();
				ObjJson.set("COMANDO",comando);
				ObjJson.set("PAYLOAD",parametro1);

				char JSONmessageBuffer[200];
				ObjJson.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
			
				// Mando el comando a la cola de comandos recibidos que luego procesara la tarea manejadordecomandos.
				xQueueSend(ColaCMND, JSONmessageBuffer, 0);
				
				// Reiniciar los buffers
				sr_buffer[0] = '\0';
				sr_buffer_pos = 0;
				sr_term_pos = 0;
				
			}
		

		}
	
		
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
	
}

// Tarea para el metodo run del objeto Riegamatico
void TaskRiegaMaticoRun( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount ();
	
	while(true){

		MiRiegaMatico.Run();
		
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}

// Tarea para generar el JSON para la cola ColaTX para que mande la telemetria
void TaskEnviaTelemetria( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequencyLargo = 10000;
	const TickType_t xFrequencyCorto = 2000;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){

	
		CocinaTelemetria(MiConfig.teleTopic + "/INFO1", MiRiegaMatico.MiEstadoJson(1));

		CocinaTelemetria(MiConfig.teleTopic + "/INFO2", MiRiegaMatico.MiEstadoJson(2));

		CocinaTelemetria(MiConfig.teleTopic + "/INFO3", MiRiegaMatico.MiEstadoJson(3));

		CocinaTelemetria(MiConfig.teleTopic + "/INFO4", MiRiegaMatico.MiEstadoJson(4));

		
		if (MiRiegaMatico.EstaRegando()){

			vTaskDelayUntil( &xLastWakeTime, xFrequencyCorto );

		}

		else {

			vTaskDelayUntil( &xLastWakeTime, xFrequencyLargo );
			
		}
	
	}
	
}

//Tarea para procesar la cola de comandos recibidos
void TaskProcesaComandos ( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 250;
	xLastWakeTime = xTaskGetTickCount ();

	char JSONmessageBuffer[200];
	
	while (true){
			
			// Limpiar el Buffer
			memset(JSONmessageBuffer, 0, sizeof JSONmessageBuffer);

			if (xQueueReceive(ColaCMND,JSONmessageBuffer,0) == pdTRUE ){

				String COMANDO;
				String PAYLOAD;
				DynamicJsonBuffer jsonBuffer;

				JsonObject& ObjJson = jsonBuffer.parseObject(JSONmessageBuffer);

				if (ObjJson.success()) {
				
					COMANDO = ObjJson["COMANDO"].as<String>();
					PAYLOAD = ObjJson["PAYLOAD"].as<String>();
					
					// De aqui para abajo la retaila de comandos que queramos y lo qude han de hacer.

					// ##### COMANDOS PARA LA GESTION DE LA CONFIGURACION

					if (COMANDO == "WSsid"){
						
						String(PAYLOAD).toCharArray(MiConfig.Wssid, sizeof(MiConfig.Wssid));
						Serial.println("Wssid OK: " + PAYLOAD);

					}

					else if (COMANDO == "WPasswd"){

						String(PAYLOAD).toCharArray(MiConfig.WPasswd, sizeof(MiConfig.WPasswd));
						Serial.println("Wpasswd OK: " + PAYLOAD);
						
					}

					else if (COMANDO == "MQTTSrv"){

						String(PAYLOAD).toCharArray(MiConfig.mqttserver, sizeof(MiConfig.mqttserver));
						Serial.println("MQTTSrv OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTUser"){

						String(PAYLOAD).toCharArray(MiConfig.mqttusuario, sizeof(MiConfig.mqttusuario));
						Serial.println("MQTTUser OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTPasswd"){

						String(PAYLOAD).toCharArray(MiConfig.mqttpassword, sizeof(MiConfig.mqttpassword));
						Serial.println("MQTTPasswd OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTTopic"){

						String(PAYLOAD).toCharArray(MiConfig.mqtttopic, sizeof(MiConfig.mqtttopic));
						Serial.println("MQTTTopic OK: " + PAYLOAD);

					}

					else if (COMANDO == "SaveCom"){

						if (MiConfig.escribeconfig()){

							MisComunicaciones.SetMqttServidor(MiConfig.mqttserver);
							MisComunicaciones.SetMqttUsuario(MiConfig.mqttusuario);
							MisComunicaciones.SetMqttPassword(MiConfig.mqttpassword);
							MisComunicaciones.SetMqttTopic(MiConfig.mqtttopic);
							MisComunicaciones.SetMqttClientId("SINIMPLEMENTAR");
							WiFi.begin(MiConfig.Wssid, MiConfig.WPasswd);

						}
						
					}

					else if (COMANDO == "Help"){

						Serial.println("Comandos para la configuracion de las comunicaciones:");
						Serial.println("WSsid <SSID> - Configurar SSID de la Wifi");
						Serial.println("WPasswd <Contraseña> - Configurar contraseña de la Wifi ");
						Serial.println("MQTTSrv <IP|URL> - Direccion del Broker MQTT");
						Serial.println("MQTTUser <usuario> - Usuario para el Broker MQTT");
						Serial.println("MQTTPasswd <contraseña> - Contraseña para el usuario del Broker MQTT");
						Serial.println("MQTTTopic <string> - Nombre para la jerarquia de los topics MQTT");
						Serial.println("SaveCom - Salvar la configuracion en el microcontrolador");
						
					}

					// Comandos del riegamatico

					else if (COMANDO == "REGAR"){

						MiRiegaMatico.Regar();
						
					}

					else if (COMANDO == "CANCELAR"){

						MiRiegaMatico.Cancelar();

					}

					else if (COMANDO == "TRIEGO"){

						MiRiegaMatico.ConfigTiempoRiego(PAYLOAD.toDouble());

					}

					else if (COMANDO == "TPAUSA"){

						MiRiegaMatico.ConfigEsperaParciales(PAYLOAD.toDouble());

					}

					else if (COMANDO == "NPARCIALES"){

						MiRiegaMatico.ConfigNumParciales(PAYLOAD.toInt());

					}

					else if (COMANDO == "BOMBASET"){

						MiRiegaMatico.ConfigPWMBomba(PAYLOAD.toInt());

					}

					else if (COMANDO == "CARGAR"){

						MiRiegaMatico.GestionCarga(true);

					}

					else if (COMANDO == "REBOOT"){

						ESP.restart();

					}


					// Y Ya si no es de ninguno de estos ....

					else {

						Serial.println("Me ha llegado un comando que no entiendo");
						Serial.println("Comando: " + COMANDO);
						Serial.println("Payload: " + PAYLOAD);

					}

				}

				// Y si por lo que sea la libreria JSON no puede convertir el comando recibido
				else {

						Serial.println("La tarea de procesar comandos ha recibido uno que no puede deserializar.");
						Serial.println(JSONmessageBuffer);
						
				}
			
			}
		
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}

// Tarea para procesar la cola de envios
void TaskTX( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 250;
	xLastWakeTime = xTaskGetTickCount ();
	
	char JSONmessageBuffer[300];
	

	while(true){

		// Limpiar el Buffer
		memset(JSONmessageBuffer, 0, sizeof JSONmessageBuffer);

		// Saco el mensaje de la cola si hay conexion MQTT ( si no no porque si lo primero el false ya no se ejecuta lo segundo)
		if (MisComunicaciones.IsConnected() && xQueueReceive(ColaTX,JSONmessageBuffer,0) == pdTRUE ){

				DynamicJsonBuffer jsonBuffer;
				
				JsonObject& ObjJson = jsonBuffer.parseObject(JSONmessageBuffer);

				if (ObjJson.success()) {
					
					String TIPO = ObjJson["TIPO"].as<String>();
					String CMND = ObjJson["CMND"].as<String>();
					String MQTTT = ObjJson["MQTTT"].as<String>();
					String RESP = ObjJson["RESP"].as<String>();
					
					char BufferTopic[75];
					char BufferPayload[200];

					strcpy(BufferTopic, MQTTT.c_str());
					strcpy(BufferPayload, RESP.c_str());

					if (TIPO == "BOTH"){

						MisComunicaciones.Enviar(BufferTopic, BufferPayload);
						Serial.println(ClienteNTP.getFormattedTime() + " " + CMND + " " + RESP);
						
					}

					else 	if (TIPO == "MQTT"){

						MisComunicaciones.Enviar(BufferTopic, BufferPayload);
																								
					}
					
					else 	if (TIPO == "SERIE"){

							Serial.println(ClienteNTP.getFormattedTime() + " " + CMND + " " + RESP);
						
					}
						
				}
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}

#pragma endregion

#pragma region Funcion Setup() de ARDUINO

// funcion SETUP de Arduino
void setup() {
	
	// Puerto Serie
	Serial.begin(115200);
	Serial.println();

	Serial.println("-- Iniciando Controlador RiegaMatico --");

	// Asignar funciones Callback
	MiRiegaMatico.SetRespondeComandoCallback(MandaRespuesta);

	// Comunicaciones
	WiFi.onEvent(WiFiEventCallBack);
	WiFi.setHostname("RIEGAMATICO");
	
	// Iniciar la Wifi
	WiFi.begin();

	// Incializar el objeto Riegamatico con sus cosas.
	MiRiegaMatico.Begin();
	

	// Iniciar el sistema de ficheros y formatear si no lo esta
	SPIFFStatus = SPIFFS.begin(true);

	if (SPIFFS.begin()){

		Serial.println("Sistema de ficheros montado");

		// Leer la configuracion de Comunicaciones
		if (MiConfig.leeconfig()){

			// Las funciones callback de la libreria MQTT	
			MisComunicaciones.SetEventoCallback(EventoComunicaciones);
			MisComunicaciones.SetMqttServidor(MiConfig.mqttserver);
			MisComunicaciones.SetMqttUsuario(MiConfig.mqttusuario);
			MisComunicaciones.SetMqttPassword(MiConfig.mqttpassword);
			MisComunicaciones.SetMqttClientId(MiConfig.mqtttopic);
			MisComunicaciones.SetMqttTopic(MiConfig.mqtttopic);

			// Tarea de gestion de la conexion MQTT. Lanzamos solo si conseguimos leer la configuracion
			xTaskCreatePinnedToCore(TaskGestionRed,"MQTT_Conectar",3000,NULL,1,&THandleTaskGestionRed,0);
	
		}

		// Leer configuracion salvada del Objeto MiRiegaMatico
		MiRiegaMatico.LeeConfig();

	}

	else {

		SPIFFS.begin(true);

	}

	// COLAS
	ColaCMND = xQueueCreate(10,200);
	ColaTX = xQueueCreate(20,300);
	

	// TASKS
	Serial.println("Creando tareas del sistema.");
	
	// Tareas CORE0
	xTaskCreatePinnedToCore(TaskProcesaComandos,"ProcesaComandos",3000,NULL,1,&THandleTaskProcesaComandos,0);
	xTaskCreatePinnedToCore(TaskTX,"EnviaMQTT",2000,NULL,1,&THandleTaskTX,0);
	xTaskCreatePinnedToCore(TaskEnviaTelemetria,"MandaTelemetria",2000,NULL,1,&THandleTaskEnviaTelemetria,0);
	xTaskCreatePinnedToCore(TaskComandosSerieRun,"ComandosSerieRun",1000,NULL,1,&THandleTaskComandosSerieRun,0);
	
	// Tareas CORE1
	xTaskCreatePinnedToCore(TaskRiegaMaticoRun,"RiegaMaticoRun",2000,NULL,1,&THandleTaskRiegaMaticoRun,1);
	
	
	// Init Completado.
	Serial.println("Setup Completado.");
		
}

#pragma endregion

#pragma region Funcion Loop() de ARDUINO

// Funcion LOOP de Arduino
// Se ejecuta en el Core 1
// Como todo se getiona por Task aqui no se pone NADA
void loop() {

	// Esto aqui de momento porque no me rula en una task y tengo que averiguar por que.
	ArduinoOTA.handle();
	MiRiegaMatico.RunFast();	
	MisComunicaciones.RunFast();	

}

#pragma endregion

/// FIN DEL PROGRAMA ///