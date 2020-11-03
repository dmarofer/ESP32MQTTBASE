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

//#include <ConfigCom.h>					// La configuracion de las comunicaciones
#include <RiegaMatico.h>				// La clase RiegaMatico
#include <FS.h>							// Libreria Sistema de Ficheros
#include <ArduinoJson.h>				// OJO: Tener instalada una version NO BETA (a dia de hoy la estable es la 5.13.4). Alguna pata han metido en la 6
#include <string>						// Para el manejo de cadenas
#include <NTPClient.h>					// Para la gestion de la hora por NTP
#include <WiFiUdp.h>					// Para la conexion UDP con los servidores de hora.
#include <ArduinoOTA.h>					// Actualizaciones de firmware por red.
#include <Comunicaciones.h>
#include <IotWebConf.h>					// Para el portal web de configuracion

#pragma endregion

#pragma region Objetos

// Los manejadores para las tareas. El resto de las cosas que hace nuestro controlador que son un poco mas flexibles que la de los pulsos del Stepper
TaskHandle_t THandleTaskRiegaMaticoRun,THandleTaskProcesaComandos,THandleTaskComandosSerieRun,THandleTaskEnviaTelemetria,THandleTaskGestionRed,THandleTaskTX;	

// Manejadores Colas para comunicaciones inter-tareas
QueueHandle_t ColaCMND,ColaTX;

// Para el IotWebConf
const char thingName[] = "RIEGAMATICO";
const char wifiInitialApPassword[] = "12345678";

// Para la id de config de este proyecto
#define CONFIG_VERSION "RGMTEST"
//#define CONFIG_PIN D2
#define STATUS_PIN PINLED
// Definicion de las funciones callback
void wifiConnected();
void configSaved();
boolean formValidator();
// Objetos para el AP
DNSServer dnsServer;
WebServer server(80);
HTTPUpdateServer httpUpdater;

// Buffers para los parametros que vienen de web y/o de la EEPROM
#define STRING_LEN 128
#define NUMBER_LEN 32
char MqttServerValue[STRING_LEN];
char MqttUsuarioValue[STRING_LEN];
char MqttPasswordValue[STRING_LEN];
char MqttTopicValue[STRING_LEN];

// Objeto iotwebconfig
IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);

// Para meter una cabecera Javascript
const char CUSTOMHTML_SCRIPT_INNER[] PROGMEM = "\n\
document.addEventListener('DOMContentLoaded', function(event) {\n\
  let elements = document.querySelectorAll('input[type=\"password\"]');\n\
  for (let p of elements) {\n\
  	let btn = document.createElement('INPUT'); btn.type = 'button'; btn.value = '🔓'; btn.style.width = 'auto'; p.style.width = '83%'; p.parentNode.insertBefore(btn,p.nextSibling);\n\
    btn.onclick = function() { if (p.type === 'password') { p.type = 'text'; btn.value = '🔒'; } else { p.type = 'password'; btn.value = '🔓'; } }\n\
  };\n\
});\n";
// -- HTML element will be added inside the body element.
const char CUSTOMHTML_BODY_INNER[] PROGMEM = "<div><img src='data:image/png;base64,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'/></div>\n";

// -- This is an OOP technique to override behaviour of the existing
// IotWebConfHtmlFormatProvider. Here two method are overriden from
// the original class. See IotWebConf.h for all potentially overridable
// methods of IotWebConfHtmlFormatProvider .
class CustomHtmlFormatProvider : public IotWebConfHtmlFormatProvider
{
protected:
  String getScriptInner() override
  {
    return
      IotWebConfHtmlFormatProvider::getScriptInner() +
      String(FPSTR(CUSTOMHTML_SCRIPT_INNER));
  }
  String getBodyInner() override
  {
    return
      String(FPSTR(CUSTOMHTML_BODY_INNER)) +
      IotWebConfHtmlFormatProvider::getBodyInner();
  }
};
// -- An instance must be created from the class defined above.
CustomHtmlFormatProvider customHtmlFormatProvider;

// Parametros custom
IotWebConfSeparator separator1 = IotWebConfSeparator("Configuracion MQTT");
IotWebConfParameter MqttServer = IotWebConfParameter("Servidor MQTT", "MqttServer", MqttServerValue, STRING_LEN, "text", NULL, "");
IotWebConfParameter MqttUsuario = IotWebConfParameter("Usuario MQTT", "MqttUsuario", MqttUsuarioValue, STRING_LEN, "text", NULL, "");
IotWebConfParameter MqttPassword = IotWebConfParameter("Contraseña MQTT", "MqttPassword", MqttPasswordValue, STRING_LEN, "password", NULL, "");
IotWebConfParameter MqttTopic = IotWebConfParameter("Topic Base", "MqttTopic", MqttTopicValue, STRING_LEN, "text", NULL, "RIEGAMATICOTEST");

// Para los topics MQTT
String cmndTopic;
String statTopic;
String teleTopic;
String lwtTopic;

// Conexion UDP para la hora
WiFiUDP UdpNtp;

// Manejador del NTP. Cliente red, servidor, offset zona horaria, intervalo de actualizacion.
// FALTA IMPLEMENTAR ALGO PARA CONFIGURAR LA ZONA HORARIA
static NTPClient ClienteNTP(UdpNtp, "europe.pool.ntp.org", HORA_LOCAL * 3600, 600000);

// Para el sensor de temperatura de la CPU. Definir aqui asi necesario es por no estar en core Arduino.
extern "C" {uint8_t temprature_sens_read();}

// Para las Comunicaciones
Comunicaciones MisComunicaciones = Comunicaciones();

// Objeto riegamtico
RiegaMatico* RiegaMatico::sRiegaMatico	 = 0;						// Instancia vacia de la clase (reserva de mem)
RiegaMatico MiRiegaMatico(FICHERO_CONFIG_PRJ, ClienteNTP);

#pragma endregion

#pragma region Funciones de Eventos, CallBack y Auxiliares

// Callback. Manda a la cola de TX el mensaje de respuesta desde la clase RiegaMatico
void MandaRespuesta(String comando, String payload) {

			String t_topic = statTopic + "/" + comando;

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
		CocinaTelemetria(teleTopic + "/INFO1", MiRiegaMatico.MiEstadoJson(1));
		CocinaTelemetria(teleTopic + "/INFO2", MiRiegaMatico.MiEstadoJson(2));
		CocinaTelemetria(teleTopic + "/INFO3", MiRiegaMatico.MiEstadoJson(3));
		CocinaTelemetria(teleTopic + "/INFO4", MiRiegaMatico.MiEstadoJson(4));
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

// Funciones callback del iotWebConf
void handleRoot(){

  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal()){
    // -- Captive portal request were already served.
    return;
  }

  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>RIEGAMATICO MQTT</title></head><body>";
  s += CUSTOMHTML_BODY_INNER;
  s += "Riegamatico MQTT - Diego Maroto - BilbaoMakers 2020";
  s += "<ul>";
  s += "<li>WIFI: ";
  s += WiFi.SSID();
  s += "<li>TOPICBASE: ";
  s += MqttTopicValue;
  s += "<li>CONEXION OK: ";
  s += MisComunicaciones.IsConnected();
  s += "</ul>";
  s += "Acceder a la <a href='config'>pagina de configuracion</a> para cambiar los valores";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void wifiConnected(){

  Serial.println("Evento Conectado a la Wifi");

}

void configSaved(){

  Serial.println("Configuracion recibida via web.");
  
}

boolean formValidator(){
  
  Serial.println("Validando configuracion web");
  
  return true;

}

#pragma endregion

#pragma region TASKS

// Tarea para vigilar la conexion con el MQTT y conectar si no estamos conectados
void TaskGestionRed ( void * parameter ) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5000;
	xLastWakeTime = xTaskGetTickCount ();


	while(true){

		if (iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE){
			
			if ( !MisComunicaciones.IsConnected() ){

				MisComunicaciones.Conectar();

			}

			// Esto teoricamente se ejecuta en loop y solo actualiza en el tiempo que le damos al constructor, pero con esto es suficiente
			ClienteNTP.update();
			
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
	const TickType_t xFrequency = 15000;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){

	
		CocinaTelemetria(teleTopic + "/INFO1", MiRiegaMatico.MiEstadoJson(1));

		CocinaTelemetria(teleTopic + "/INFO2", MiRiegaMatico.MiEstadoJson(2));

		CocinaTelemetria(teleTopic + "/INFO3", MiRiegaMatico.MiEstadoJson(3));

		CocinaTelemetria(teleTopic + "/INFO4", MiRiegaMatico.MiEstadoJson(4));

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		
			
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
					
					/*
					if (COMANDO == "WSsid"){
						
						String(PAYLOAD).toCharArray(MiConfig.Wssid, sizeof(MiConfig.Wssid));
						Serial.println("Wssid OK: " + PAYLOAD);

					}

					else if (COMANDO == "WPasswd"){

						String(PAYLOAD).toCharArray(MiConfig.WPasswd, sizeof(MiConfig.WPasswd));
						Serial.println("Wpasswd OK: " + PAYLOAD);
						
					}
					*/

					if (COMANDO == "MQTTSrv"){

						String(PAYLOAD).toCharArray(MqttServerValue, sizeof(MqttServerValue));
						Serial.println("MQTTSrv OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTUser"){

						String(PAYLOAD).toCharArray(MqttUsuarioValue, sizeof(MqttUsuarioValue));
						Serial.println("MQTTUser OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTPasswd"){

						String(PAYLOAD).toCharArray(MqttPasswordValue, sizeof(MqttPasswordValue));
						Serial.println("MQTTPasswd OK: " + PAYLOAD);

					}

					else if (COMANDO == "MQTTTopic"){

						String(PAYLOAD).toCharArray(MqttTopicValue, sizeof(MqttTopicValue));
						Serial.println("MQTTTopic OK: " + PAYLOAD);

					}
					
					else if (COMANDO == "SaveCom"){

						iotWebConf.configSave();

					}

					else if (COMANDO == "Help"){

						Serial.println("Comandos para la configuracion de las comunicaciones:");
						//Serial.println("WSsid <SSID> - Configurar SSID de la Wifi");
						//Serial.println("WPasswd <Contraseña> - Configurar contraseña de la Wifi ");
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

						Serial.println("Me ha llegado un comando que no entiendo. Para ayuda comando Help.");
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

	// Incializar el objeto Riegamatico con sus cosas.
	MiRiegaMatico.Begin();

	// Callback de la libreria de comunicaciones
	MisComunicaciones.SetEventoCallback(EventoComunicaciones);

	// iotWebConf
	iotWebConf.setStatusPin(STATUS_PIN);
  	  		
	iotWebConf.addParameter(&separator1);
	iotWebConf.addParameter(&MqttServer);
	iotWebConf.addParameter(&MqttUsuario);
	iotWebConf.addParameter(&MqttPassword);
	iotWebConf.addParameter(&MqttTopic);

  	iotWebConf.setConfigSavedCallback(&configSaved);
  	iotWebConf.setFormValidator(&formValidator);
  	iotWebConf.setWifiConnectionCallback(&wifiConnected);
	iotWebConf.getApTimeoutParameter()->visible = false;
	iotWebConf.setupUpdateServer(&httpUpdater);
	
	iotWebConf.skipApStartup();
	
	Serial.println("Arrancando Portal Web de Configuracion");
  	
	// Para la cabecera Javascript
	iotWebConf.setHtmlFormatProvider(&customHtmlFormatProvider);

	boolean validConfig = iotWebConf.init();
  	if (!validConfig){
    
		//stringParamValue[0] = '\0';
		Serial.println("ATENCION: No es posible cargar la configuracion.");
  	
	}

	else{

		cmndTopic = "cmnd/" + String(MqttTopicValue) + "/#";
		statTopic = "stat/" + String(MqttTopicValue);
		teleTopic = "tele/" + String(MqttTopicValue);
		lwtTopic = teleTopic + "/LWT";

		MisComunicaciones.SetMqttServidor(MqttServerValue);
		MisComunicaciones.SetMqttUsuario(MqttUsuarioValue);
		MisComunicaciones.SetMqttPassword(MqttPasswordValue);
		MisComunicaciones.SetMqttClientId(MqttTopicValue);
		MisComunicaciones.SetMqttTopic(MqttTopicValue);

		// Tarea de gestion de la conexion MQTT. Lanzamos solo si conseguimos leer la configuracion
		xTaskCreatePinnedToCore(TaskGestionRed,"MQTT_Conectar",8192,NULL,1,&THandleTaskGestionRed,0);

	}
	
  	// Pasar al servidor web los manejadores de paginas
  	server.on("/", handleRoot);
  	server.on("/config", []{ iotWebConf.handleConfig(); });
  	server.onNotFound([](){ iotWebConf.handleNotFound(); });

	// COLAS
	ColaCMND = xQueueCreate(10,200);
	ColaTX = xQueueCreate(20,300);
	
	// TASKS
	Serial.println("Portal Web OK. Creando tareas del sistema.");
	xTaskCreatePinnedToCore(TaskProcesaComandos,"ProcesaComandos",8192,NULL,1,&THandleTaskProcesaComandos,1);
	xTaskCreatePinnedToCore(TaskTX,"EnviaMQTT",8192,NULL,1,&THandleTaskTX,1);
	xTaskCreatePinnedToCore(TaskEnviaTelemetria,"MandaTelemetria",8192,NULL,1,&THandleTaskEnviaTelemetria,1);
	xTaskCreatePinnedToCore(TaskComandosSerieRun,"ComandosSerieRun",8192,NULL,1,&THandleTaskComandosSerieRun,1);
	xTaskCreatePinnedToCore(TaskRiegaMaticoRun,"RiegaMaticoRun",8192,NULL,1,&THandleTaskRiegaMaticoRun,1);
		
	// Init Completado.
	Serial.println("** Setup Completado. Sistema iniciado **");
		
}

#pragma endregion

#pragma region Funcion Loop() de ARDUINO

// Funcion LOOP de Arduino
// Se ejecuta en el Core 1

void loop() {

	// Esto aqui de momento porque no me rula en una task y tengo que averiguar por que.
	iotWebConf.doLoop();
	ArduinoOTA.handle();
	MiRiegaMatico.RunFast();	
	MisComunicaciones.RunFast();	
	
}

#pragma endregion

/// FIN DEL PROGRAMA ///