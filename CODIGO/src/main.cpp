
#pragma region COMENTARIOS

/*
RIEGAMATICO_MQTT
Controlador de Riego con ESP32 y comunicaciones Wifi MQTT
Pensado para ser gestionado y controlado desde HomeAssistant

Autor: Diego Maroto - BilbaoMakers 2019 - info@bilbaomakers.org

*/

#pragma endregion

#pragma region INCLUDES

#include <SerialCommands.h>			// Libreria para la gestion de comandos por el puerto serie https://github.com/ppedro74/Arduino-SerialCommands
#include <AsyncMqttClient.h>		// Vamos a probar esta que es Asincrona: https://github.com/marvinroger/async-mqtt-client
#include <FS.h>									// Libreria Sistema de Ficheros
#include <WiFi.h>								// Para las comunicaciones WIFI del ESP32
#include <DNSServer.h>					// La necesita WifiManager para el portal captivo
#include <WebServer.h>					// La necesita WifiManager para el formulario de configuracion (ESP32)
#include <WiFiManager.h>				// Para la gestion avanzada de la wifi
#include <ArduinoJson.h>				// OJO: Tener instalada una version NO BETA (a dia de hoy la estable es la 5.13.4). Alguna pata han metido en la 6
#include <string>								// Para el manejo de cadenas
#include <Bounce2.h>						// Libreria para filtrar rebotes de los Switches: https://github.com/thomasfredericks/Bounce2
#include <SPIFFS.h>							// Libreria para sistema de ficheros SPIFFS

#pragma endregion


#pragma region Variables y estructuras


// Estructura para las configuraciones MQTT
struct MQTTCFG
{

	//Valores c_str para conexion al broker MQTT. Si existen posterioirmente en el fichero de configuracion en el SPIFFS se sobreescibiran con esos valores.
	char mqtt_server[40] = "";
	char mqtt_port[6] = "";
	char mqtt_topic[33] = "";
	char mqtt_usuario[19] = "";
	char mqtt_password[19] = "";

	// Variables internas string para los nombres de los topics. Se les da valor luego al final del setup()
	// El de comandos al que me voy a suscribir para "escuchar".
	String cmndTopic;
	// Y estos como son para publicar, defino la raiz de la jerarquia. Luego cuando publique ya añado al topic lo que necesite (por ejemplo tele/AZIMUT/LWT , etc ...)
	String statTopic;
	String teleTopic;

} MiConfigMqtt;


// flag para saber si tenemos que salvar los datos en el fichero de configuracion.
bool shouldSaveConfig = false;

#pragma endregion

#pragma region Objetos

// Wifimanager (aqui para que se vea tambien en el Loop)
WiFiManager wifiManager;

// Para la conexion MQTT
WiFiClient Clientered;
AsyncMqttClient  ClienteMQTT;

// Los manejadores para las tareas
TaskHandle_t THandleTaskMQTTRun,THandleTaskComandosSerieRun,THandleTaskMandaTelemetria,THandleTaskConexionMQTT;	
	

#pragma endregion


#pragma region FUNCIONES

#pragma region funciones de gestion de la configuracion

// Funcion Callback disparada por el WifiManager para que sepamos que hay que hay una nueva configuracion que salvar (para los custom parameters).
void saveConfigCallback() {
	Serial.println("Lanzado SaveConfigCallback");
	shouldSaveConfig = true;
}

// Funcion para leer la configuracion desde el fichero de configuracion
void LeeConfig() {

	// El true es para formatear el sistema de ficheros si falla el montage. Si veo que hace cosas raras mejorar (no hacerlo siempre)
	if (SPIFFS.begin(true)) {
		Serial.println("Sistema de ficheros montado");
		if (SPIFFS.exists("/config.json")) {
			// Si existe el fichero abrir y leer la configuracion y asignarsela a las variables definidas arriba
			Serial.println("Leyendo configuracion del fichero");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile) {
				Serial.println("Fichero de configuracion encontrado");
				size_t size = configFile.size();
				// Declarar un buffer para almacenar el contenido del fichero
				std::unique_ptr<char[]> buf(new char[size]);
				// Leer el fichero al buffer
				configFile.readBytes(buf.get(), size);
				DynamicJsonBuffer jsonBuffer;
				JsonObject& json = jsonBuffer.parseObject(buf.get());
				//json.printTo(Serial);
				if (json.success()) {
					Serial.println("Configuracion del fichero leida");

					// Leer los valores del MQTT
					strcpy(MiConfigMqtt.mqtt_server, json["mqtt_server"]);
					strcpy(MiConfigMqtt.mqtt_port, json["mqtt_port"]);
					strcpy(MiConfigMqtt.mqtt_topic, json["mqtt_topic"]);
					strcpy(MiConfigMqtt.mqtt_usuario, json["mqtt_usuario"]);
					strcpy(MiConfigMqtt.mqtt_password, json["mqtt_password"]);

				}
				else {
					Serial.println("No se puede carcar la configuracion desde el fichero");
				}
			}
		}
	}
	else {

		Serial.println("No se puede montar el sistema de ficheros");
	}


}

// Funcion para Salvar la configuracion en el fichero de configuracion
void SalvaConfig() {

	Serial.println("Salvando la configuracion en el fichero");
	DynamicJsonBuffer jsonBuffer;
	JsonObject& json = jsonBuffer.createObject();
	json["mqtt_server"] = MiConfigMqtt.mqtt_server;
	json["mqtt_port"] = MiConfigMqtt.mqtt_port;
	json["mqtt_topic"] = MiConfigMqtt.mqtt_topic;
	json["mqtt_usuario"] = MiConfigMqtt.mqtt_usuario;
	json["mqtt_password"] = MiConfigMqtt.mqtt_password;

	File configFile = SPIFFS.open("/config.json", "w");
	if (!configFile) {
		Serial.println("No se puede abrir el fichero de configuracion");
	}

	//json.prettyPrintTo(Serial);
	json.printTo(configFile);
	configFile.close();
	Serial.println("Configuracion Salvada");
	//end save

}

#pragma endregion

#pragma region Funciones de implementacion de los comandos disponibles por MQTT

// Maneja un comando con un parametro. De momento salvo necesidad SOLO 1 parametro
void ManejadorComandos(String comando, String parametros) {

	// Y estas son las 3 variables con las que vamos a trabajar


	if (parametros.indexOf(" ") >> 0) {

		Serial.println("Me ha llegado un comando");
		Serial.println("Comando: " + comando);
		Serial.println("Parametro: " + parametros);

		// COMANDO GOTO
		if (comando == "TEST") {

			// COSAS A HACER

		}

		// COMANDO STATUS
		else if (comando == "STATUS") {

			//COSAS A HACER
			
		}

		
	}

	else {

		Serial.println("Me ha llegado un comando con demasiados parametros");

	}

}

#pragma endregion

#pragma region Funciones de gestion de las conexiones Wifi

// Funcion lanzada cuando entra en modo AP
void APCallback(WiFiManager *wifiManager) {
	Serial.println("Lanzado APCallback");
	Serial.println(WiFi.softAPIP());
	Serial.println(wifiManager->getConfigPortalSSID());
}

// Funcion ante un evento de la wifi
void WiFiEventCallBack(WiFiEvent_t event) {
    
		//Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("Conexion WiFi: Conetado. IP: ");
        Serial.println(WiFi.localIP());
        //FALTA: Conectar al MQTT pero NO se puede desde aqui (el Task de la Wifi me manda a tomar por culo por meterme en su terreno)
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("Conexion WiFi: Desconetado");
        //xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
				//xTimerStart(wifiReconnectTimer, 0);
        break;

		default:
				break;

    }
		
}

#pragma endregion

#pragma region Funciones de gestion de las conexiones MQTT

// Manejador del evento de conexion al MQTT
void onMqttConnect(bool sessionPresent) {


	bool susflag = false;
	bool lwtflag = false;


	Serial.println("Conexion MQTT: Conectado");
	
	// Suscribirse al topic de Entrada de Comandos
	if (ClienteMQTT.subscribe(MiConfigMqtt.cmndTopic.c_str(), 2)) {

		// Si suscrito correctamente
		Serial.println("Suscrito al topic " + MiConfigMqtt.cmndTopic);

		susflag = true;				

	}
		
	else { Serial.println("Error Suscribiendome al topic " + MiConfigMqtt.cmndTopic); }

	
	// Publicar un Online en el LWT
	if (ClienteMQTT.publish((MiConfigMqtt.teleTopic + "/LWT").c_str(), 2,true,"Online")){

		// Si llegamos hasta aqui es estado de las comunicaciones con WIFI y MQTT es OK
		Serial.println("Publicado Online en Topic LWT: " + (MiConfigMqtt.teleTopic + "/LWT"));
		
		lwtflag = true;

	}


	if (!susflag || !lwtflag){

		// Si falla la suscripcion o el envio del Online malo kaka. Me desconecto para repetir el proceso.
		ClienteMQTT.disconnect(false);

	}

	else{

		// Si todo ha ido bien, proceso de inicio terminado.
		Serial.println("Controlador Azimut Iniciado Correctamente: ComOK");

	}


}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  
  Serial.println("Conexion MQTT: Desconectado");

  if (WiFi.isConnected()) {
    
	  ClienteMQTT.connect();
	
  }

}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.print("Subscripcion Realizada. PacketID: ");
  Serial.println(packetId);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.print("Subscricion Cancelada. PacketID: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  
  String s_topic = String(topic);

  // Sacamos el prefijo del topic, o sea lo que hay delante de la primera /
	int Indice1 = s_topic.indexOf("/");
	String Prefijo = s_topic.substring(0, Indice1);
	

	// Sacamos el "COMANDO" del topic, o sea lo que hay detras de la ultima /
	int Indice2 = s_topic.lastIndexOf("/");
	String Comando = s_topic.substring(Indice2 + 1);
		
	// Si el prefijo es cmnd se lo mandamos al manejador de comandos
	if (Prefijo == "cmnd") { ManejadorComandos(Comando, payload); }

}

void onMqttPublish(uint16_t packetId) {
  
	// Al publicar no hacemos nada de momento.

}


// Devuelve al topic correspondiente la respuesta a un comando. Esta funcion la uso como CALLBACK para el objeto cupula
void MandaRespuesta(String comando, String respuesta) {
	
	ClienteMQTT.publish((MiConfigMqtt.statTopic + "/" + comando).c_str(), 2, false, respuesta.c_str());
}


// envia al topic tele la telemetria en Json
void MandaTelemetria() {
	
	if (ClienteMQTT.connected()){

		Serial.println("Enviando Telemetria");

		ClienteMQTT.publish((MiConfigMqtt.teleTopic + "/INFO1").c_str(),2, false, "TELEMETRIA1");
		ClienteMQTT.publish((MiConfigMqtt.teleTopic + "/INFO2").c_str(),2, false, "TELEMETRIA2");

	}

	
}

#pragma endregion

#pragma region Funciones de implementacion de los comandos disponibles por el puerto serie



// Manejadores de los comandos. Aqui dentro lo que queramos hacer con cada comando.
void cmd_WIFI_hl(SerialCommands* sender)
{

	char* parametro1 = sender->Next();
	if (parametro1 == NULL)
	{
		sender->GetSerial()->println("SSID: " + WiFi.SSID() + " Password: " + WiFi.psk());
		return;
	}

	char* parametro2 = sender->Next();
	if (parametro2 == NULL)
	{
		sender->GetSerial()->println("SSID: " + WiFi.SSID() + " Password: " + WiFi.psk());
		return;
	}

	char buffer_ssid[30];
	char buffer_passwd[100];

	String(parametro1).toCharArray(buffer_ssid, sizeof(buffer_ssid));
	String(parametro2).toCharArray(buffer_passwd, sizeof(buffer_passwd));

	sender->GetSerial()->println("Conectando a la Wifi. SSID: " + String(buffer_ssid) + " Password: " + String(buffer_passwd));

	WiFi.begin(buffer_ssid, buffer_passwd);

}


void cmd_MQTTSrv_hl(SerialCommands* sender)
{

	char* parametro = sender->Next();
	if (parametro == NULL)
	{
		sender->GetSerial()->println("MQTTSrv: " + String(MiConfigMqtt.mqtt_server));
		return;
	}

	strcpy(MiConfigMqtt.mqtt_server, parametro);

	sender->GetSerial()->println("MQTTSrv: " + String(parametro));
}


void cmd_MQTTUser_hl(SerialCommands* sender)
{

	char* parametro = sender->Next();
	if (parametro == NULL)
	{
		sender->GetSerial()->println("MQTTUser: " + String(MiConfigMqtt.mqtt_usuario));
		return;
	}

	strcpy(MiConfigMqtt.mqtt_usuario, parametro);

	sender->GetSerial()->println("MQTTUser: " + String(parametro));
}


void cmd_MQTTPassword_hl(SerialCommands* sender)
{

	char* parametro = sender->Next();
	if (parametro == NULL)
	{
		sender->GetSerial()->println("MQTTPassword: " + String(MiConfigMqtt.mqtt_password));
		return;
	}

	strcpy(MiConfigMqtt.mqtt_password, parametro);

	sender->GetSerial()->println("MQTTPassword: " + String(parametro));
}


void cmd_MQTTTopic_hl(SerialCommands* sender)
{

	char* parametro = sender->Next();
	if (parametro == NULL)
	{
		sender->GetSerial()->println("MQTTTopic: " + String(MiConfigMqtt.mqtt_topic));
		return;
	}

	strcpy(MiConfigMqtt.mqtt_topic, parametro);

	sender->GetSerial()->println("MQTTTopic: " + String(parametro));
}


void cmd_SaveConfig_hl(SerialCommands* sender)
{

	SalvaConfig();

}


void cmd_Prueba_hl(SerialCommands* sender)
{

	ManejadorComandos("STATUS", "NADA");

}


// Manejardor para comandos desconocidos
void cmd_error(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("ERROR: No se reconoce el comando [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

// Objetos Comandos
SerialCommand cmd_WIFI("WIFI", cmd_WIFI_hl);
SerialCommand cmd_MQTTSrv("MQTTSrv", cmd_MQTTSrv_hl);
SerialCommand cmd_MQTTUser("MQTTUser", cmd_MQTTUser_hl);
SerialCommand cmd_MQTTPassword("MQTTPassword", cmd_MQTTPassword_hl);
SerialCommand cmd_MQTTTopic("MQTTTopic", cmd_MQTTTopic_hl);
SerialCommand cmd_SaveConfig("SaveConfig", cmd_SaveConfig_hl);
SerialCommand cmd_Prueba("Prueba", cmd_Prueba_hl);

// Buffer para los comandos. Gordote para el comando Wifi que lleva parametros gordos.
char serial_command_buffer_[120];

// Bind del objeto con el puerto serie usando el buffer
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


#pragma endregion

#pragma region TASKS

// Tarea para vigilar la conexion con el MQTT y conectar si no estamos conectados
void TaskConexionMQTT( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5000;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){

		if (WiFi.isConnected() && !ClienteMQTT.connected()){

			ClienteMQTT.connect();
			
		}
		
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}


// Tarea para "atender" a los mensajes MQTT. Hay que ver tambien cuan rapido podemos ejecutarla
void TaskMQTTRun( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){

		//ClienteMQTT.loop();

		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}

}

// Tarea para los comandos que llegan por el puerto serie
void TaskComandosSerieRun( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){

		serial_commands_.ReadSerial();

		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
	
}

// tarea para el envio periodico de la telemetria
void TaskMandaTelemetria( void * parameter ){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10000;
	xLastWakeTime = xTaskGetTickCount ();

	while(true){


		MandaTelemetria();

		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
	
}


#pragma endregion



#pragma region Funcion Setup() de ARDUINO

// funcion SETUP de Arduino
void setup() {

#pragma region Inicializar cosas varias

	// Puerto Serie
	Serial.begin(115200);
	Serial.println();
	
	// Añadir los comandos al objeto manejador de comandos serie
	serial_commands_.AddCommand(&cmd_WIFI);
	serial_commands_.AddCommand(&cmd_MQTTSrv);
	serial_commands_.AddCommand(&cmd_MQTTUser);
	serial_commands_.AddCommand(&cmd_MQTTPassword);
	serial_commands_.AddCommand(&cmd_MQTTTopic);
	serial_commands_.AddCommand(&cmd_SaveConfig);
	serial_commands_.AddCommand(&cmd_Prueba);
	
	// Manejador para los comandos Serie no reconocidos.
	serial_commands_.SetDefaultHandler(&cmd_error);

	
	// Inicializacion de las GPIO


	// Leer la configuracion que hay en el archivo de configuracion config.json
	Serial.println("Leyendo fichero de configuracion");
	LeeConfig();
	   	
#pragma endregion		   

#pragma region Configuracion e inicializacion de la WIFI

	// Añadir al wifimanager parametros para el MQTT
	WiFiManagerParameter custom_mqtt_server("server", "mqtt server", MiConfigMqtt.mqtt_server, 40);
	WiFiManagerParameter custom_mqtt_port("port", "mqtt port", MiConfigMqtt.mqtt_port, 5);
	WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", MiConfigMqtt.mqtt_topic, 34);
	WiFiManagerParameter custom_mqtt_usuario("usuario", "mqtt user", MiConfigMqtt.mqtt_usuario, 20);
	WiFiManagerParameter custom_mqtt_password("password", "mqtt password", MiConfigMqtt.mqtt_password, 20);

	// Configurar el WiFiManager

	// Borrar la configuracion SSID y Password guardadas en EEPROM (Teoricamente esto hay que hacer despues de hace autoconect no se si esta bien aqui esto)
	//wifiManager.resetSettings();
	wifiManager.setDebugOutput(false);

	// Definirle la funcion para aviso de que hay que salvar la configuracion
	wifiManager.setSaveConfigCallback(saveConfigCallback);

	// Definirle la funcion que se dispara cuando entra en modo AP
	wifiManager.setAPCallback(APCallback);

	WiFi.onEvent(WiFiEventCallBack);

	// Lo hago directamente en el objeto WIFI porque a traves del Wifimanager no puedo darle el DNS para que me resuelva cositas.
	// WiFi.config(_ip, _gw, _sn, _dns, _dns);
	// Y asi pilla por DHCP
	WiFi.begin();
    
	// Añadir mis parametros custom

	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);
	wifiManager.addParameter(&custom_mqtt_topic);
	//wifiManager.addParameter(&custom_mqtt_usuario);
	//wifiManager.addParameter(&custom_mqtt_password);

	// Definir Calidad de Señal Minima para mantenerse conectado.
	// Por defecto sin parametros 8%
	wifiManager.setMinimumSignalQuality();

	// Definir la salida de Debug por el serial del WifiManager
	wifiManager.setDebugOutput(false);
	
	// Timeout para que si no configuramos el portal AP se cierre
	wifiManager.setTimeout(300);
	
	// Leer los parametros custom que tiene el wifimanager por si los he actualizado yo en modo AP
	strcpy(MiConfigMqtt.mqtt_server, custom_mqtt_server.getValue());
	strcpy(MiConfigMqtt.mqtt_port, custom_mqtt_port.getValue());
	strcpy(MiConfigMqtt.mqtt_topic, custom_mqtt_topic.getValue());

	// Salvar la configuracion en el fichero de configuracion
	if (shouldSaveConfig) {

		SalvaConfig();
	}

#pragma endregion

#pragma region Configuracion MQTT

	// Dar valor a las strings con los nombres de la estructura de los topics
	MiConfigMqtt.cmndTopic = "cmnd/" + String(MiConfigMqtt.mqtt_topic) + "/#";
	MiConfigMqtt.statTopic = "stat/" + String(MiConfigMqtt.mqtt_topic);
	MiConfigMqtt.teleTopic = "tele/" + String(MiConfigMqtt.mqtt_topic);
	
	ClienteMQTT.onConnect(onMqttConnect);
  ClienteMQTT.onDisconnect(onMqttDisconnect);
  ClienteMQTT.onSubscribe(onMqttSubscribe);
  ClienteMQTT.onUnsubscribe(onMqttUnsubscribe);
  ClienteMQTT.onMessage(onMqttMessage);
  ClienteMQTT.onPublish(onMqttPublish);
  ClienteMQTT.setServer(MiConfigMqtt.mqtt_server, 1883);
	ClienteMQTT.setCleanSession(true);
	ClienteMQTT.setClientId("ControlAzimut");
	ClienteMQTT.setCredentials(MiConfigMqtt.mqtt_usuario,MiConfigMqtt.mqtt_password);
	ClienteMQTT.setKeepAlive(2);
	//ClienteMQTT.setWill("(MiConfigMqtt.teleTopic + String("/LTW")).c_str()",2,true,"Offline");
	//ClienteMQTT.setWill(strcat("tele/AZIMUT", "/LTW"),2,true,"Offline");
	
	// Parar un par de segundos antes de lanzar las tareas.
	delay(2000);


#pragma endregion
	
#pragma region TASKS 

	// Lanzar las tareas infinitas a los cores a traves de las funciones del FreeRTOS
	
	// xTaskCreate(TareaCore0,"CORE0",1000,NULL,1,&HandleTareaCore0)
	// xTaskCreatePinnedToCore(TareaCore0,"CORE0",1000,NULL,1,&HandleTareaCore0,0)

	Serial.println("Creando tareas ...");
	
	// Tareas CORE0
	xTaskCreatePinnedToCore(TaskConexionMQTT,"MQTT_Conectar",2000,NULL,1,&THandleTaskConexionMQTT,0);
	xTaskCreatePinnedToCore(TaskMQTTRun,"MQTTRun",2000,NULL,1,&THandleTaskMQTTRun,0);
	xTaskCreatePinnedToCore(TaskMandaTelemetria,"MandaTelemetria",2000,NULL,1,&THandleTaskMandaTelemetria,0);
	xTaskCreatePinnedToCore(TaskComandosSerieRun,"ComandosSerieRun",2000,NULL,1,&THandleTaskComandosSerieRun,0);
	
	// Tareas CORE1. Si no es necesario tareas, utilizar el loop() que corre en el CORE1 como una unica tarea

	Serial.println("Sistema Iniciado");

#pragma endregion

}

#pragma endregion


#pragma region Funcion Loop() de ARDUINO

// Funcion LOOP de Arduino
void loop() {

	

}

#pragma endregion


/// FIN DEL PROGRAMA ///