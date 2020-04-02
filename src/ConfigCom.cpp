	
   
    
 #include "ConfigCom.h"
 #include <string>						// Para el manejo de cadenas
 #include <WString.h>
 #include <SPIFFS.h>						// Libreria para sistema de ficheros SPIFFS
 #include <Arduino.h>
 #include <ArduinoJson.h>				// OJO: Tener instalada una version NO BETA (a dia de hoy la estable es la 5.13.4). Alguna pata han metido en la 6
    
// Constructor
ConfigCom::ConfigCom(String fichero){

	c_fichero = fichero;

		mqttserver[0]= '\0';
		mqttport[0] = '\0';
		mqtttopic[0] = '\0';
		mqttusuario[0] = '\0';
		mqttpassword[0] = '\0';
		Wssid[0] = '\0';
		WPasswd[0]  = '\0';	
				
}

// Leer la configuracion desde el fichero
boolean ConfigCom::leeconfig(){
	
	if (SPIFFS.exists(c_fichero)) {
		// Si existe el fichero abrir y leer la configuracion y asignarsela a las variables definidas arriba
		File ComConfigFile = SPIFFS.open(c_fichero, "r");
		if (ComConfigFile) {
			size_t size = ComConfigFile.size();
			// Declarar un buffer para almacenar el contenido del fichero
			std::unique_ptr<char[]> buf(new char[size]);
			// Leer el fichero al buffer
			ComConfigFile.readBytes(buf.get(), size);
			DynamicJsonBuffer jsonBuffer;
			JsonObject& json = jsonBuffer.parseObject(buf.get());
			//json.printTo(Serial);
			
			if (json.success()) {
				Serial.print("Configuracion del fichero leida: ");
				json.printTo(Serial);
			    Serial.println("");
				// Leer los valores del MQTT
				strcpy(mqttserver, json["mqttserver"]);
				strcpy(mqttport, json["mqttport"]);
				strcpy(mqtttopic, json["mqtttopic"]);
				strcpy(mqttusuario, json["mqttusuario"]);
				strcpy(mqttpassword, json["mqttpassword"]);

				// Dar valor a las strings con los nombres de la estructura de los topics
				cmndTopic = "cmnd/" + String(mqtttopic) + "/#";
				statTopic = "stat/" + String(mqtttopic);
				teleTopic = "tele/" + String(mqtttopic);
				lwtTopic = teleTopic + "/LWT";
				return true;

				Serial.println("Servidor MQTT: " + String(mqttserver)) + ":" + String(mqttport);
				Serial.println("Topic Comandos: " + cmndTopic);
				Serial.println("Topic Respuestas: " + statTopic);
				Serial.println("Topic Telemetria: " + teleTopic);
				Serial.println("Topic LWT: " + lwtTopic);
										
			}
					
			else {

				Serial.println("No se puede cargar la configuracion desde el fichero");
				return false;

			}
		}

		else {

			Serial.println ("No se puede leer el fichero de configuracion");
			return false;

		}

	}

	else	{

			Serial.println("No se ha encontrado un fichero de configuracion.");
			Serial.println("Por favor configura el dispositivo desde el terminal serie y reinicia el controlador.");
			Serial.println("Informacion de los comandos con el comando Help");
			return false;

	}

}
	
// Salvar la configuracion en el fichero
boolean ConfigCom::escribeconfig(){

	Serial.println("Salvando la configuracion en el fichero");
	DynamicJsonBuffer jsonBuffer;
	JsonObject& json = jsonBuffer.createObject();
	json["mqttserver"] = mqttserver;
	json["mqttport"] = mqttport;
	json["mqtttopic"] = mqtttopic;
	json["mqttusuario"] = mqttusuario;
	json["mqttpassword"] = mqttpassword;
		
	File ComConfigFile = SPIFFS.open(c_fichero, "w");
	if (!ComConfigFile) {
		Serial.println("No se puede abrir el fichero de configuracion de las comunicaciones");
		return false;
	}

	//json.prettyPrintTo(Serial);
	json.printTo(ComConfigFile);
	ComConfigFile.close();
	Serial.println("Configuracion Salvada");
		
	// Dar valor a las strings con los nombres de la estructura de los topics
	cmndTopic = "cmnd/" + String(mqtttopic) + "/#";
	statTopic = "stat/" + String(mqtttopic);
	teleTopic = "tele/" + String(mqtttopic);
	lwtTopic = teleTopic + "/LWT";

	return true;

}

