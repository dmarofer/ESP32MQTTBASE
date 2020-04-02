#ifndef _ConfigCom
#define _ConfigCom


//#include <string>	
#include <WString.h>
#include <Arduino.h>

class ConfigCom{

	private:

		String c_fichero;
	
	public:
	
		char mqttserver[40];
		char mqttport[6];
		char mqtttopic[33];
		char mqttusuario[19];
		char mqttpassword[19];

		String cmndTopic;
		String statTopic;
		String teleTopic;
		String lwtTopic;

		// Esto no se salva en el fichero, lo hace el objeto Wifi
		// Lo pongo aqui como almacenamiento temporal para los comandos de configuracion
		char Wssid[30];
		char WPasswd[100];

		// Otras configuraciones permanentes del proyecto
		
		ConfigCom(String fichero);
		~ConfigCom() {};

		boolean leeconfig ();
		boolean escribeconfig ();
		
};

#endif