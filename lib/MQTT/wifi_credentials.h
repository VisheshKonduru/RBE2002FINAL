#include <Arduino.h>

#idef __HOME_NETWORK__
// if working off campus, update these values
// suitable for your network., then change in .ini file accordingly
const char* ssid ...;
const char* password ...;

#elif defined __USE_RBE_NETWORK__
//RBE
const char* ssid = "RBE";
const char* password = "elm69wisest16poisoned";

#endif 

// robomqtt credientials
const char* mqtt_server = "robomqtt.wpi.edu";
#define mqtt_port 1883
#define MQTT_USER <username>
#define MQTT_PASSWORD <password>