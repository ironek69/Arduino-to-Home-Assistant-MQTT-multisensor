/* 
 ====== MultiSensorsJolka by Iron ======
 ======      Testing platform     ======
 Arduino Mega 2560 + Ethernet Shield
 ====== Changelog ======
 v0.1.1
 - Odczyt na żądanie tylko 
 v0.1.0
 - Odczyt ze zdalnych czujników przez RS485;
 v0.0.9
 - optymalizacja programu.;
 v0.0.8
 - optymalizacja programu.;
 v0.0.7
 - poprawa kilku features związanych z nowym HA 105;
 v0.0.6
 - autoDiscovery Home Assistant;
 v0.0.5
 - zmiana precyzji do 1 mijsca po przecinku;
 - dodanie zmiennej int SerialPrint = 0 - wyłącza debug do portu szeregowego, int WebServer = 0 - wyłącza WebServer;
 v0.0.4
 - dodanie webservera do Arduino
 - odczyt ostatniego stanu przekaźników z homeassistant
 v0.0.3
 - sterowanie dwoma przekaźnikami "arduino01/cmnd/POWER1" ON/OFF;
 - wysyłanie statusu Arduino "arduino01/tele/LWT" Aktywny/Nieaktywny;
 - ustawianie statusu przekaźników “arduino01/stat/POWER1” po odbiorze komendy ON / OFF wykorzystywane do ustalenia ostatniego stanu przekaźników;
 - porządek w topicach według struktury by tasmota: "arduino01/tele/SENSORtemperature", "arduino01/tele/SENSORhumidity", "arduino01/tele/SENSORdoor", "arduino01/tele/SENSORlight", 
 - wysyłanie statusu do HASS "arduino01/tele/HASS_STATE" z wersją projektu;
 v0.0.2
 - odczyt wilgotności co 1% a nie co 0.10 - ustawienie można zmienić w float diffhum = 1.0; // częstotliwość odswierzania co 1 %, float diff = 0.1; - tempratura co 0.1 stopień
 - dodanie obsługi kontaktronu (Kontaktron podpinamy pod PIN 3 Arduino, drugi przewód do GND. - #define KONTAKTRON 3)
 - wysyłanie statusu otwarte/zamknięte w topicu "arduino01/door"
 - dodanie obsługi czujnika/detektora światła LM393 -  Czujnik podpiamy do Arduino do Pinu 5 oraz 5V i GND;
 - czujnik ma on tylko stan 0 lub 1 (jasno/ciemno) ale potencjometrem można sterować czułość przy jakim natężeniu światła czujnik zadziała.
 v0.0.1
 - dodanie obsługi czujnika temperatury i wilgotności AM2302 lub DHT21, DHT22 (Czujnik podłączamy pod 2 pin Arduino oraz 5V i GND - #define DHTPIN 2) #define DHTTYPE DHT22 //21 or 22 w zaleznosci od czujnika
 - odczyt temperatury i wilgotności i publiowanie w topicu arduino01/tempertura, arduino01/humidity;
 - przesyłanie tylko różnicy wartości wcześniej odczytej
 ====== ToDo List ======
 - dodanie kolejnych czujników np. Miernik energii, detektor gazu/czadu itp…
 - zapis loga na kartę MicroSD w Ethernet Shield;
 ================================================================================================================================================================================================*/
#include <SPI.h>
#include <Ethernet.h>
#define MQTT_KEEPALIVE 10
//#define MQTT_VERSION MQTT_VERSION_3_1_1
#undef  MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 1000
#include <PubSubClient.h>
#include <DHT.h>
#include "RS485_protocol.h"
#include <SoftwareSerial.h>
#define SoftVersion "MultiSensorsJolka v0.1.0 by Iron"
#define SoftBuildDate "2020-02-29T20:04:00"
#define ModuleType "Arduino Mega 2560+Ethernet Shield"
//#define sensors_topic "dom-88fd54db6487cd19_arduino01/tele/SENSOR"
//#define sensors_topic "homeassistant/sensor/temp_room_1/state"
#define sensors1_topic "dom-88fd54db6487cd19_arduino01/tele/SENSOR1"
#define sensors2_topic "dom-88fd54db6487cd19_arduino01/tele/SENSOR2"
#define sensors3_topic "dom-88fd54db6487cd19_arduino01/tele/SENSOR3"
#define contactron_topic "dom-88fd54db6487cd19_arduino01/tele/SENSORdoor01"
#define light_topic "dom-88fd54db6487cd19_arduino01/tele/SENSORlight01"
#define availability_topic "dom-88fd54db6487cd19_arduino01/tele/LWT" // Aktywny / Nieaktywny
#define state_topic "dom-88fd54db6487cd19_arduino01/tele/HASS_STATE"
#define statestatus_topic "dom-88fd54db6487cd19_arduino01/stat/STATUS"
#define statepower1_topic "dom-88fd54db6487cd19_arduino01/tele/POWER1" // ON/OFF
#define statepower2_topic "dom-88fd54db6487cd19_arduino01/tele/POWER2" // ON/OFF
#define command1_topic "dom-88fd54db6487cd19_arduino01/cmnd/POWER1"
#define command2_topic "dom-88fd54db6487cd19_arduino01/cmnd/POWER2"
#define autodiscovery_send_topic "dom/cmnd/SetOption19"
#define autodiscovery_light_topic "homeassistant/light/arduino01/light/config"
#define autodiscovery_switch1_topic "homeassistant/switch/arduino01_RL_1/config"
#define autodiscovery_switch2_topic "homeassistant/switch/arduino01_RL_2/config"
#define autodiscovery_binary1_topic "homeassistant/binary_sensor/arduino01_BTN_1/config"
#define autodiscovery_binary2_topic "homeassistant/binary_sensor/arduino01_BTN_2/config"

#define autodiscovery_sensor1_topic "homeassistant/sensor/arduino01_temperature/config"
#define autodiscovery_sensor2_topic "homeassistant/sensor/arduino01_humidity/config"
#define autodiscovery_sensor3_topic "homeassistant/sensor/arduino01_temperature1/config"
#define autodiscovery_sensor4_topic "homeassistant/sensor/arduino01_humidity1/config"
#define autodiscovery_sensor5_topic "homeassistant/sensor/arduino01_Light02/config"
#define autodiscovery_status_topic "homeassistant/sensor/arduino01_status/config"
#define sensorpom1 "Salon" // 1
#define sensorpom2 "Pokój Ani" // 2
#define DHTPIN 8 // 8 Pin Arduino do ktorego podlaczony jest czujnik
//#define RS485 9 // 9 Pin Arduino do ktorego podlaczony jest konwerter RS485
const byte ENABLE_PIN = 25; //D48

#define DHTTYPE DHT22 //21 or 22 w zaleznosci od czujnika lub AM2302 = DTH22
#define KONTAKTRON 3 //3 pin Arduino do kontaktronu
#define CZUJNIKSWIATLA 5 //5 pin Arduino do czujnika światła
#define PRZEKAZNIK1 6 //6 pin Arduino do przekaźnika 1
#define PRZEKAZNIK2 7 //7 pin Arduino do przekaźnika 2
#define PRZEKAZNIK3 8 //8 pin Arduino do przekaźnika 3
#define PRZEKAZNIK4 9 //9 pin Arduino do przekaźnika 4
//USE PIN Mega Ethernet Shield 0,1, 4,10, 50,51,52,53
//USE PIN UNO  Ethernet Shield 0,1, 4,10, 11,12,13

int SerialPrint = 1; // 0 - nie wysyłaj nic do portu szeregowego, 1 - debug log do portu szeregowego
bool WebServer = false;   // false - nie uruchamiaj WebServera, true - uruchom webserver
bool AutoRestart = false; // false - bez restaru, true1 - autorestart
int RestartTime = 1800000; // 1800000 = 30 minut
volatile byte relayState = LOW;

DHT dht(DHTPIN, DHTTYPE);
unsigned long readTime;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xAD };
IPAddress ip(192, 168, 2, 225); // Adres IP Arduino
IPAddress server(192, 168, 2, 1); // Adres IP bramki z serwerem MQTT
char message_buff[100];
String AddIP;
String AddMAC;
String przekaznik1stan;
String przekaznik2stan;
int startprg = 1;
long MQTTread;

void callback(char* topic, byte* payload, unsigned int length);
EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);
EthernetServer serverweb(80);
SoftwareSerial rs485 (50, 48);  // receive pin, transmit pin 

/*Register Map:
byte  Desc 
 0    status (slave status)
 1    sensor Id
 2    temperature (float)
 3    temperature
 4    temperature
 5    temperature
 6    humidity (float)
 7    humidity (float)
 8    humidity (float)
 9    humidity (float)
 10   pressure (float)
 11   pressure (float)
 12   pressure (float)
 13   pressure (float)
 14   altitude (float)
 15   altitude (float)
 16   altitude (float)
 17   altitude (float)
 18   set temperature (float)
 19   set temperature (float)
 20   set temperature (float)
 21   set temperature (float)
 22   light (int)
 23   fire (int)
 24   lamp (int)
 */
typedef struct sensorData_t {
  byte stat;
  byte sensorId;
  float temp;
  float hum;
  float pressure;
  float alt;
  float settemp;
  byte light;
  byte fire;
  byte lamp;
};
#define PACKET_SIZE sizeof(sensorData_t)

typedef union rs485_Packet_t{
 sensorData_t sensor;
 byte rs485Packet[sizeof(sensorData_t)];
};

rs485_Packet_t leakinfo;  

void callback(char* topic, byte* payload, unsigned int length) {
  String payloadmsg;
  for (int i=0;i<length;i++) {
    payloadmsg += (char)payload[i];
  }
 if (SerialPrint == 1) {
        Serial.print("topic received = ");
        Serial.println(topic);}
  
  // Pierwsze włączenie arduino - odczyt stanów przekaźników z HA
  if (startprg == 1 || startprg == 2){
    // Przekaźnik 1   
    if (strcmp (topic, statepower1_topic) == 0) {
      startprg = 2;
      MQTTread += 1;
      if (SerialPrint == 1) {
        Serial.print("Stan przekaźnika 1 = ");
        Serial.println(payloadmsg);}
      if (payloadmsg == "ON") {
        przekaznik1stan = "ZAŁĄCZONY";
        digitalWrite(PRZEKAZNIK1, LOW); // Załącz przekaźnik 1
      }else {
        przekaznik1stan = "WYŁĄCZONY";
        digitalWrite(PRZEKAZNIK1, HIGH); // Wyłącz przekaźnik 1
      }
    } // End Przekaźnik 1

    // Przekaźnik 2   
    if (strcmp (topic, statepower2_topic) == 0) {
      startprg = 3;
      MQTTread += 1;
      if (SerialPrint == 1) { Serial.print("Stan przekaźnika 2 = ");
        Serial.println(payloadmsg);}
      if (payloadmsg == "ON") {
        przekaznik2stan = "ZAŁĄCZONY";
        digitalWrite(PRZEKAZNIK2, LOW); // Załącz przekaźnik 2
      }else {
        przekaznik2stan = "WYŁĄCZONY";
        digitalWrite(PRZEKAZNIK2, HIGH); // Wyłącz przekaźnik 2
      }
    } // End Przekaźnik 2
  }
    // Zalaczenie przekaznika 1   
    if (strcmp (topic, command1_topic) == 0) {
      MQTTread += 1;
      if (SerialPrint == 1) {Serial.print("Przekaźnik 1 = ");}
      if (payloadmsg == "ON") {
        przekaznik1stan = "ZAŁĄCZONY";
        if (SerialPrint == 1) {Serial.println("ZAŁĄCZ");}
        digitalWrite(PRZEKAZNIK1, LOW); // Załącz przekaźnik 1
        client.publish(statepower1_topic, "ON", true);
      }
      if (payloadmsg == "OFF") {
        przekaznik1stan = "WYŁĄCZONY";
        if (SerialPrint == 1) {Serial.println("WYŁĄCZ");}
        digitalWrite(PRZEKAZNIK1, HIGH); // Wyłącz przekaźnik 1
        client.publish(statepower1_topic, "OFF", true);    
      }
    }
  
    // Zalaczenie przekaznika 2
    if (strcmp (topic, command2_topic) == 0) {
      MQTTread += 1;
      if (SerialPrint == 1) {Serial.print("Przekaźnik 2 = ");}
      if (payloadmsg == "ON") {
        przekaznik2stan = "ZAŁĄCZONY";
        if (SerialPrint == 1) {Serial.println("ZAŁĄCZ");}
        digitalWrite(PRZEKAZNIK2, LOW); // Załącz przekaźnik 2
        client.publish(statepower2_topic, "ON", true);    
      }
      if (payloadmsg == "OFF") {
        przekaznik2stan = "WYŁĄCZONY";
        if (SerialPrint == 1) {Serial.println("WYŁĄCZ");}
        digitalWrite(PRZEKAZNIK2, HIGH); // Wyłącz przekaźnik 2
        client.publish(statepower2_topic, "OFF", true);    
      }
    }

      // Autodiscovery po starcie HA
  if (strcmp (topic, autodiscovery_send_topic) == 0) { autodiscoveryHA(0); }
}// end callback

void reconnect() {
  // Oczekowani na polaczenie z serwerem
  while (!client.connected()) {
    if (SerialPrint == 1) {Serial.print("Attempting MQTT connection...");}
    // Attempt to connect
    if (client.connect("dom-88fd54db6487cd19_arduino01","DVES_USER","","dom-88fd54db6487cd19_arduino01/tele/LWT",1,true,"Nieaktywny")) {
      if (SerialPrint == 1) {Serial.println("connected");}
      autodiscoveryHA(1); // Przedstawienie się autodiscovery przy starcie
      SendSensorStatus(); // Send status to HA
      boolean rc1 = client.subscribe(command1_topic); // Oczykiwanie na topic dla przekaźnika 1
      boolean rc2 = client.subscribe(command2_topic); // Oczykiwanie na topic dla przekaźnika 2
      boolean rcst1 = client.subscribe(statepower1_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 1
      boolean rcst2 = client.subscribe(statepower2_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 2
      boolean rcst3 = client.subscribe(autodiscovery_send_topic); // SetOption19 

      if (rc1+rc2+rcst1+rcst2+rcst3 == 5){
        Serial.println("Subscribe OK");
        }
      else {
        Serial.println("Subscribe ERROR");
      }
    } else {
      if (SerialPrint == 1) {Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");}
      // Czekaj 5 sekund przed ponownym polaczeniem
      delay(500);
    }
  }
}// end reconnect

void fWrite (const byte what)
  {
  rs485.write (what);  
  }
  
int fAvailable ()
  {
  return rs485.available ();  
  }

int fRead ()
  {
  return rs485.read ();  
  }
  
void setup()
{
  if (SerialPrint == 1) {Serial.begin(9600);}
  client.setServer("192.168.2.1", 1883); // Adres serwera MQTT - bramka
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  if (WebServer){  serverweb.begin();}
  dht.begin(); // Start sensor DHT
  pinMode(KONTAKTRON, INPUT_PULLUP); //Kontaktron jako wejście
  pinMode(CZUJNIKSWIATLA, INPUT); // Czujnik światła
  pinMode(PRZEKAZNIK1, OUTPUT); // Ustawienie Pinu przekaznika 1 jako Wyjscie
  pinMode(PRZEKAZNIK2, OUTPUT); // Ustawienie Pinu przekaznika 2 jako Wyjscie
  digitalWrite(PRZEKAZNIK1, HIGH); // Wyłacz przekaźnik 1 przy starcie , LOW = włącz
  digitalWrite(PRZEKAZNIK2, HIGH); // Wyłacz przekaźnik 2 przy starcie , LOW = włącz       

  rs485.begin (9600);
  pinMode(ENABLE_PIN, OUTPUT);
  delay(10);

  if (SerialPrint == 1) {Serial.println(Ethernet.localIP());} //Wyswietlenie adresu IP Arduino
  AddIP = String(Ethernet.localIP()).c_str();
  byte macBuffer[6];  // create a buffer to hold the MAC address
  //Ethernet.MACAddress(macBuffer); // fill the buffer
  Ethernet.MACAddress(macBuffer); //Wyswietlenie MAC Arduino
  for (byte octet = 0; octet < 6; octet++) {
    AddMAC += macBuffer[octet], HEX;
    if (SerialPrint == 1) {Serial.print(macBuffer[octet], HEX);}
    if (octet < 5) {
      if (SerialPrint == 1) {Serial.print('-');}
    }
  }
  if (SerialPrint == 1) {Serial.println();}
  readTime = 0;
} // End setup loop

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}
  long lastMsg = 0;
  long lastMsg1 = 0;
  long lastMsg2 = 0;
  long lastMsg3 = 0;
  float temp = 0.0;
  float hum = 0.0;
  float diffhum = 1.0; // częstotliwość odswierzania co 1 %
  float diff = 0.1;
  int newKONT = 0; //
  int KONT = 2; // stan 2 przy pierwszym odczycie
  int newlight = 0; //
  int light = 2; // stan 2 przy pierwszym odczycie
  int fire = 0; // fire alarm
  int changeTEHU = 0; // wyslij stan sensorów po zmianie
  int pwmval = 0;
  int numsensors = 1; // # of remote sensors
  long rcverr1 = 0; // error received from sensors # of sensors

void loop(){
  if (!client.connected()) {reconnect();}
  client.loop(); 
  changeTEHU = 0;
  long czekaj;
  float newTemp = 0.0;
  float newHum  = 0.0;
  String msgSendMQT;
  String msgSendMQT1;
  String msgSendMQT2;
  String msgSendMQT3;
  String msgSendMQT4;
  String msgSendMQT5;
  String msgSendMQT6;
  String msgSendMQT7;
  int rs485Status = 0;
  
  // Send status to HA every 1 min
  long now1 = millis();
  if (now1 - lastMsg1 > 60000) {
    lastMsg1 = now1;
    SendSensorStatus(); // Send ststus to HA
    //if (SerialPrint == 1) {Serial.print("Free RAM: "); Serial.println(freeRam());}
    //if (SerialPrint == 1) {Serial.print("Minutes : "); Serial.println(millis()*0.000016666666666667);}
 } // Send Status do HA every 1 min

 
  // Read local sensors every 5 seconds
    long now2 = millis();
  if (now2 - lastMsg2 > 5000) {
    lastMsg2 = now2;
   
  // GET data from remote sensors
  rs485Status = 0;
  getData(1); // Send Request to get sensor 1 Data
  Serial.println("Send invitation to sensor 1...");
  rs485Status = receiveData(); // Receive Data from sensor 1
  if (rs485Status == sizeof(sensorData_t))
  {
    rcverr1 = 0;
    Serial.println("---SENSOR 1 ---");
    Serial.print("Sensor stat: ");
    Serial.println(leakinfo.sensor.stat); 
    Serial.print("Sensor ID: ");
    Serial.println(leakinfo.sensor.sensorId); 
    Serial.print("Temperatura: ");
    Serial.println(leakinfo.sensor.temp);
    Serial.print("Wilgotność: ");
    Serial.println(leakinfo.sensor.hum);
    Serial.print("Ciśnienie: ");
    Serial.println(leakinfo.sensor.pressure);
    Serial.print("Wysokość: ");
    Serial.println(leakinfo.sensor.alt);
    Serial.print("Światło: ");
    Serial.println(leakinfo.sensor.light);
    Serial.print("Temp. w pom: ");
    Serial.println(leakinfo.sensor.settemp);    
    Serial.print("Fire alert: ");
    Serial.println(leakinfo.sensor.fire);
    Serial.print("Lamp: ");
    Serial.println(leakinfo.sensor.lamp);
    msgSendMQT = String(leakinfo.sensor.temp);
    msgSendMQT1 = String(leakinfo.sensor.hum);
    msgSendMQT2 = String(leakinfo.sensor.pressure);
    msgSendMQT3 = String(leakinfo.sensor.alt);
    msgSendMQT4 = String(leakinfo.sensor.light);
    msgSendMQT5 = String(leakinfo.sensor.settemp);
    if (leakinfo.sensor.fire==1){msgSendMQT6 = String("on");}
    else {msgSendMQT6 = String("off");}
    msgSendMQT7 = String(leakinfo.sensor.lamp);
    client.publish("homeassistant/sensor/arduino01_temperature1/state", String(msgSendMQT).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_humidity1/state", String(msgSendMQT1).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_pressure1/state", String(msgSendMQT2).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_altitude1/state", String(msgSendMQT3).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_light1/state", String(msgSendMQT4).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_temperaturepom1/state", String(msgSendMQT5).c_str(), true);
    client.publish("homeassistant/binary_sensor/arduino01_firepom1/state", String(msgSendMQT6).c_str(), true);
    client.publish("homeassistant/binary_sensor/arduino01_lamppom1/state", String(msgSendMQT7).c_str(), true);
        
  }else{
    rcverr1++;
    Serial.print("Sensor 1 receive errors: ");
    Serial.println(rcverr1);
    if (rcverr1 >= 10) {
      Serial.println(F("No packet received from sensor 1"));
      client.publish("homeassistant/sensor/arduino01_temperature1/state","niedostępny",true);
      client.publish("homeassistant/sensor/arduino01_humidity1/state","niedostępny",true);
      client.publish("homeassistant/sensor/arduino01_pressure1/state","niedostępny",true);
      client.publish("homeassistant/sensor/arduino01_altitude1/state","niedostępny",true);
      client.publish("homeassistant/sensor/arduino01_light1/state","niedostępny",true);
      client.publish("homeassistant/binary_sensor/arduino01_firepom1/state","niedostępny",true);
      client.publish("homeassistant/binary_sensor/arduino01_lamppom1/state","niedostępny",true);
      client.publish("homeassistant/sensor/arduino01_temperaturepom1/state","niedostępny",true);  
      delay(1000);
      rcverr1 = 0;
    }
  }

    // READ local sensors 
    newTemp = dht.readTemperature();
    newHum  = dht.readHumidity();
    int newKONT = digitalRead(KONTAKTRON);
    int newlight = digitalRead(CZUJNIKSWIATLA);

    // ODCZYT TEMPERATURY
    if (checkBound(newTemp, temp, diff)) {
      temp = newTemp;
      changeTEHU = 1;
      if (SerialPrint == 1) {Serial.print("Local Temperatura: ");
        Serial.println(newTemp,1);}
    }
    // ODCZYT WILGOTNOSCI
    if (checkBound(newHum, hum, diffhum)) {
      hum = newHum;
      changeTEHU = 1;
      if (SerialPrint == 1) {Serial.print("Local Wilgotność: ");
        Serial.println(newHum,1);}
      //client.publish(humidity_topic, String(newHum,1).c_str(), true);
    }
      if (changeTEHU == 1 ) {
        msgSendMQT = String(newTemp);
        msgSendMQT1 = String(newHum);
        /*msgSendMQT = "{\"temperature\": "+String(newTemp)+",\"TempUnit\":\"C\"}";
        msgSendMQT1 = "{\"humidity\": "+String(newHum)+"\"}";*/
        client.publish("homeassistant/sensor/arduino01_temperature/state", String(msgSendMQT).c_str(), true);
        client.publish("homeassistant/sensor/arduino01_humidity/state", String(msgSendMQT1).c_str(), true);
        //msgSendMQT = "{\"temperature1\": "+String(s1_1)+",\"TempUnit\":\"C\"}";
        //msgSendMQT1 = "{\"humidity1\": "+String(s1_2)+"\"}";
      }
    
    // STAN KONTAKTRONU PRZY ZMIANIE
    if (newKONT != KONT) {     
      KONT = newKONT;
      if (KONT == 0){
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: ZAMKNIETY");}
        client.publish(contactron_topic, "off", true);
      }
      if (KONT == 1){
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: OTWARTY");}
        client.publish(contactron_topic, "on", true);
      }
    }
    // STAN KONTAKTRONU PRZY URUCHOMIENIU
    if (KONT == 2) {     
      if (newKONT == 0){
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: ZAMKNIĘTY");}
        client.publish(contactron_topic, "off", true);
      }
      if (newKONT == 1){
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: OTWARTY");}
        client.publish(contactron_topic, "on", true);
      }  
        client.publish(availability_topic, "online", true);
    } 

    // STAN CZUJNIKA PRZY ZMIANIE
    if (newlight != light) {     
      light = newlight;
      if (light == 0){
        if (SerialPrint == 1) {Serial.println("Swiatlo: JASNO");}
        client.publish(light_topic, "on", true);
      }
      if (light == 1){
        if (SerialPrint == 1) {Serial.println("Swiatlo: CIEMNO");}
        client.publish(light_topic, "off", true);
      }
    }
    // ODCZYT NATEZENIA SWIATLA
    if (light == 2) {     
      if (newlight == 0){
        if (SerialPrint == 1) {Serial.println("Swiatlo: JASNO");}
        client.publish(light_topic, "on", true);
      }
      if (newlight == 1){
        if (SerialPrint == 1) {Serial.println("Swiatlo: CIEMNO");}
        client.publish(light_topic, "off", true);
      }  
    }  
  }// End local sensors

  // Send local sensors to HA every 10 sec
 long nowmin = millis();
 if (nowmin - lastMsg2 > 10000) {
    lastMsg2 = nowmin;
    newTemp = dht.readTemperature();
    newHum  = dht.readHumidity();
    msgSendMQT = String(newTemp);
    msgSendMQT1 = String(newHum);
    /*msgSendMQT = "{\"temperature\": "+String(newTemp)+",\"TempUnit\":\"C\"}";
    msgSendMQT1 = "{\"humidity\": "+String(newHum)+"\"}";*/
    client.publish("homeassistant/sensor/arduino01_temperature/state", String(msgSendMQT).c_str(), true);
    client.publish("homeassistant/sensor/arduino01_humidity/state", String(msgSendMQT1).c_str(), true);
    //msgSendMQT = "{\"temperature1\": "+String(s1_1)+",\"TempUnit\":\"C\"}";
    //msgSendMQT1 = "{\"humidity1\": "+String(s1_2)+"\"}";
 }  // End Send all sensors to HA every 10 sec


  if (Serial.available() > 0)
  {
    if (Serial.read() == '@')
    {
      if (SerialPrint == 1) {Serial.println("Rebooting. . .");}
      client.publish(availability_topic, "Nieaktywny", true);
      delay(100); // Give the computer time to receive the "Rebooting. . ." message, or it won't show up
      void (*reboot)(void) = 0; // Creating a function pointer to address 0 then calling it reboots the board.
      reboot();
    }
  }
 if (AutoRestart) {
    if (millis() >= RestartTime){
      if (SerialPrint == 1) {Serial.println("Rebooting. . .");}
      void (*reboot)(void) = 0;
      reboot();
    }
 }
  
  // WEB SERVER ------------------------------------------------------------------------------------------------------------------------------------------------------- //
  if (WebServer){
    EthernetClient client = serverweb.available();
    if (client) {
      if (SerialPrint == 1) {Serial.println("new client");}
      // an http request ends with a blank line
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          if (SerialPrint == 1) {Serial.write(c);}
            if (c == '\n' && currentLineIsBlank) {
            // send a standard http response header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html><meta charset=\"utf-8\"><title>MultiSensor by Iron</title>");
            // add a meta refresh tag, so the browser pulls again every 10 seconds:
            client.println("<meta http-equiv=\"refresh\" content=\"10\"><B>");
            client.print(SoftVersion);
            client.print(" ");
            client.print(SoftBuildDate);
            client.print("</B><BR>Moduł: ");
            client.print(ModuleType);
            client.print("<BR>Odczytano: ");
            client.print(MQTTread);
            client.print("<BR>Uruchomiony: ");
            client.print(millis());
            client.print(" milisekund");
            client.print("<BR><BR>Temperatura: ");
            client.print(String(temp).c_str());
            client.print(" &#176;C");
            client.print("<BR>Wilgotność: ");
            client.print(String(hum).c_str());
            client.print("%");
  
            if (KONT == 0) {client.print("<BR>Kontaktron: ZAMKNIĘTY");}
            if (KONT == 1) {client.print("<BR>Kontaktron: OTWARTY");}
            if (light == 0) {client.print("<BR>Światło: JASNO");}
            if (light == 1) {client.print("<BR>Światło: CIEMNO");}
  
            client.print("<BR>Przekaźnik1: ");
            client.print(przekaznik1stan);
            client.print("<BR>Przekaźnik2: ");
            client.print(przekaznik2stan);
            client.print("<BR><BR>Nano ID: ");
            client.print(leakinfo.sensor.sensorId);
            client.print("<BR>Nano Temp: ");
            client.print(leakinfo.sensor.temp);
            client.print("<BR>Nano Wilg: ");
            client.print(leakinfo.sensor.hum);
            client.print("<BR>Nano Ligh: ");
            client.print(leakinfo.sensor.light);
            client.print("<BR>Nano Fire: ");
            client.print(leakinfo.sensor.fire);
            client.println("<br />");       
            client.println("</html>");
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
          } 
          else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        }
      }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      client.stop();
      if (SerialPrint == 1) {Serial.println("client disonnected");}
    }
  }// End WebServer
}// End Main Loop

void autodiscoveryHA(int start) {
  // Autodiscovery HomeAssistant
  if (SerialPrint == 1) {Serial.println("-------------Sending autodiscovery to HA");Serial.println("Send @ to restart Arduino"); }

  boolean xx1 = client.publish("homeassistant/sensor/arduino01_temperature/config", "{\"device_class\":\"temperature\",\"name\":\"Temperatura Salon\"}", true);
  if (xx1 != true){Serial.println("ERROR Temperature1 config");}
  boolean xx2 = client.publish("homeassistant/sensor/arduino01_humidity/config", "{\"device_class\":\"humidity\",\"name\":\"Wilgotność Salon\"}", true);
  if (xx2 != true){Serial.println("ERROR Humidity1 config");}

  boolean xx3 = client.publish("homeassistant/sensor/arduino01_temperature1/config", "{\"device_class\":\"temperature\",\"name\":\"Temperatura pom1\"}", true);
  if (xx3 != true){Serial.println("ERROR Temperature2 config");}
  boolean xx4 = client.publish("homeassistant/sensor/arduino01_humidity1/config", "{\"device_class\":\"humidity\",\"name\":\"Wilgotność pom1\"}", true);
  if (xx4 != true){Serial.println("ERROR Humidity2 config");}
  boolean xx5 = client.publish("homeassistant/sensor/arduino01_pressure1/config", "{\"device_class\":\"pressure\",\"name\":\"Ciśnienie pom1\"}", true);
  if (xx5 != true){Serial.println("ERROR Pressure2 config");}
  boolean xx6 = client.publish("homeassistant/sensor/arduino01_altitude1/config", "{\"device_class\":\"pressure\",\"name\":\"Wysokość pom1\"}", true);
  if (xx6 != true){Serial.println("ERROR Altitude2 config");}
  boolean xx7 = client.publish("homeassistant/sensor/arduino01_light1/config", "{\"device_class\":\"illuminance\",\"name\":\"Światło pom1\"}", true);
  if (xx7 != true){Serial.println("ERROR Light2 config");}
  boolean xx8 = client.publish("homeassistant/sensor/arduino01_temperaturepom1/config", "{\"device_class\":\"temperature\",\"name\":\"Temperatura w pom1\"}", true);
  if (xx8 != true){Serial.println("ERROR Temperature w pom1 config");}
  boolean xx9 = client.publish("homeassistant/binary_sensor/arduino01_firepom1/config", "{\"device_class\":\"smoke\",\"name\":\"Pożar w pom1\"}", true);
  if (xx9 != true){Serial.println("ERROR Fire w pom1 config");}
  boolean xx10 = client.publish("homeassistant/binary_sensor/arduino01_lamppom1/config", "{\"device_class\":\"light\",\"name\":\"Światło w pom1\"}", true);
  if (xx10 != true){Serial.println("ERROR Światło w pom1 config");}

  if ((xx1+xx2+xx3+xx4+xx5+xx6+xx7+xx8+xx9+xx10) == 10) { Serial.println("Autodiscovery published!");  }
    
  /*
    if (client.publish("homeassistant/sensor/arduino01_temperature/config", "{\"name\": \"Temperatura1 Ard1\",\"state_topic\": \"~SENSOR\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_temperature\",\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"°C\",\"device_class\": \"temperature\"}", true))
   {if (SerialPrint == 1) {Serial.println("OK Pub temp1");}else {if (SerialPrint == 1) {Serial.println("ERROR Pub temp1");}}}
    if (client.publish("homeassistant/sensor/arduino01_humidity/config", "{\"name\": \"Wilgotność1 Ard1\",\"state_topic\": \"~SENSOR1\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_humidity\",\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"%\",\"device_class\": \"humidity\"}", true))
   {if (SerialPrint == 1) {Serial.println("OK Pub hum1");}else {if (SerialPrint == 1) {Serial.println("ERROR Pub hum1");}}}
   if (client.publish("homeassistant/sensor/arduino01_temperature1/config", "{\"name\": \"Temperatura2 Ard1\",\"state_topic\": \"~SENSOR2\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_temperature1\",\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"°C\",\"device_class\": \"temperature\"}", true))
   {if (SerialPrint == 1) {Serial.println("OK Pub temp2");}else {if (SerialPrint == 1) {Serial.println("ERROR Pub temp2");}}}
    if (client.publish("homeassistant/sensor/arduino01_humidity1/config", "{\"name\": \"Wilgotność2 Ard1\",\"state_topic\": \"~SENSOR3\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_humidity1\",\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"%\",\"device_class\": \"humidity\"}", true))
    client.publish(autodiscovery_status_topic, "{\"name\": \"Arduino01 status\",\"stat_t\": \"~HASS_STATE\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"json_attributes_topic\": \"~HASS_STATE\",\"unit_of_meas\": \" \",\"val_tpl\": \"{{value_json['MqttReadCount']}}\",\"ic\": \"mdi:information-outline\",\"uniq_id\": \"arduino01_status\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]],\"name\": \"arduino01\",\"model\": \"arduino01\",\"sw_version\": \"0.1.1\",\"manufacturer\": \"Iron\"},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\"}", true);
    client.publish(autodiscovery_sensor1_topic, "{\"name\": \"Temperature Arduino01\",\"state_topic\": \"~SENSOR\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_temperature\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{value_json.temperature}}\",\"device_class\": \"temperature\"}", true);
    client.publish(autodiscovery_sensor2_topic, "{\"name\": \"Humidity Arduino01\",\"state_topic\": \"~SENSOR1\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_humidity\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"%\",\"value_template\": \"{{value_json.humidity}}\",\"device_class\": \"humidity\"}", true);
    client.publish(autodiscovery_sensor3_topic, "{\"name\": \"Temperature Arduino02\",\"state_topic\": \"~SENSOR2\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_temperature1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"°C\",\"value_template\": \"{{value_json.temperature1}}\",\"device_class\": \"temperature\"}", true);
    client.publish(autodiscovery_sensor4_topic, "{\"name\": \"Humidity Arduino02\",\"state_topic\": \"~SENSOR3\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_humidity1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_measurement\": \"%\",\"value_template\": \"{{value_json.humidity1}}\",\"device_class\": \"humidity\"}", true);
    client.publish(autodiscovery_sensor5_topic, "{\"name\": \"Light Arduino02\",\"stat_t\": \"~SENSORlight02\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"Arduino01_Light\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_meas\": \"%\",\"value_template\": \"\",\"dev_cla\": \"light\"}", true);
    client.publish(autodiscovery_switch1_topic, "{\"name\": \"Arduino01 Switch 01\",\"cmd_t\": \"~cmnd/POWER1\",\"stat_t\": \"~tele/POWER1\",\"pl_off\": \"OFF\",\"pl_on\": \"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_RL_1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/\"}", true);
    client.publish(autodiscovery_switch2_topic, "{\"name\": \"Arduino01 Switch 02\",\"cmd_t\": \"~cmnd/POWER2\",\"stat_t\": \"~tele/POWER2\",\"pl_off\": \"OFF\",\"pl_on\": \"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_RL_2\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/\"}", true);
    client.publish(autodiscovery_binary1_topic, "{\"name\": \"Arduino01 Drzwi 01\",\"stat_t\": \"~SENSORdoor01\",\"avty_t\": \"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_BTN_1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/\",\"device_class\": \"door\",\"pl_on\": \"on\",\"pl_off\": \"off\"}", true);
    client.publish(autodiscovery_binary2_topic, "{\"name\": \"Arduino01 Światło 01\",\"stat_t\": \"~SENSORlight01\",\"avty_t\": \"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_BTN_2\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"dom-88fd54db6487cd19_arduino01/\",\"device_class\": \"light\",\"pl_on\": \"on\",\"pl_off\": \"off\"}", true);
*/   
    changeTEHU = 1;
  if (start == 0) {
    if (przekaznik1stan == "ZAŁĄCZONY") {
      client.publish(statepower1_topic, "ON", true);  
    } else {
      client.publish(statepower1_topic, "OFF", true);  }
    if (przekaznik2stan == "ZAŁĄCZONY") {
      client.publish(statepower2_topic, "ON", true);  
    } else {
      client.publish(statepower2_topic, "OFF", true);  }
  }
} //end Autodiscovery

// Send invitation to get Data from sensors 
int getData(int sensorID)
{
    byte msg [] = { 
     sensorID,    // device 0 - Centrala, 1 - Arduino nano 01
     1,    // 1 = received OK
    };
    delay (1);  // give the master a moment to prepare to receive
    digitalWrite (ENABLE_PIN, HIGH);    // enable sending
    sendMsg (fWrite, msg, sizeof msg);  // send confirmation  
    digitalWrite (ENABLE_PIN, LOW);     // disable sending   
} // end getData

int receiveData()
{
  byte byteArray[PACKET_SIZE];
  byte received = recvMsg (fAvailable, fRead, byteArray, sizeof(sensorData_t)); 
  if (received){
    if (SerialPrint == 1) {Serial.println("RS485 - received");}
    for (int k=0; k < PACKET_SIZE; k++)
    { 
      leakinfo.rs485Packet[k] = byteArray[k];
    }
    return received;
  }  // end got packet
  else
  { return 0; } // No Packet received
  
} // end getData

void SendSensorStatus(){
  //Wyslij status
  String statemsg = "";
  statemsg += "{\"Version\":\"";
  statemsg += SoftVersion;
  statemsg += "\",\"BuildDateTime\":\"";
  statemsg += SoftBuildDate;
  statemsg += "\",\"MqttReadCount\":";
  statemsg += MQTTread;
  statemsg += ",\"UptimemSec\":";
  statemsg += millis();
  statemsg += ",\"Module\":\"";
  statemsg += ModuleType;
  //statemsg += "\",\"IPAddress\":\"";
  //statemsg += String(AddIP).c_str();
  //statemsg += "\",\"MACAddress\":\"";
  //statemsg += String(AddMAC).c_str();
  statemsg += "\"}";
   client.publish(state_topic, String(statemsg).c_str(), true);
  if (SerialPrint == 1) {Serial.println(String(statemsg).c_str());}
  client.publish(availability_topic, "Aktywny", true);
}
