/* 
 ====== MultiSensorsJolka by Iron ======
 ======      Testing platform     ======
 Arduino Mega 2560 + Ethernet Shield
 ====== Changelog ======
 v0.1.2
 - Dodanie kolejnych sensorów - Zewnętrzny
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
//#include <SPI.h>
#include <TimeLib.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "RS485_protocol.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Udp.h>

#define SoftVersion "MultiSensorsJolka v0.1.0 by Iron"
#define SoftBuildDate "2020-02-29T20:04:00"
#define ModuleType "Arduino Mega 2560+Ethernet Shield"
//#define sensors_topic "arduino01/tele/SENSOR"
//#define sensors_topic "homeassistant/sensor/temp_room_1/state"
//#define sensors1_topic "arduino01/tele/SENSOR1"
//#define sensors2_topic "arduino01/tele/SENSOR2"
//#define sensors3_topic "arduino01/tele/SENSOR3"
#define contactron_topic "arduino01/tele/SENSORdoor01"
#define light_topic "arduino01/tele/SENSORlight01"
#define availability_topic "arduino01/tele/LWT" // Aktywny / Nieaktywny
#define state_topic "arduino01/tele/HASS_STATE"
#define statestatus_topic "arduino01/stat/STATUS"
#define statepower1_topic "arduino01/tele/POWER1" // ON/OFF
#define statepower2_topic "arduino01/tele/POWER2" // ON/OFF

#define sensor1statepower1_topic "arduino01/tele/lamppom1" // ON/OFF
#define sensor2statepower1_topic "wemospiec/tele/POWER1" // ON/OFF
#define command1_topic "arduino01/cmnd/POWER1"
#define command2_topic "arduino01/cmnd/POWER2"

#define sensor1command1_topic "arduino01/cmnd/lamppom1"
#define sensor2command1_topic "wemospiec/cmnd/POWER1"
#define autodiscovery_send_topic "dom/cmnd/SetOption19"

//#define autodiscovery_light_topic "homeassistant/light/arduino01/light/config"
//#define autodiscovery_switch1_topic "homeassistant/switch/arduino01_RL_1/config"
//#define autodiscovery_switch2_topic "homeassistant/switch/arduino01_RL_2/config"
//#define autodiscovery_binary1_topic "homeassistant/binary_sensor/arduino01_BTN_1/config"
//#define autodiscovery_binary2_topic "homeassistant/binary_sensor/arduino01_BTN_2/config"
//#define autodiscovery_sensor1_topic "homeassistant/sensor/arduino01_temperature/config"
//#define autodiscovery_sensor2_topic "homeassistant/sensor/arduino01_humidity/config"
//#define autodiscovery_sensor3_topic "homeassistant/sensor/arduino01_temperature1/config"
//#define autodiscovery_sensor4_topic "homeassistant/sensor/arduino01_humidity1/config"
//#define autodiscovery_sensor5_topic "homeassistant/sensor/arduino01_Light02/config"
#define autodiscovery_status_topic "homeassistant/sensor/arduino01_status/config"
#define sensorpom1 "Salon" // 1
#define sensorpom2 "Pokój Ani" // 2
#define DHTPIN 22 // 8->22 Pin Arduino do ktorego podlaczony jest czujnik
const byte ENABLE_PIN = 25; //D48

#define DHTTYPE DHT22 //21 or 22 w zaleznosci od czujnika lub AM2302 = DTH22
#define KONTAKTRON 3 //3 pin Arduino do kontaktronu
#define PRZEKAZNIK1 6 //6 pin Arduino do przekaźnika 1
#define PRZEKAZNIK2 7 //7 pin Arduino do przekaźnika 2
#define PRZEKAZNIK3 8 //8 pin Arduino do przekaźnika 3
#define PRZEKAZNIK4 9 //9 pin Arduino do przekaźnika 4
//USE PIN Mega Ethernet Shield 0,1, 4,10, 50,51,52,53
//USE PIN UNO  Ethernet Shield 0,1, 4,10, 11,12,13
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 26; 
int LightSensor = A8; //define analog pin8

LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);
int SerialPrint = 1; // 0 - nie wysyłaj nic do portu szeregowego, 1 - debug log do portu szeregowego
bool WebServer = false;   // false - nie uruchamiaj WebServera, true - uruchom webserver
bool AutoRestart = false; // false - bez restaru, true1 - autorestart
int RestartTime = 1800000; // 1800000 = 30 minut
volatile byte relayState = LOW;
byte relay = 0; // relay,lamp

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

/* ******** NTP Server Settings ******** */
/* us.pool.ntp.org NTP server
   (Set to your time server of choice) */
IPAddress timeServer(192, 168, 2, 2);

/* Set this to the offset (in seconds) to your local time
   This example is GMT - 4 */
//const long timeZoneOffset = -14400L; 
const long timeZoneOffset = 7200L; 

/* Syncs to NTP server every 15 seconds for testing,
   set to 1 hour or more to be reasonable */
unsigned int ntpSyncTime = 3600;       


/* ALTER THESE VARIABLES AT YOUR OWN RISK */
// local port to listen for UDP packets
unsigned int localPort = 8888;
// NTP time stamp is in the first 48 bytes of the message
const int NTP_PACKET_SIZE= 48;     
// Buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE]; 
// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;                   
// Keeps track of how long ago we updated the NTP server
unsigned long ntpLastUpdate = 0;   
// Check last time clock displayed (Not in Production)
time_t prevDisplay = 0;          

/*Register Map:
byte  Desc 
 0    status (slave status)
 1    sensor Id   (unsigned char 0-255)
 2    temperature (float)
 3    temperature (float)
 4    temperature (float)
 5    temperature (float)
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
 22   1-light,2-light (unsigned char 0-255)
 23   1-fire, 2-sun   (unsigned char 0-255)
 24   1-lamp, 2-rain  (unsigned char 0-255)
 25   1---- , 2----   (unsigned char 0-255)
 */

//sensor1 22-light%, 23-fire%, 24-lamp 0/1
//sensor2 22-light%, 23-sun%, 24-rain%

typedef struct sensorData_t {
  byte stat;
  unsigned char sensorId;
  float temp;
  float hum;
  float pressure;
  float alt;
  float settemp;
  unsigned char sensor1;
  unsigned char sensor2;
  unsigned char sensor3;
  unsigned char sensor4;
};
#define PACKET_SIZE sizeof(sensorData_t)
bool HAturnon = true;

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
    
    // Zalaczenie przekaznika w sensor1 (lampka)
    if (strcmp (topic, sensor1command1_topic) == 0) {
      HAturnon = true;
      MQTTread += 1;
      if (SerialPrint == 1) {Serial.print("Sensor1 - Lampa = ");}
      if (payloadmsg == "ON") {
        if (SerialPrint == 1) {Serial.println("ZAŁĄCZ");}
        client.publish(sensor1statepower1_topic, "ON", true);
        //leakinfo.sensor.lamp=1;
        relay = 1;
        
      }
      if (payloadmsg == "OFF") {
        if (SerialPrint == 1) {Serial.println("WYŁĄCZ");}
        client.publish(sensor1statepower1_topic, "OFF", true);
        //leakinfo.sensor.lamp=0;
        relay = 0;
      }
    }
    
    // Zalaczenie przekaznika w sensor 2   
    if (strcmp (topic, sensor2command1_topic) == 0) {
      MQTTread += 1;
      if (SerialPrint == 1) {Serial.print("Sensor2 - Przekaźnik 1 = ");}
      if (payloadmsg == "ON") {
        if (SerialPrint == 1) {Serial.println("ZAŁĄCZ");}
        client.publish(sensor2statepower1_topic, "ON", true);
        relay = 1;
      }
      if (payloadmsg == "OFF") {
        if (SerialPrint == 1) {Serial.println("WYŁĄCZ");}
        client.publish(sensor2statepower1_topic, "OFF", true);
        relay = 0;    
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
    if (client.connect("arduino01","DVES_USER","","arduino01/tele/LWT",1,true,"Nieaktywny")) {
      if (SerialPrint == 1) {Serial.println("connected");}
      autodiscoveryHA(1); // Przedstawienie się autodiscovery przy starcie
      SendSensorStatus(); // Send status to HA
      boolean rc1 = client.subscribe(command1_topic); // Oczykiwanie na topic dla przekaźnika 1
      boolean rc2 = client.subscribe(command2_topic); // Oczykiwanie na topic dla przekaźnika 2
      boolean rc3 = client.subscribe(sensor1command1_topic); // Oczykiwanie na topic dla przekaźnika 1
      boolean rc4 = client.subscribe(sensor2command1_topic); // Oczykiwanie na topic dla przekaźnika 2
      boolean rcst1 = client.subscribe(statepower1_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 1
      boolean rcst2 = client.subscribe(statepower2_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 2
      boolean rcst3 = client.subscribe(autodiscovery_send_topic); // SetOption19 

      if (rc1+rc2+rc3+rc4+rcst1+rcst2+rcst3 == 7){
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
  if (SerialPrint == 1) {Serial.println("Arduino MEGA 2560 MQTT - HA");}
  client.setServer("192.168.2.1", 1883); // Adres serwera MQTT - bramka
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  if (WebServer){  serverweb.begin();}

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("MultiSensor IRON");
  lcd.setCursor(0,1);
  lcd.print("initializing...");
  dht.begin();
  if (isnan(dht.readTemperature()) || isnan(dht.readHumidity())) {
      Serial.println("!!! Failed to read from DHT sensor!");
  }  
  pinMode(KONTAKTRON, INPUT_PULLUP); //Kontaktron jako wejście
  //pinMode(CZUJNIKSWIATLA, INPUT); // Czujnik światła
  //pinMode(PRZEKAZNIK1, OUTPUT); // Ustawienie Pinu przekaznika 1 jako Wyjscie
  //pinMode(PRZEKAZNIK2, OUTPUT); // Ustawienie Pinu przekaznika 2 jako Wyjscie
  //digitalWrite(PRZEKAZNIK1, HIGH); // Wyłacz przekaźnik 1 przy starcie , LOW = włącz
  //digitalWrite(PRZEKAZNIK2, HIGH); // Wyłacz przekaźnik 2 przy starcie , LOW = włącz       
  rs485.begin (9600);
  pinMode(ENABLE_PIN, OUTPUT);
  delay(10);

  if (SerialPrint == 1) {Serial.println(Ethernet.localIP());} //Wyswietlenie adresu IP Arduino
  AddIP = String(Ethernet.localIP()).c_str();
  byte macBuffer[6];  // create a buffer to hold the MAC address
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

   //Try to get the date and time
   int trys=0;
   while(!getTimeAndDate() && trys<10) {
     trys++;
   }
  
} // End setup loop

// Do not alter this function, it is used by the system
int getTimeAndDate() {
   int flag=0;
   Udp.begin(localPort);
   sendNTPpacket(timeServer);
   delay(1000);
   if (Udp.parsePacket()){
     Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
     unsigned long highWord, lowWord, epoch;
     highWord = word(packetBuffer[40], packetBuffer[41]);
     lowWord = word(packetBuffer[42], packetBuffer[43]); 
     epoch = highWord << 16 | lowWord;
     epoch = epoch - 2208988800 + timeZoneOffset;
     flag=1;
     setTime(epoch);
     ntpLastUpdate = now();
     Serial.print("NTP Last update: ");
     Serial.println(ntpLastUpdate);
   }
   return flag;
}

// Do not alter this function, it is used by the system
unsigned long sendNTPpacket(IPAddress& address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;                 
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket();
}

// Clock display of the time and date (Basic)
void clockDisplay(){
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(year());
  Serial.println();
}

// Utility function for clock display: prints preceding colon and leading 0
void printDigits(int digits){
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void printDigitslcd(int digits){
  lcd.print(":");
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}

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
  int light = 0; // stan 2 przy pierwszym odczycie
  int fire = 0; // fire alarm
  int pwmval = 0;
  int numsensors = 1; // # of remote sensors
  long rcverr1 = 0; // error received from sensor 1
  long rcverr2 = 0; // error received from sensor 2

void loop(){
  if (!client.connected()) {reconnect();}
  client.loop(); 
  long czekaj;
  float newTemp = 0.0;
  float newHum  = 0.0;
  String msgSendMQTfire;
  String msgSendMQTfire1;
  String msgSendMQTlamp;

    // Update the time via NTP server as often as the time you set at the top
    if(now()-ntpLastUpdate > ntpSyncTime) {
      int trys=0;
      while(!getTimeAndDate() && trys<10){
        trys++;
      }
      if(trys<10){
        Serial.println("ntp server update success");
      }
      else{
        Serial.println("ntp server update failed");
      }
    }

    // Display the time if it has changed by more than a second.
    if( now() != prevDisplay){
      prevDisplay = now();
      clockDisplay(); 
    }
    
if( now() != prevDisplay) //update the display only if the time has changed
  {
    prevDisplay = now();
    digitalClockDisplay();    
  }
  
    // STAN KONTAKTRONU PRZY ZMIANIE
    if (newKONT != KONT) {           
      KONT = newKONT;
      if (KONT == 0){client.publish(contactron_topic, "off", true);
      }else {client.publish(contactron_topic, "on", true);}
    }
    // STAN CZUJNIKA PRZY ZMIANIE
    if (newlight != light) {     
      light = newlight;
      if (light == 0){client.publish(light_topic, "on", true);
      }else {client.publish(light_topic, "off", true);}
    }
    
  // Send status to HA every 1 min
  long now1 = millis();
  if (now1 - lastMsg1 > 60000) {
    lastMsg1 = now1;
    SendSensorStatus(); // Send status to HA
    //if (SerialPrint == 1) {Serial.print("Free RAM: "); Serial.println(freeRam());}
    //if (SerialPrint == 1) {Serial.print("Minutes : "); Serial.println(millis()*0.000016666666666667);}
 } // Send Status do HA every 1 min

 
  // Read local sensors every 1 seconds
    long now3 = millis();
  if (now3 - lastMsg3 > 1000) {
    lastMsg3 = now3;

    byte rs485Status = 0;
    rs485Status = receiveData(); // Receive Data from sensors
    Serial.print("\nSizeOf: "); Serial.println(sizeof(sensorData_t));    
    Serial.print("\nrs485Status: "); Serial.println(rs485Status);    
    //rs485Status == sizeof(sensorData_t) &&

  
  
   
    /*if (rs485Status == sizeof(sensorData_t) && leakinfo1.sensor.sensorId == 1){Serial.println("Received from sensor 1");}
      else{
        rcverr1++;
        Serial.print(rcverr1);
        if (rcverr1 >= 30) {
          Serial.println(F("No packet received from sensor 1"));
          client.publish("arduino01/tele/temperaturepom1","niedostępny",true);
          client.publish("arduino01/tele/humiditypom1","niedostępny",true);
          client.publish("arduino01/tele/pressurepom1","niedostępny",true);
          client.publish("arduino01/tele/altitudepom1","niedostępny",true);
          client.publish("arduino01/tele/lightpom1","niedostępny",true);
          client.publish("arduino01/tele/temperaturesetpom1","niedostępny",true);  
          client.publish("arduino01/tele/firepom1/state","niedostępny",true);
          client.publish("arduino01/tele/lamppom1","niedostępny",true);
          delay(100);
          rcverr1 = 0;
        }
      }
    if (HAturnon == true){leakinfo1.sensor.lamp=relay;HAturnon = false;}
*/
 /*   if (rs485Status == sizeof(sensorData_t) && leakinfo1.sensor.sensorId == 2){Serial.println("Received from sensor 2");}
      else{
        rcverr2++;
        Serial.print(rcverr2);
        if (rcverr2 >= 30) {
          Serial.println("");
          Serial.println(F("No packet received from sensor 2"));
          //client.publish("wemospiec/tele/temperature","niedostępny",true);
          //client.publish("wemospiec/tele/pressure","niedostępny",true);
          delay(100);
          rcverr2 = 0;
        }
      }
    if (HAturnon == true){leakinfo2.sensor.lamp=relay;HAturnon = false;}*/
    
  }// end 1s receive data
  
     long now2 = millis();
  if (now2 - lastMsg2 > 10000) {
    lastMsg2 = now2;  
    //getData(1); // Send Request to get sensor 1 Data
    //delay(100);
    getData(2); // Send Request to get sensor 2 Data
    delay(100);
    Serial.println("");Serial.print("_________________________________"); Serial.println("Send invitation to sensor 1,2...");
   // GET data from remote sensors
    rcverr1 = 0;
   if (leakinfo.sensor.sensorId == 1){Serial.print("\nReceived data from sensor 1");
    Serial.println("---SENSOR 1 ---");
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
    Serial.print("Temp. w pom: ");
    Serial.println(leakinfo.sensor.settemp);    
    Serial.print("Światło: ");
    Serial.println(leakinfo.sensor.sensor1);
    Serial.print("Fire alert: ");
    Serial.println(leakinfo.sensor.sensor2);
    Serial.print("Lamp: ");
    Serial.println(leakinfo.sensor.sensor3);   
   }
  if (leakinfo.sensor.sensorId == 2){Serial.println("\nReceived data from sensor 2");
    Serial.println("---SENSOR 2 ZEWNĘTRZNY ---");
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
    Serial.println(leakinfo.sensor.sensor1);
    Serial.print("Słońce: ");
    Serial.println(leakinfo.sensor.sensor2);
    Serial.print("Deszcz: ");
    Serial.println(leakinfo.sensor.sensor3);
    client.publish("arduino01/tele/temperaturezewn", String(leakinfo.sensor.temp).c_str(), true);
    client.publish("arduino01/tele/humidityzewn", String(leakinfo.sensor.hum).c_str(), true);
    client.publish("arduino01/tele/pressurezewn", String(leakinfo.sensor.pressure).c_str(), true);
    client.publish("arduino01/tele/lightzewn", String(leakinfo.sensor.sensor1).c_str(), true);
    client.publish("arduino01/tele/sunzewn", String(leakinfo.sensor.sensor2).c_str(), true);
    client.publish("arduino01/tele/rainzewn", String(leakinfo.sensor.sensor3).c_str(), true);
   }
    /*if (leakinfo.sensor.fire==1){msgSendMQTfire = String("ON");}
      else {msgSendMQTfire = String("OFF");}
    if (leakinfo.sensor.lamp==1){msgSendMQTlamp = String("ON");relay=1;}
      else {msgSendMQTlamp = String("OFF");relay=0;}
    client.publish("arduino01/tele/temperaturepom1", String(leakinfo.sensor.temp).c_str(), true);
    client.publish("arduino01/tele/humiditypom1", String(leakinfo.sensor.hum).c_str(), true);
    client.publish("arduino01/tele/pressurepom1", String(leakinfo.sensor.pressure).c_str(), true);
    client.publish("arduino01/tele/altitudepom1", String(leakinfo.sensor.alt).c_str(), true);
    client.publish("arduino01/tele/lightpom1", String(leakinfo.sensor.light).c_str(), true);
    client.publish("arduino01/tele/temperaturesetpom1", String(leakinfo.sensor.settemp).c_str(), true);
    client.publish("arduino01/tele/firepom1", String(msgSendMQTfire).c_str(), true);
    client.publish("arduino01/cmnd/lamppom1", String(msgSendMQTlamp).c_str(), true);
    client.publish("arduino01/tele/lamppom1", String(msgSendMQTlamp).c_str(), true);
    */
    //client.publish("wemospiec/tele/temperature", String(leakinfo.sensor.temp).c_str(), true);
    //client.publish("wemospiec/tele/pressure", String(leakinfo.sensor.pressure).c_str(), true);
    //client.publish("wemospiec/tele/POWER1", String(msgSendMQTfire1).c_str(), true);   

      // READ local sensors 
    newTemp = dht.readTemperature();
    newHum  = dht.readHumidity();
    int newlight = analogRead(LightSensor);
    light = map(newlight, 1016, 5, 0, 100);
    
    int newKONT = digitalRead(KONTAKTRON);
    
    Serial.println("");
    Serial.println("____________________");
    Serial.println("---LOCAL SENSORS ---");
    Serial.print("Temperatura: ");
    Serial.println(newTemp,1);
    Serial.print("Wilgotność: ");
    Serial.println(newHum,1);
    Serial.print("Światło: ");
    Serial.println(light);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(newTemp,1);
    lcd.print(" W:");
    lcd.print(newHum,0);
    lcd.print(" L:");
    lcd.print(light);
    lcd.print(" ");
    lcd.setCursor(0,1);    
    lcd.print(hour());
    printDigitslcd(minute());
    printDigitslcd(second());
    lcd.print(" ");
    lcd.print(day());
    lcd.print("-");
    lcd.print(month());
    lcd.print("-");
    lcd.print(year());
    
    client.publish("arduino01/tele/temperature", String(newTemp).c_str(), true);
    client.publish("arduino01/tele/humidity", String(newHum).c_str(), true);
    client.publish("arduino01/tele/light", String(light).c_str(), true);
   
    if (KONT == 0){
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: ZAMKNIETY");}}
        else {if (SerialPrint == 1) {Serial.println("Stan kontaktronu: OTWARTY");}
    }
  }
    
   
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
            client.print("<BR><BR>____________________");
            client.print("<BR><BR>Sensor ID: ");
            client.print(leakinfo.sensor.sensorId);
            client.print("<BR>Temperatura: ");
            client.print(leakinfo.sensor.temp);
            client.print("<BR>Wilgotność: ");
            client.print(leakinfo.sensor.hum);
            client.print("<BR>Ciśnienie: ");
            client.print(leakinfo.sensor.pressure);
            client.print("<BR>Wysokość: ");
            client.print(leakinfo.sensor.alt);
            client.print("<BR>Ogień: ");
            client.print(leakinfo.sensor.sensor1);
            client.print("<BR>Ogień: ");
            client.print(leakinfo.sensor.sensor2);
            client.print("<BR>Ogień: ");
            client.print(leakinfo.sensor.sensor3);
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



// Send invitation to get Data from sensors 
int getData(int sensorID)
{
    byte msg [] = { 
     sensorID,    // device 0 - Centrala, 1 - Arduino nano 01, 2 - Arduino Nano Zewn
     1,    // 1 = received OK
     relay, // relay/lamp 0 = off, 1 = on
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
  int sensorID = 0;
  if (received){
    if (SerialPrint == 1) {Serial.println("");Serial.print("* RS485 - received from ");}
      for (int k=0; k < PACKET_SIZE; k++)
      { 
        if (k==1) {sensorID = byteArray[k];Serial.print("Sensor #: ");Serial.print(sensorID);}         
          leakinfo.rs485Packet[k] = byteArray[k];
          Serial.print(",");Serial.print(k);Serial.print(":");Serial.print(byteArray[k]);
        }
        
    return received;  
  }  // end got packet
  else
  { return 0; } // No Packet received
} // end receiveData

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


void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.println();
}

void autodiscoveryHA(int start) {
  // Autodiscovery HomeAssistant
  if (SerialPrint == 1) {Serial.println("Send @ to restart Arduino"); Serial.println("Sending autodiscovery to HA"); }
  // LOCAL SENSORS
  boolean xx1 = client.publish("homeassistant/sensor/arduino01_temperature/config", "{\"dev_cla\":\"temperature\",\"name\":\"Temperatura Centrala\",\"stat_t\":\"~tele/temperature\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"°C\"}", true);
  if (xx1 != true){Serial.println("ERROR Temperature Centrala config");}
  boolean xx2 = client.publish("homeassistant/sensor/arduino01_humidity/config", "{\"dev_cla\":\"humidity\",\"name\":\"Wilgotność Centrala\",\"stat_t\":\"~tele/humidity\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx2 != true){Serial.println("ERROR Humidity Centrala config");}
  boolean xx3 = client.publish("homeassistant/sensor/arduino01_light/config", "{\"dev_cla\":\"illuminance\",\"name\":\"Światło Centrala\",\"stat_t\":\"~tele/light\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx3 != true){Serial.println("ERROR Light Centrala config");}
  // REMOTE SENSOR 2 ZEWNETRZNY
  boolean xx4 = client.publish("homeassistant/sensor/arduino01_temperaturezewn/config", "{\"dev_cla\":\"temperature\",\"name\":\"Temperatura na zewnątrz\",\"stat_t\":\"~tele/temperaturezewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"°C\"}", true);
  if (xx4 != true){Serial.println("ERROR Temperature zewn config");}
  boolean xx5 = client.publish("homeassistant/sensor/arduino01_humidityzewn/config", "{\"dev_cla\":\"humidity\",\"name\":\"Wilgotność na zewnątrz\",\"stat_t\":\"~tele/humidityzewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx5 != true){Serial.println("ERROR Humidity zewn config");}
  boolean xx6 = client.publish("homeassistant/sensor/arduino01_pressurezewn/config", "{\"dev_cla\":\"pressure\",\"name\":\"Ciśnienie na zewnątrz\",\"stat_t\":\"~tele/pressurezewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"hPa\"}", true);
  if (xx6 != true){Serial.println("ERROR Pressure zewn config");}
  boolean xx7 = client.publish("homeassistant/sensor/arduino01_lightzewn/config", "{\"dev_cla\":\"illuminance\",\"name\":\"Światło na zewnątrz\",\"stat_t\":\"~tele/lightzewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx7 != true){Serial.println("ERROR Light zewn config");}
  boolean xx8 = client.publish("homeassistant/sensor/arduino01_sunzewn/config", "{\"dev_cla\":\"illuminance\",\"name\":\"Słońce\",\"stat_t\":\"~tele/sunzewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx8 != true){Serial.println("ERROR Sun zewn config");}
  boolean xx9 = client.publish("homeassistant/sensor/arduino01_rainzewn/config", "{\"dev_cla\":\"humidity\",\"name\":\"Deszcz\",\"stat_t\":\"~tele/rainzewn\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx9 != true){Serial.println("ERROR Rain zewn config");}

  if ((xx1+xx2+xx3+xx4+xx5+xx6+xx7+xx8+xx9) == 9) { Serial.println("Autodiscovery published correctly!");  } else {Serial.println("Autodiscovery published ERROR!");}


  // REMOTE SENSORS POM 1
  /*boolean xx4 = client.publish("homeassistant/sensor/arduino01_temperaturepom1/config", "{\"dev_cla\":\"temperature\",\"name\":\"Temperatura pom1\",\"stat_t\":\"~tele/temperaturepom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"°C\"}", true);
  if (xx4 != true){Serial.println("ERROR Temperature pom1 config");}
  boolean xx5 = client.publish("homeassistant/sensor/arduino01_humiditypom1/config", "{\"dev_cla\":\"humidity\",\"name\":\"Wilgotność pom1\",\"stat_t\":\"~tele/humiditypom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx5 != true){Serial.println("ERROR Humidity pom1 config");}
  boolean xx6 = client.publish("homeassistant/sensor/arduino01_pressurepom1/config", "{\"dev_cla\":\"pressure\",\"name\":\"Ciśnienie pom1\",\"stat_t\":\"~tele/pressurepom1\",\"avty_t\": \"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"hPa\"}", true);
  if (xx6 != true){Serial.println("ERROR Pressure pom1 config");}
  boolean xx7 = client.publish("homeassistant/sensor/arduino01_altitudepom1/config", "{\"name\":\"Wysokość pom1\",\"stat_t\":\"~tele/altitudepom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"m\"}", true);
  if (xx7 != true){Serial.println("ERROR Altitude pom1 config");}
  boolean xx8 = client.publish("homeassistant/sensor/arduino01_lightpom1/config", "{\"dev_cla\":\"illuminance\",\"name\":\"Światło pom1\",\"stat_t\":\"~tele/lightpom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"%\"}", true);
  if (xx8 != true){Serial.println("ERROR Light pom1 config");}
  boolean xx9 = client.publish("homeassistant/sensor/arduino01_temperaturesetpom1/config", "{\"dev_cla\":\"temperature\",\"name\":\"Temperatura set pom1\",\"stat_t\":\"~tele/temperaturesetpom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"~\":\"arduino01/\",\"unit_of_meas\":\"°C\"}", true);
  if (xx9 != true){Serial.println("ERROR Temperature set pom1 config");}
  boolean xx10 = client.publish("homeassistant/binary_sensor/arduino01_firepom1/config", "{\"dev_cla\":\"smoke\",\"name\":\"Pożar w pom1\",\"stat_t\":\"~tele/firepom1\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"~\":\"arduino01/\"}", true);
  if (xx10 != true){Serial.println("ERROR Fire pom1 config");}
  boolean xx11 = client.publish("homeassistant/switch/arduino01_lamppom1/config", "{\"name\":\"Lampa pom1\",\"cmd_t\":\"~cmnd/lamppom1\",\"stat_t\":\"~tele/lamppom1\",\"avty_t\":\"~tele/LWT\",\"~\":\"arduino01/\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\"}", true);
  if (xx11 != true){Serial.println("ERROR Lamp pom1 config");}
  if ((xx1+xx2+xx3+xx4+xx5+xx6+xx7+xx8+xx9+xx10+xx11) == 11) { Serial.println("Autodiscovery published correctly!");  } else {Serial.println("Autodiscovery published ERROR!");}
*/

  /*
     client.publish(autodiscovery_status_topic, "{\"name\": \"Arduino01 status\",\"stat_t\": \"~HASS_STATE\",\"avty_t\": \"~LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"json_attributes_topic\": \"~HASS_STATE\",\"unit_of_meas\": \" \",\"val_tpl\": \"{{value_json['MqttReadCount']}}\",\"ic\": \"mdi:information-outline\",\"uniq_id\": \"arduino01_status\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]],\"name\": \"arduino01\",\"model\": \"arduino01\",\"sw_version\": \"0.1.1\",\"manufacturer\": \"Iron\"},\"~\": \"arduino01/tele/\"}", true);
    client.publish(autodiscovery_switch1_topic, "{\"name\": \"Arduino01 Switch 01\",\"cmd_t\": \"~cmnd/POWER1\",\"stat_t\": \"~tele/POWER1\",\"pl_off\": \"OFF\",\"pl_on\": \"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_RL_1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"arduino01/\"}", true);
    client.publish(autodiscovery_switch2_topic, "{\"name\": \"Arduino01 Switch 02\",\"cmd_t\": \"~cmnd/POWER2\",\"stat_t\": \"~tele/POWER2\",\"pl_off\": \"OFF\",\"pl_on\": \"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_RL_2\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"arduino01/\"}", true);
    client.publish(autodiscovery_binary1_topic, "{\"name\": \"Arduino01 Drzwi 01\",\"stat_t\": \"~SENSORdoor01\",\"avty_t\": \"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_BTN_1\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"arduino01/\",\"device_class\": \"door\",\"pl_on\": \"on\",\"pl_off\": \"off\"}", true);
    client.publish(autodiscovery_binary2_topic, "{\"name\": \"Arduino01 Światło 01\",\"stat_t\": \"~SENSORlight01\",\"avty_t\": \"~tele/LWT\",\"pl_avail\": \"Aktywny\",\"pl_not_avail\": \"Nieaktywny\",\"uniq_id\": \"arduino01_BTN_2\",\"device\": {\"identifiers\": [\"arduino01\"],\"connections\": [[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\": \"arduino01/\",\"device_class\": \"light\",\"pl_on\": \"on\",\"pl_off\": \"off\"}", true);
*/   
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
