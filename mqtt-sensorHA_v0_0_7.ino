/* 
 ====== MultiSensorsJolka by Iron ======
 ======      Testing platform     ======
 Arduino Mega 2560 + Ethernet Shield
 Arduino Uno + Ethernet Shield

 ====== Changelog ======
 v0.0.8
 - optymalizacja programu;
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
#include <PubSubClient.h>
#include <DHT.h>
#define SoftVersion "MultiSensorsJolka v0.0.8 by Iron"
#define SoftBuildDate "2020-02-10T21:20:00"
#define ModuleType "Arduino Mega 2560+Ethernet Shield"
#define sensors_topic "dom-88fd54db6487cd19_arduino01/tele/SENSOR"
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
#define autodiscovery_sensor1_topic "homeassistant/sensor/arduino01_AM2302_Temperature/config"
#define autodiscovery_sensor2_topic "homeassistant/sensor/arduino01_AM2302_Humidity/config"
#define autodiscovery_status_topic "homeassistant/sensor/arduino01_status/config"
#define DHTPIN 8 // 8 Pin Arduino do ktorego podlaczony jest czujnik
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
int WebServer = 0;   // 0 - nie uruchamiaj WebServera, 1 - uruchom webserver
int AutoRestart = 0; // 0 - bez restaru, 1 - autorestart
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
//EthernetServer serverweb(80);

void callback(char* topic, byte* payload, unsigned int length) {
  String payloadmsg;
  for (int i=0;i<length;i++) {
    payloadmsg += (char)payload[i];
  }
 if (SerialPrint == 1) {
        Serial.print("topic received = ");
        Serial.println(topic);}
    // Autodiscovery po starcie HA
  if (strcmp (topic, autodiscovery_send_topic) == 0) { autodiscoveryHA(); }
  
  // Pierwsze włączenie arduino - odczyt stanów przekaźników z HA
  if (startprg == 1 || startprg == 2){
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
    }
    if (strcmp (topic, statepower2_topic) == 0) {
      startprg = 3;
      MQTTread += 1;
      if (SerialPrint == 1) { Serial.print("Stan przekaźnika 2 = ");
        Serial.println(payloadmsg);}
      if (payloadmsg == "ON") {
        przekaznik2stan = "ZAŁĄCZONY";
        digitalWrite(PRZEKAZNIK2, LOW); // Załącz przekaźnik 1
      }else {
        przekaznik2stan = "WYŁĄCZONY";
        digitalWrite(PRZEKAZNIK2, HIGH); // Wyłącz przekaźnik 1
      }
    }
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
  } 



void reconnect() {
  // Oczekowani na polaczenie z serwerem
  while (!client.connected()) {
    if (SerialPrint == 1) {Serial.print("Attempting MQTT connection...");}
    // Attempt to connect
    if (client.connect("dom-88fd54db6487cd19_arduino01","dom-88fd54db6487cd19_arduino01/tele/LWT",1,true,"Nieaktywny")) {
    if (SerialPrint == 1) {Serial.println("connected");}
    autodiscoveryHA(); // Przedstawienie się autodiscovery przy starcie
    boolean rc1 = client.subscribe(command1_topic); // Oczykiwanie na topic dla przekaźnika 1
    boolean rc2 = client.subscribe(command2_topic); // Oczykiwanie na topic dla przekaźnika 2
    boolean rcst1 = client.subscribe(statepower1_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 1
    boolean rcst2 = client.subscribe(statepower2_topic); // Oczekiwanie na topic ze statusem dla przekaźnika 2
    boolean rcst3 = client.subscribe(autodiscovery_send_topic); // Oczykiwanie na topic ze statusem dla przekaźnika 2
    } else {
      if (SerialPrint == 1) {Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");}
      // Czekaj 5 sekund przed ponownym polaczeniem
      delay(500);
    }
  }
}

void setup()
{
  if (SerialPrint == 1) {Serial.begin(9600);}
  client.setServer("192.168.2.1", 1883); // Adres serwera MQTT - bramka
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  //if (WebServer==1){  serverweb.begin();}
  dht.begin(); // Start sensor DHT
  pinMode(KONTAKTRON, INPUT_PULLUP); //Kontaktron jako wejście
  pinMode(CZUJNIKSWIATLA, INPUT); // Czujnik światła
  pinMode(PRZEKAZNIK1, OUTPUT); // Ustawienie Pinu przekaznika 1 jako Wyjscie
  pinMode(PRZEKAZNIK2, OUTPUT); // Ustawienie Pinu przekaznika 2 jako Wyjscie
  digitalWrite(PRZEKAZNIK1, HIGH); // Wyłacz przekaźnik 1 przy starcie , LOW = włącz
  digitalWrite(PRZEKAZNIK2, HIGH); // Wyłacz przekaźnik 2 przy starcie , LOW = włącz       
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

}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}
  long lastMsg = 0;
  long lastMsg1 = 0;
  float temp = 0.0;
  float hum = 0.0;
  float diffhum = 1.0; // częstotliwość odswierzania co 1 %
  float diff = 0.1;
  int newKONT = 0; //
  int KONT = 2; // stan 2 przy pierwszym odczycie
  int newlight = 0; //
  int light = 2; // stan 2 przy pierwszym odczycie

void loop()
{
  if (!client.connected()) {
  reconnect();
  }
  client.loop();
  
  long now1 = millis();
  long czekaj;
  if (light == 2 ) {czekaj = 100;} else {czekaj = 60000;}
  if (now1 - lastMsg1 > czekaj) {
    lastMsg1 = now1;
    if (SerialPrint == 1) {Serial.print("Free RAM: "); Serial.println(freeRam());}
    if (SerialPrint == 1) {Serial.print("Minutes : "); Serial.println(millis()*0.000016666666666667);}
    
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
    
  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    float newTemp = dht.readTemperature();
    float newHum  = dht.readHumidity();
    int newKONT = digitalRead(KONTAKTRON);
    int newlight = digitalRead(CZUJNIKSWIATLA);
    String msgSendMQT;
    int changeTEHU = 0;
    
    // ODCZYT TEMPERATURY
    if (checkBound(newTemp, temp, diff)) {
      temp = newTemp;
      changeTEHU = 1;
      if (SerialPrint == 1) {Serial.print("Temperatura: ");
        Serial.println(newTemp,1);}
    }
    // ODCZYT WILGOTNOSCI
    if (checkBound(newHum, hum, diffhum)) {
      hum = newHum;
      changeTEHU = 1;
      if (SerialPrint == 1) {Serial.print("Wilgotność: ");
        Serial.println(newHum,1);}
      //client.publish(humidity_topic, String(newHum,1).c_str(), true);
    }
      if (changeTEHU == 1 ) {
        msgSendMQT = "{\"Time\":\"\",\"AM2302\":{\"Temperature\":"+String(newTemp)+",\"Humidity\":"+String(newHum)+"},\"TempUnit\":\"C\"}";
        client.publish(sensors_topic, String(msgSendMQT).c_str(), true);
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
        if (SerialPrint == 1) {Serial.println("Stan kontaktronu: ZAMKNIETY");}
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
 if ( AutoRestart == 1 ) {
    if (millis() >= RestartTime){
      if (SerialPrint == 1) {Serial.println("Rebooting. . .");}
      void (*reboot)(void) = 0;
      reboot();
    }
 }
  
  /* WEB SERVER
  if (WebServer==1){
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
            client.println("<html>");
            // add a meta refresh tag, so the browser pulls again every 10 seconds:
            client.println("<meta http-equiv=\"refresh\" content=\"10\">");
            client.print("Temperatura: ");
            client.print(String(temp).c_str());
            client.print("<BR>Wilgotnosc: ");
            client.print(String(hum).c_str());
  
            if (KONT == 0) {client.print("<BR>Kontaktron: ZAMKNIETY");}
            if (KONT == 1) {client.print("<BR>Kontaktron: OTWARTY");}
            if (light == 0) {client.print("<BR>Swiatlo: JASNO");}
            if (light == 1) {client.print("<BR>Swiatlo: CIEMNO");}
  
            client.print("<BR>Przekaznik1: ");
            client.print(przekaznik1stan);
            client.print("<BR>Przekaznik2: ");
            client.print(przekaznik2stan);
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
  }
  delay(100);*/
}

void autodiscoveryHA() {
    // Autodiscovery HomeAssistant
    if (SerialPrint == 1) {Serial.println("Sending autodiscovery to HA");Serial.println("Send @ to restart Arduino"); }
    client.publish(autodiscovery_status_topic, "{\"name\":\"Arduino01 status\",\"stat_t\":\"~HASS_STATE\",\"avty_t\":\"~LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"json_attributes_topic\":\"~HASS_STATE\",\"unit_of_meas\":\" \",\"val_tpl\":\"{{value_json['MqttReadCount']}}\",\"ic\":\"mdi:information-outline\",\"uniq_id\":\"arduino01_status\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]],\"name\":\"arduino01\",\"model\":\"arduino01\",\"sw_version\":\"0.0.8\",\"manufacturer\":\"Iron\"},\"~\":\"dom-88fd54db6487cd19_arduino01/tele/\"}", true);
    client.publish(autodiscovery_sensor1_topic, "{\"name\":\"AM2302 Temperature Arduino01\",\"stat_t\":\"~SENSOR\",\"avty_t\":\"~LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"Arduino01_AM2302_Temperature\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_meas\":\"°C\",\"val_tpl\":\"{{value_json['AM2302'].Temperature}}\",\"dev_cla\":\"temperature\"}", true);
    client.publish(autodiscovery_sensor2_topic, "{\"name\":\"AM2302 Humidity Arduino01\",\"stat_t\":\"~SENSOR\",\"avty_t\":\"~LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"Arduino01_AM2302_Humidity\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/tele/\",\"unit_of_meas\":\"%\",\"val_tpl\":\"{{value_json['AM2302'].Humidity}}\",\"dev_cla\":\"humidity\"}", true);
    client.publish(autodiscovery_switch1_topic, "{\"name\":\"Arduino01 Switch 01\",\"cmd_t\":\"~cmnd/POWER1\",\"stat_t\":\"~tele/POWER1\",\"pl_off\":\"OFF\",\"pl_on\":\"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"arduino01_RL_1\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/\"}", true);
    client.publish(autodiscovery_switch2_topic, "{\"name\":\"Arduino01 Switch 02\",\"cmd_t\":\"~cmnd/POWER2\",\"stat_t\":\"~tele/POWER2\",\"pl_off\":\"OFF\",\"pl_on\":\"ON\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"arduino01_RL_2\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/\"}", true);
    client.publish(autodiscovery_binary1_topic, "{\"name\":\"Arduino01 Drzwi 01\",\"stat_t\":\"~tele/SENSORdoor01\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"arduino01_BTN_1\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/\",\"device_class\":\"door\",\"pl_on\":\"on\",\"pl_off\":\"off\"}", true);
    client.publish(autodiscovery_binary2_topic, "{\"name\":\"Arduino01 Światło 01\",\"stat_t\":\"~tele/SENSORlight01\",\"avty_t\":\"~tele/LWT\",\"pl_avail\":\"Aktywny\",\"pl_not_avail\":\"Nieaktywny\",\"uniq_id\":\"arduino01_BTN_2\",\"device\":{\"identifiers\":[\"arduino01\"],\"connections\":[[\"mac\",\"DE:AD:BE:EF:FE:AD\"]]},\"~\":\"dom-88fd54db6487cd19_arduino01/\",\"device_class\":\"light\",\"pl_on\":\"on\",\"pl_off\":\"off\"}", true);
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
