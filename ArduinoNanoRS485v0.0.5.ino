//Version0.0.5 - Send sensor data after server request
//Version0.0.4 - OLED 0.91, RF433 receiver
//Version0.0.3 - LCD 4x20
//Version0.0.2 - Add BME280 sensor
//Version0.0.1 - Light Sensor
//
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C // 0X3C+SA0 - 0x3C or 0x3D 0x3C - Wyświetlacz OLED
#define RST_PIN -1 // Define proper RST_PIN if required.
#include "RS485_protocol.h"
#include <SoftwareSerial.h>
#include <RCSwitch.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#define LED_LIGHT 7 //Light LED D7
const byte ENABLE_PIN = 6; //D6 RS485 Enable pin DE+RE
char light_sensor_pin = A0; // led lamp
char fire_sensor_pin = A3; // flame/fire sensor
int siren_pin = 3; //D3 Buzzer
int fire_sensor;
byte light_on = false;

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

int sensorValue = 0;
long lastMsg = 0;
long lastMsgrcv = 0;
int nowrcv = 0;
int minut;
int godzin;
int dni;
int srverr = 0;
long srvTotalerr = 0;
String Serwer = "";
int srvrecv = 0;

float bmetemp = 0.00;
float bmehum = 0.00;
float bmepress = 0.00;
float bmealt = 0.00;
int light = 0;
float settemp = 20.0;
byte fire = false;
byte lamp = false;

SSD1306AsciiAvrI2c oled;
RCSwitch mySwitch = RCSwitch();
Adafruit_BME280 bme;
SoftwareSerial rs485 (4, 5);  // receive pin, transmit pin

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
  Serial.begin(9600);
  delay(1);
  rs485.begin (9600);
  Serial.println("Serial Begin Arduino Nano");
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  mySwitch.enableReceive(0);  //pin D2 433in
  pinMode(fire_sensor_pin,INPUT); // Fire sensor
  pinMode (ENABLE_PIN, OUTPUT);  // driver output enable
  pinMode (LED_BUILTIN, OUTPUT);  // built-in LED
  pinMode (LED_LIGHT, OUTPUT);  // built-in LED
  pinMode (siren_pin, OUTPUT);  // Buzzer pin

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0
  
  bmetemp = bme.readTemperature();
  bmehum = bme.readHumidity();
  bmepress = bme.readPressure() / 100.0F;
  bmealt = bme.readAltitude(SEALEVELPRESSURE_HPA); 
  sensorValue = analogRead(light_sensor_pin); //Swiatlo
  light = map(sensorValue, 1016, 5, 0, 100);
  
}  // end of setup

void loop()
{
   // RF433 receive
  if (mySwitch.available()) {
    output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    if ( mySwitch.getReceivedValue() == 6792738) { 
      digitalWrite (LED_BUILTIN, HIGH);  // turn on LED if received
      settemp = settemp+0.5;
      Serial.print("SET TEMP + : ");  
      Serial.println(settemp);}
    if ( mySwitch.getReceivedValue() == 6792737) { 
      digitalWrite (LED_BUILTIN, HIGH);  // turn on LED if received
      settemp = settemp-0.5;
      Serial.print("SET TEMP - : ");  
      Serial.println(settemp);}
    if ( mySwitch.getReceivedValue() == 6792740) { 
      fire= false;
      if (light_on == true){
        digitalWrite(LED_LIGHT, LOW); // gasimy Lampkę
        light_on = false;
        lamp = false;
        noTone(siren_pin);
        digitalWrite (siren_pin, LOW);
      }else {
        digitalWrite(LED_LIGHT, HIGH); // zapalamy Lampkę
        light_on = true;
        lamp = true;
        }
      
      }

    mySwitch.resetAvailable();
    delay(1);
    digitalWrite (LED_BUILTIN, LOW);  // turn off LED
  }
  
 // receive invitation from Server
  byte rbuf [2];
  byte received = recvMsg (fAvailable, fRead, rbuf, sizeof rbuf);
  if (received) {
    if (rbuf[0] == 1){
      if (rbuf[1] == 1){
        Serial.println("");  
        Serial.println("OK - Response invitation from Server!");
        Serwer = "OK !";
        srverr = 0;   
        srvrecv++;   
        // Send sensors state
        sendData(1); // Send data to master from sensor 1 
        digitalWrite (LED_BUILTIN, LOW);  // turn off LED if OK
      }
    }
  } else {
      srverr++; 
      // If no response from server 10 times
      if (srverr >= 50){       
        srvTotalerr++;
        srverr=0;
        Serial.println("NO - Response from Server 10 times!");                   
        Serial.print("Total errors: ");
        Serial.println(srvTotalerr);
        digitalWrite (LED_BUILTIN, HIGH);  // turn on LED if error
        Serwer = "ERR!";
        }
  }// end !received


 

  // run every 3 sec
  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    minut = millis()/1000 / 60;
    godzin = millis()/1000 / 3600;
    dni = godzin/24;
    fire_sensor = analogRead(fire_sensor_pin);
    if (fire_sensor < 1019){
      fire = true;
      digitalWrite(LED_LIGHT, HIGH); // zapalamy Lampkę
      digitalWrite(LED_BUILTIN, HIGH); // zapalamy LED
    }
    if (fire == true){Siren();}
    
    bmetemp = bme.readTemperature();
    bmehum = bme.readHumidity();
    bmepress = bme.readPressure() / 100.0F;
    bmealt = bme.readAltitude(SEALEVELPRESSURE_HPA); 
    sensorValue = analogRead(light_sensor_pin); //Swiatlo
    light = map(sensorValue, 1016, 5, 0, 100);
    displayoled(Serwer); // Display OLED
    displayserial();
  } //end 3s

}  // end of main loop

void displayserial(){
  // Send sensors state
  Serial.print("Temperature = ");
  Serial.print(bmetemp);
  Serial.println("*C");
  Serial.print("Humidity = ");
  Serial.print(bmehum);
  Serial.println("%");
  Serial.print("Pressure = ");
  Serial.print(bmepress);
  Serial.println("hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(bmealt);
  Serial.println("m");  
  Serial.print("Light = ");
  Serial.print(light);
  Serial.println("%");
  Serial.print("SetTemp = ");
  Serial.print(settemp);
  Serial.println("*C");  
  Serial.print("Fire = ");
  Serial.println(fire_sensor);
  Serial.print("Fire = ");
  Serial.println(fire);
  Serial.print("Lamp = ");
  Serial.println(lamp);
}       

void displayoled(String srv) {
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.setInvertMode(0%2);
  oled.set2X();
  oled.print(bmetemp);
  oled.print( "*C ");
  oled.setCol(221);
  oled.print(light);
  oled.println("%"); 
  oled.setCol(0);
  oled.print(bmehum);
  oled.println( " %");
  if (fire == true){
    oled.setCol(221);
    oled.print(" FIR");}
  oled.setCol(221);
  oled.print(bmepress);
  oled.println(" hPa"); 

  for (int i = 0; i < 4; i++) {
  // Toggle invert mode for next line of text.
    oled.setInvertMode((0)%2);
    oled.print("S:");
    oled.setInvertMode(i%2);
    oled.print(srv);
    oled.setInvertMode((0)%2);
    oled.print("E:");
    oled.println(srvTotalerr);
    delay(300);
  }
}// End DisplayOLED

// Send data to Master Arduino Mega 
int sendData(int sensorID)
{
  byte byteArray[PACKET_SIZE];    
  leakinfo.sensor.stat = 1; 
  leakinfo.sensor.sensorId = sensorID; 
  leakinfo.sensor.temp = bmetemp; 
  leakinfo.sensor.hum =  bmehum; 
  leakinfo.sensor.pressure = bmepress; 
  leakinfo.sensor.alt = bmealt; 
  leakinfo.sensor.light = light; 
  leakinfo.sensor.settemp = settemp; 
  leakinfo.sensor.fire = fire; 
  leakinfo.sensor.lamp = lamp; 
    
  delay (1);  // give the master a moment to prepare to receive
  digitalWrite (ENABLE_PIN, HIGH);    // enable sending
  sendMsg (fWrite, leakinfo.rs485Packet, PACKET_SIZE);  // send confirmation  
  digitalWrite (ENABLE_PIN, LOW);     // disable sending   
} // end sendData

void Siren(){
  digitalWrite(siren_pin, HIGH);
 delay(300);
for(int i=3;i<=6;i++)
noTone(siren_pin);
tone(3,494,500);
delay(300);

noTone(siren_pin);
tone(siren_pin,523,300);
delay(200);

delay(50);

delay(50);
noTone(siren_pin);
}
