#include <SoftwareSerial.h> //библиотека для работы с RS485
#include <ETH.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

//-------- порты для rs 485
#define SSerialRx        19  // Serial Receive pin RO
#define SSerialTx        17   // Serial Transmit pin DI
//-------- инициализация
SoftwareSerial RS485Serial(SSerialRx, SSerialTx); // Rx, Tx
//// линия управления передачи приема
#define SerialControl 18  // RS485 Direction control 5 or 18
/////// флаг приема передачи
#define RS485Transmit    HIGH
#define RS485Receive     LOW
// Определения для сервера MQTT
#define MQTT_HOST IPAddress(212, 92, 170, 246) ///< адрес сервера MQTT
#define MQTT_PORT 1883 ///< порт сервера MQTT

/// создаем объекты для управления MQTT-клиентом:
///Создаем объект для управления MQTT-клиентом и таймеры, которые понадобятся для повторного подключения к MQTT-брокеру или WiFi-роутеру, если связь вдруг оборвется.
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

/////// команды
//byte testConnect[] = { 0x00, 0x00 };
byte testConnect[] = { 0x16, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; // жестко задаем адрес счетчика
byte Access[]      = { 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
byte Sn[]          = { 0x00, 0x08, 0x00 }; // серийный номер
byte Freq[]        = { 0x00, 0x08, 0x16, 0x40 }; // частота
byte Current[]     = { 0x00, 0x08, 0x16, 0x21 };//  ток
byte Suply[]       = { 0x00, 0x08, 0x16, 0x11 }; // напряжение
byte Power[]       = { 0x00, 0x08, 0x16, 0x00 };// мощность
byte Angle[]       = { 0x00, 0x08, 0x16, 0x51 }; // углы
byte activPower[]  = { 0x00, 0x05, 0x00, 0x00 };///  суммарная энергия прямая + обратная + активная + реактивная
byte sumPower[]    = { 0x00, 0x08, 0x11, 0x00 };
byte odometr[]     = { 0x16, 0x05, 0x00, 0x00 }; // команда запроса общего пробега
byte response[19];
int byteReceived;
int byteSend;
int netAdr;

// логин и пароль сети WiFi
const char* ssid = "MikroTik-1EA2D2";
const char* password = "ferrari220";

IPAddress ip;

/*!
 \brief функция подключения к сети wifi
  осуществляет подключение к сети.
 */
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
             //  "Подключаемся к WiFi..."
  WiFi.begin(ssid, password);
IPAddress ip = WiFi.localIP();
}

//Функция подключения к MQTT
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
             //  "Подключаемся к MQTT... "
  mqttClient.connect();
}

//Функция переподключения к Wifi и MQTT  при обрыве связи
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");  //  "Подключились к WiFi"
      Serial.println("IP address: ");  //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
                 //  "WiFi-связь потеряна"
      // делаем так, чтобы ESP32
      // не переподключалась к MQTT
      // во время переподключения к WiFi:
      Serial.printf("SSID=");
      Serial.println(ssid);
      Serial.printf("PASS=");
      Serial.println(password);
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// в этом фрагменте добавляем топики,
// на которые будет подписываться ESP32:
void onMqttConnect(bool sessionPresent) {
  // подписываем ESP32 на топики «phone/ALL», "phone/AUTO":
// пример команды подписки
 uint16_t packetIdSub = mqttClient.subscribe("phone/Counter_22", 0);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
             //  "Отключились от MQTT."
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
             //  "Подписка подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
  Serial.print("  qos: ");  //  "  Уровень качества обслуживания: "
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
            //  "Отписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

/*void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
            //  "Публикация подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

// этой функцией управляется то, что происходит
// при получении того или иного сообщения в топиках

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  // проверяем, получено ли MQTT-сообщение в топике «phone/ALL»:
  if (strcmp(topic, "phone/Counter_22") == 0) {
    if (messageTemp == "1") {
        GetOdo();
    }
  }

}

void setup() {
  // настраиваем сеть


RS485Serial.begin(9600);
Serial.begin(9600);
// 5 пин в режим выхода
pinMode(SerialControl, OUTPUT);
// ставим на прием
digitalWrite(SerialControl, RS485Receive);
//delay(300);

//настройка сети
mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
connectToWifi();
WiFi.onEvent(WiFiEvent);


mqttClient.onConnect(onMqttConnect);
mqttClient.onDisconnect(onMqttDisconnect);
mqttClient.onSubscribe(onMqttSubscribe);
mqttClient.onUnsubscribe(onMqttUnsubscribe);
mqttClient.onMessage(onMqttMessage);
//mqttClient.onPublish(onMqttPublish);
mqttClient.setServer(MQTT_HOST, MQTT_PORT);

Serial.println(" ");
Serial.println("Start_v02.01\r\n");
}

void GetOdo(){
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
  Serial.println(response[i]);
  Serial.println(", ");
}
  delay(1000);
  send(odometr, sizeof(odometr), response);
  for (int i=0; i<19; i++){
Serial.println(response[i]);
Serial.println(", ");
}
  long result_odo;
  result_odo=response[2];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[1];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[4];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[3];
  float r = result_odo;
  float r1 = r/1000.0f;
  Serial.println(r1,3);
  String var = String(r1,3);
  //uint16_t packetIdPub2 = mqttClient.publish("ESP32_Counter/Counter_22", 1, true, var.c_str());

  uint16_t packetIdPub = mqttClient.publish("ESP32Counter/Counter22", 1, true, var.c_str());
}

void loop() {
             while (Serial.available()){
                   char incomingBytes[15];
                   if(Serial.available()>0){
                        Serial.readBytes(incomingBytes,10);
                    }
                    String getS = String(incomingBytes);
                    if(getS.substring(0,5) == "init_") {
                            GetOdo();
                          }
                          }
                          }


/*
String getSerialNumber(int netAdr)
{
  String s1,s2,s3,s4;
  response[0]=0;
  Sn[0] = netAdr;
  send(Sn, sizeof(Sn),response);
  if((int)response[1] < 10) { s1="0" + String((int)response[1]); } else {s1=String((int)response[1]);}
  if((int)response[2] < 10) { s2="0" + String((int)response[2]); } else {s2=String((int)response[2]);}
  if((int)response[3] < 10) { s3="0" + String((int)response[3]); } else {s3=String((int)response[3]);}
  if((int)response[4] < 10) { s4="0" + String((int)response[4]); } else {s4=String((int)response[4]);}
  //String n = String((int)response[1]) + String((int)response[2]) +String((int)response[3])+ String((int)response[4]);
  String n = s1+s2+s3+s4;
  return String(response[0])+";"+n;
}

String getPowerNow(int netAdr)
{
  response[0]=0;
  Power[0] = netAdr;
  send(Power, sizeof(Power),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r);
  r = 0;
  r |= (long)response[10]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getAngle(int netAdr)
{
  response[0]=0;
  Angle[0] = netAdr;
  send(Angle, sizeof(Angle),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getCurrent(int netAdr)
{
  response[0]=0;
  Current[0] = netAdr;
  send(Current, sizeof(Current),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getSuply(int netAdr)
{
  response[0]=0;
  Suply[0] = netAdr;
  send(Suply, sizeof(Suply),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}


String getFreq(int netAdr)
{
  response[0]=0;
  Freq[0] = netAdr;
  send(Freq, sizeof(Freq),response);
  //String n = String((int)response[1]) + String((int)response[2]) +String((int)response[3])+ String((int)response[4]);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String fr= String(r);
  //return fr;
  if(response[0] == netAdr)   return String(response[0])+";"+fr;
  else   return String("Error");
}

String getARPower(int netAdr)
{
  response[0]=0;
  activPower[0] = netAdr;
  send(activPower, sizeof(activPower),response);
  if(response[0] == netAdr)
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }
  //if(response[0] == netAdr)   String(A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  else   return String("Error");
}*/
//////////////////////////////////////////////////////////////////////////////////
void send(byte *cmd, int s, byte *response) {
 // Serial.print("sending...");
  unsigned int crc = crc16MODBUS(cmd, s);
  unsigned int crc1 = crc & 0xFF;
  unsigned int crc2 = (crc>>8) & 0xFF;
  delay(10);
  digitalWrite(SerialControl, RS485Transmit);  // Init Transceiver
       for(int i=0; i<s; i++)
       {
              RS485Serial.write(cmd[i]);
       }
  RS485Serial.write(crc1);
  RS485Serial.write(crc2);
  byte i = 0;
  digitalWrite(SerialControl, RS485Receive);  // Init Transceiver
  delay(200);
         if (RS485Serial.available())
           {
             while (RS485Serial.available())
               {
                byteReceived= RS485Serial.read();    // Read received byte
                delay(10);
                response[i++] = byteReceived;
                }
           }
  delay(20);
}

unsigned int crc16MODBUS(byte *s, int count) {
  unsigned int crcTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    unsigned int crc = 0xFFFF;
    for(int i = 0; i < count; i++) {
        crc = ((crc >> 8) ^ crcTable[(crc ^ s[i]) & 0xFF]);
    }
    return crc;
}
