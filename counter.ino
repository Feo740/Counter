#include <SoftwareSerial.h> //библиотека для работы с RS485
#include <ETH.h>
#include <WiFi.h>
#include "DHT.h"
#include <HTTPClient.h> // для работы с гугл таблицами
#include <AsyncMqttClient.h>
#include "GyverStepper2.h" // для работы с шаговым мотором
#include "OneWire.h" // библиотека для работы с 18в20
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

//Подключаем датчик влажности
#define DHTPIN 14     ///< контакт, к которому подключается DHT
#define DHTPIN2 23     ///< контакт, к которому подключается DHT
//#define DHTRELAYPIN 35 // выбираем пин для урпавления питанием датчиков влажности.
#define DHTTYPE DHT22   ///<  DHT 11
#define TEMPSENSORPIN 15 ///< контакт для подключения датчика температуры
OneWire ds(TEMPSENSORPIN);
#define RELAY 22  // контакт реле включения вентиляции

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
//#define MQTT_HOST IPAddress(212, 92, 170, 246) ///< адрес сервера MQTT
#define MQTT_HOST IPAddress(192, 168, 100, 196) ///< адрес сервера MQTT
#define MQTT_PORT 1883 ///< порт сервера MQTT
#define MQTT_USERNAME "feo"
#define MQTT_PASSWORD "ferrari220"
// концевик заслонки вентиляции
#define LIMSW_X 16
//long target = 500; // количество шагов до момента открытия заслонки
String error_flag = "0"; // флаг нахождения заслонки в ошибке 0-работа 1-ошибка
String open_flag = "1"; // флаг открытой заслонки 1-открыт 0-закрыт
String went_flag = "1"; // флаг включенного вентилятора 0-вкл 1-откл

/// создаем объекты для управления MQTT-клиентом:
///Создаем объект для управления MQTT-клиентом и таймеры, которые понадобятся для повторного подключения к MQTT-брокеру или WiFi-роутеру, если связь вдруг оборвется.
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

/////// команды
//byte testConnect[] = { 0x00, 0x00 };
byte testConnect[] = { 0x16, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; // пакет подключения к счетчику
byte Access[]      = { 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//byte Sn[]          = { 0x00, 0x08, 0x00 }; // серийный номер
//byte Freq[]        = { 0x00, 0x08, 0x16, 0x40 }; // частота
byte Current[]     = { 0x00, 0x08, 0x16, 0x21 };//  ток фаза 1
//byte Suply[]       = { 0x00, 0x08, 0x16, 0x11 }; // напряжение
//byte Power[]       = { 0x00, 0x08, 0x16, 0x00 };// мощность
//byte Angle[]       = { 0x00, 0x08, 0x16, 0x51 }; // углы
//byte activPower[]  = { 0x00, 0x05, 0x00, 0x00 };//  суммарная энергия прямая + обратная + активная + реактивная
byte sumPower[]    = { 0x1, 0x08, 0x16, 0x08 };// команда запроса потребляемой мощности
byte odometr[]     = { 0x1, 0x05, 0x00, 0x00 }; // команда запроса общего пробега
byte p_v[]         = { 0x1, 0x08, 0x11, 0x11 }; // команда запроса напряжения по фазе
byte sensor_oil[]  = { 0x28, 0x90, 0xC3, 0x5, 0x5, 0x0, 0x0, 0x77 };// датчик температуры масла
byte response[19];
int byteReceived;
int byteSend;
int netAdr;
int valve_angle; //значение угла открытия вентиляционной заслонки 0-100%
//Массив для данных с терминала
char incomingBytes[15];

byte flag = 0;
byte relay_flag = 1;
byte open_flag_byte = 2;


String odometr_data; //строка пробега считанного со счетчика функцией GetOdo
String voltage_data; // строка значения напряжения считанного функцией GetVoltage по фазе
String current_data; // строка значения тока считанного функцией GetCurrent по фазе
String sumpower_data; // строка значения суммарной мощности по всем фазам
String power_data1; // строка значения  мощности по фазе 1
String power_data2; // строка значения  мощности по фазе 2
String power_data3; // строка значения  мощности по фазе 3

//String voltage_data2; // строка значения напряжения считанного функцией GetVoltage по фазе 2
//String voltage_data3; // строка значения напряжения считанного функцией GetVoltage по фазе 3
// дней*(24 часов в сутках)*(60 минут в часе)*(60 секунд в минуте)*(1000 миллисекунд в секунде)
const unsigned long period_counter PROGMEM = 43200000;//86400000;  ///< таймер для проверки счетчика, раз в сутки
unsigned long p_counter = 0; ///< Техническая переменная счетчика таймера
unsigned int period_DHT22 = 60000;  ///< таймер для датчика влажности
unsigned int period_18b20_1 = 60000;  ///< таймер для первого датчика температуры
unsigned long period_clapan = 10000; //таймер для проверки что заслонка не застряла
unsigned long period_voltage = 5000; // таймер для снятия показаний напряжения и тока
unsigned long p_clapan = 0; ///< Техническая переменная счетчика таймера
unsigned int period_18b20_read = 500; ///< таймер ожидания преобразования в датчике 18b20
unsigned long dht22 = 0; ///< Техническая переменная счетчика таймера
unsigned long T18b20_1 = 0; ///< Техническая переменная счетчика таймера
unsigned long read_18b20 = 0; ///< Техническая переменная счетчика таймера
unsigned long voltageP = 0; ///< Техническая переменная счетчика таймера для снятия напр и тока

// логин и пароль сети WiFi
//const char* ssid = "MikroTik-1EA2D2";
//const char* password = "ferrari220";
//const char* ssid = "US_WIFI";
//const char* password = "beeline2022";
const char* ssid = "4G-UFI-3a43";
const char* password = "1234567890";
String GOOGLE_SCRIPT_ID = "AKfycbxwurwRRddUcZicLEqtov0QGkh9jDIjnCa8uorSOR40XKirSNvfyvXQqiIgGy0tZUTZ"; //ID Google таблички

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
IPAddress ip;

GStepper2<STEPPER4WIRE> stepper(2048, 33, 25, 26, 27); // кол-во шагов,пины шаговика

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
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // подписываем ESP32 на топики:
 uint16_t packetIdSub2 = mqttClient.subscribe("phone/Counter", 0); //топик счетчика электроэнергии
 uint16_t packetIdSub3 = mqttClient.subscribe("phone/Went", 0);//топик мотора вытяжки
 uint16_t packetIdSub4 = mqttClient.subscribe("phone/Went_valve", 0);//топик клапана вытяжки
 uint16_t packetIdSub5 = mqttClient.subscribe("phone/Zopen", 0);//топик клапана вытяжки
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
  // проверяем, получено ли MQTT-сообщение в топике «phone/Zopen»:
  if (strcmp(topic, "phone/Zopen") == 0) {
    if (messageTemp == "1" && open_flag == "0") {
    open_flag_byte=1;
  } else {
            open_flag_byte = 0;
                }
  }
  // проверяем, получено ли MQTT-сообщение в топике «phone/Counter»:
  if (strcmp(topic, "phone/Counter") == 0) {
    if (messageTemp == "1") {
      flag = 1;
          }
  }
  // проверяем, получено ли MQTT-сообщение в топике "phone/Went"
  if (strcmp(topic, "phone/Went") == 0) {
    if (messageTemp == "1") {
      relay_flag = 1;
          }
    if (messageTemp == "0") {
            relay_flag = 0;
                }
  }

  // проверяем, получено ли MQTT-сообщение в топике "phone/Went_valve"
  if (strcmp(topic, "phone/Went_valve") == 0) {
    valve_angle=messageTemp.toInt();
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
mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

Serial.println(" ");
Serial.println("Start_v02.01\r\n");

dht22 = millis();
p_counter = millis();
dht.begin();
dht2.begin();
pinMode(RELAY,OUTPUT);
//pinMode(DHTRELAYPIN,OUTPUT); //задаем пин реле датчиков влажности как выход

// пуллапим. Кнопки замыкают на GND
pinMode(LIMSW_X, INPUT_PULLUP);
//stepper.setRunMode(FOLLOW_POS);
homing(); // по умолчанию заслонку закрываем
}

//Функция закрытия заслонки
void homing() {
  open_flag = "0";
  // сообщаем, что заслонка закрылась
  String var = "ESP32Counter/Zopen";
  char var2[19];
  var.toCharArray(var2,19);
  uint16_t packetIdPub = mqttClient.publish(var2, 1, true, open_flag.c_str());
  stepper.disable();                // тормозим, приехали
  stepper.reset(); // сбросить текущую позицию в 0
  p_clapan = millis();
  if (digitalRead(LIMSW_X)) {       // если концевик X не нажат
    stepper.setSpeed(-100);       // ось Х, -10 шаг/сек
    while (digitalRead(LIMSW_X)) {  // пока кнопка не нажата
      stepper.tick();               // крутим
      if ((millis()-p_clapan)>=period_clapan){ // проверяем, не застряла ли наша заслонка в открытом виде
        // пишем мкутт сообщение об ошибке
        error_flag = "1";
        open_flag = "1";
        //сообщаем об ошибке
        String var = "ESP32Counter/Zerror";
        char var1[20];
        var.toCharArray(var1,20);
        uint16_t packetIdPub = mqttClient.publish(var1, 1, true, error_flag.c_str());
        // сообщаем что заслонка осталась открыта
        var = "ESP32Counter/Zopen";
        char var2[19];
        var.toCharArray(var2,19);
        uint16_t packetIdPub2 = mqttClient.publish(var2, 1, true, open_flag.c_str());
        break;
      }
    }
    // кнопка нажалась - покидаем цикл
    //сообщаем об отсутствии ошибки
    error_flag = "0";
    String var = "ESP32Counter/Zerror";
    char var1[20];
    var.toCharArray(var1,20);
    uint16_t packetIdPub = mqttClient.publish(var1, 1, true, error_flag.c_str());

  }
  open_flag_byte = 2;
}

//Функция открытия заслонки
void vent_open(){
  // MQTT сообщение  - заслонка открыта.
  open_flag = "1";
  String var = "ESP32Counter/Zopen";
  char var1[19];
  var.toCharArray(var1,19);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, open_flag.c_str());
  open_flag_byte = 2;
  // target = valve_angle*2,5; на будущее, если что-то открывать на угол
  stepper.setTarget(500);
  stepper.setMaxSpeed(100);
  while (!stepper.ready()) {  // пока кнопка не нажата
  stepper.tick();               // крутим
}
if(stepper.ready()){
  stepper.disable();
  }
}

void send_mqtt(String &value, const char* addr){
  uint16_t packetIdPub = mqttClient.publish(addr, 1, true, value.c_str());
}

//Функция считывает параметр "потребляемая мощность"
void GetPower(byte number){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  sumPower[0] = number; // определеяем адрес четчика с которого считываем мощность
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(sumPower, sizeof(sumPower), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
  }
  //Формируем результат суммарной мощности
  Serial.println("");
  long result_power=0; // переменная для общей мощности
  long result_power1=0; // переменная для мощности по первой фазе
  long result_power2=0; // переменная для мощности по второй фазе
  long result_power3=0; // переменная для мощности по третьей фазе
  result_power = response[3];
  result_power=result_power<<8;
  result_power=result_power+response[2];
  //result_power=result_power<<8;
  //result_power=result_power+response[2];
  // Формируем результат мощности по фазе 1
  result_power1 = response[6];
  result_power1=result_power1<<8;
  result_power1=result_power1+response[5];
  //result_power1=result_power1<<8;
  //result_power1=result_power1+response[5];
  // Формируем результат мощности по фазе 2
  result_power2 = response[9];
  result_power2=result_power2<<8;
  result_power2=result_power2+response[8];
  //result_power2=result_power2<<8;
  //result_power2=result_power2+response[8];
  // Формируем результат мощности по фазе 3
  result_power3 = response[12];
  result_power3=result_power3<<8;
  result_power3=result_power3+response[11];
  //result_power3=result_power3<<8;
  //result_power3=result_power3+response[11];

  //Обработка результата суммарной мощности
  float r = result_power;
  float r1 = r/100.0f;
  Serial.println(r1,3);
  sumpower_data = String(r1);
  // формируем топик ESP32Counter/Counter40/SumPower
  String var2 = "ESP32Counter/Counter"+String(number)+"/SumPower";
  Serial.println("string");
  Serial.println(var2);
  char var1[32];
  var2.toCharArray(var1,32);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, sumpower_data.c_str());
  sumpower_data.replace(".",",");

  //Обработка результата мощности по фазе 1
  r = result_power1;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data1 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase1";
  Serial.println("string");
  Serial.println(var2);
  char var3[35];
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data1.c_str());
  power_data1.replace(".",",");

  //Обработка результата мощности по фазе 2
  r = result_power2;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data2 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase2
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase2";
  Serial.println("string");
  Serial.println(var2);
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data2.c_str());
  power_data2.replace(".",",");

  //Обработка результата мощности по фазе 3
  r = result_power3;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data3 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase3";
  Serial.println("string");
  Serial.println(var2);
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data3.c_str());
  power_data3.replace(".",",");

  //обнуляем массив принятого ответа
  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

void power(){
  GetPower(40);
  String param;
  param  = "box40SumPower="+sumpower_data;
  param += "&box40Power1="+power_data1;
  param += "&box40Power2="+power_data2;
  param += "&box40Power3="+power_data3;
  write_to_google_sheet(param);
}

////Функция считывает параметр ток по определенной фазе
void GetCurrent (byte number){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  Current[0] = number; // определеяем адрес четчика с которого считываем ток
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(Current, sizeof(Current), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  //Выделяем значение тока по фазе 1
  long result_current=0;
  result_current=response[3];
  result_current=result_current<<8;
  result_current=result_current+response[2];
  float r = result_current;
  float r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  String var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase1";
  Serial.println("string");
  Serial.println(var2);
  char var1[37];
  var2.toCharArray(var1,37);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  //Выделяем значение тока по фазе 2
  result_current=0;
  result_current=response[6];
  result_current=result_current<<8;
  result_current=result_current+response[5];
   r = result_current;
   r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase2";
  Serial.println("string");
  Serial.println(var2);
  var1[37];
  var2.toCharArray(var1,37);
  packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  //Выделяем значение тока по фазе 3
  result_current=0;
  result_current=response[9];
  result_current=result_current<<8;
  result_current=result_current+response[8];
   r = result_current;
   r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase3";
  Serial.println("string");
  Serial.println(var2);
  var1[37];
  var2.toCharArray(var1,37);
  packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

//Функция считывает параметр "напряжение по фазе"
void GetVoltage (byte number, byte phase_voltage){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  p_v[0] = number; // определеяем адрес четчика с которого считываем напряжение по фазе
  p_v[3] = phase_voltage; // определеям фазу, по которой считываем значение
  // Опрос счетчика
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(p_v, sizeof(p_v), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  long result_voltage=0;
  result_voltage=response[3];
  result_voltage=result_voltage<<8;
  result_voltage=result_voltage+response[2];
  float r = result_voltage;
  float r1 = r/100.0f;
  Serial.println(r1,3);
  byte phase;
  if(phase_voltage == 0x11) {phase = 1;}
  if(phase_voltage == 0x12) {phase = 2;}
  if(phase_voltage == 0x13) {phase = 3;}
  voltage_data = String(r1);
  // формируем топик ESP32Counter/Counter40/VoltagePhase1
  String var2 = "ESP32Counter/Counter"+String(number)+"/VoltagePhase"+String(phase);
  Serial.println("string");
  Serial.println(var2);
  char var1[37];
  var2.toCharArray(var1,37);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, voltage_data.c_str());
  voltage_data.replace(".",",");
  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

// на основе данных функции GetCurrent, формирует полный пакет опроса счетчика
void current(){
  GetCurrent(40);
  delay(500);
  GetCurrent(85);
}

// на основе данных функции GetVoltage, формирует полный пакет опроса счетчика
void voltage(){
  GetVoltage(40, 0x11);
  //String param;
  //param  = "box40Voltage1="+voltage_data;
  GetVoltage(40, 0x12);
  //param += "&box40Voltage2="+voltage_data;
  GetVoltage(40, 0x13);
  //param += "&box40Voltage3="+voltage_data;
  //write_to_google_sheet(param);
}

// Функция считывает пробег со счетчика, формирует MQTT сообщение с пробегом, текстовую переменную odometr_data для формирования строки гугл-табли
void GetOdo(byte number){
  testConnect[0] = number;
  odometr[0] = number;
  // Опрос счетчика 22
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(odometr, sizeof(odometr), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  long result_odo=0;
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
  odometr_data = String(r1);
  String var2 = "ESP32Counter/Counter"+String(number);
  char var1[23];
  var2.toCharArray(var1,23);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, odometr_data.c_str());
  odometr_data.replace(".",",");
  for (int i=0; i<19; i++){
    response[i]=0;
  }
}

// на основе данных функции GetOdo, формирует полный пакет опроса счетчика
void odo(){
  GetOdo(40);
  String param;
  param  = "box40="+odometr_data;
  delay(500);
  GetOdo(85);
  param += "&box85="+odometr_data;
  write_to_google_sheet(param);
}
//Функция отправки данных в гугл таблицу
void write_to_google_sheet(String params) {
   HTTPClient http;
   String url="https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+params;
   //Serial.print(url);
    Serial.println("Posting data to Google Sheet");
    //---------------------------------------------------------------------
    //starts posting data to google sheet
    http.begin(url.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;
    if (httpCode > 0) {
        //payload = http.getString();
        Serial.println("Payload: ");//+payload);
    }
    //---------------------------------------------------------------------
    http.end();
}

void Read_18b20(byte addr[8], int t){
  //переменные для датчиков температуры
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius, fahrenheit;
  String result;


  ds.reset();
  ds.select(addr);
  ds.write(0x44, 0);        // признак выбора режима питания 0-внешнее 1-паразитное
  //delay(1000);
  if ((millis() - read_18b20) >= period_18b20_read) {

  read_18b20 = millis(); // обнуляем таймер на полсекунды - обработка датчиком


  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // читаем результат

  for ( i = 0; i < 9; i++) {           // нам требуется 9 байтов
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  //  if (flag == 1) {
  result = String(celsius);
//} else{
  //result = "**.**";
//}

if (t == 15){
  uint16_t packetIdPub2 = mqttClient.publish("Counter/oil_temp", 1, true, result.c_str());
  Serial.print("температура:");
  Serial.println(result.c_str());
}
  return;
}
}

void loop() {

  // Читаем датчик 18b20 + обновляем заслонку
if ((millis() - T18b20_1) >= period_18b20_1) {
    Read_18b20(sensor_oil, 15);
    T18b20_1 = millis(); // обнуляем таймер опроса датчика
    if (digitalRead(LIMSW_X && open_flag_byte == 0)){//если концевик не нажат, а из HA пришла команда закрыть
      homing();
    }
    send_mqtt(open_flag, "ESP32Counter/Zopen"); //отправляем регулярно состояние заслонки

    //раз в 6с отпр топик состояния заслонки.
          }

// Снятие данных счетчика раз в сутки
if ((millis() - p_counter) >= pgm_read_dword(period_counter)) {
  p_counter = millis();
  odo();
}

// снятие данных напряжения и тока по таймеру
if ((millis() - voltageP) >= period_voltage) {
  voltageP = millis();
  voltage();
  delay(500);
  current();
}
// Снятие данных по таймеру с датчика влажности
if ((millis() - dht22) >= period_DHT22) {
//  digitalWrite(DHTRELAYPIN,LOW); // подаем питание на датчики
  dht22 = millis();

// обработка датчика 1
  float h = dht.readHumidity(); // считывание данных о температуре и влажности датчика1
  delay(70);
  float t = dht.readTemperature();// считываем температуру в градусах Цельсия с датчика1
  delay(70);

  // проверяем, корректно ли прочитались данные,
  // и если нет, то выходим и пробуем снова:
  if (isnan(h) || isnan(t)) {
    Serial.print("Failed to read from DHT1 sensor!"); // "Не удалось прочитать данные с датчика DHT!"
    } else {
    String hum = String(h);
    String temp = String(t);
    Serial.print("Temp: "+ temp + " Hum: " + hum);
    String var = "ESP32Counter/Temp";
    char var1[18];
    var.toCharArray(var1,18);
    uint16_t packetIdPub = mqttClient.publish(var1, 1, true, temp.c_str());
    Serial.print("температура DHT22:");
    Serial.println(temp.c_str());
    var = "ESP32Counter/Hum";
    char var2[17];
    var.toCharArray(var2,17);
    packetIdPub = mqttClient.publish(var2, 1, true, hum.c_str());
    Serial.print("влажность DHT22:");
    Serial.println(+hum.c_str());
  }
// конец обработки датчика 1

// обработка датчика 2
    h = dht2.readHumidity(); // считывание данных о температуре и влажности датчика2
    delay(70);
    t = dht2.readTemperature();// считываем температуру в градусах Цельсия с датчика2
    delay(70);

// проверяем, корректно ли прочитались данные,
// и если нет, то выходим и пробуем снова:
if (isnan(h) || isnan(t)) {
  Serial.print("Failed to read from DHT2 sensor!"); // "Не удалось прочитать данные с датчика DHT!"
  } else {
  String hum = String(h);
  String temp = String(t);
  Serial.print("Temp2: "+ temp + " Hum2: " + hum);
  String var = "ESP32Counter/Temp2";
  Serial.println(var);
  char var1[19];
  var.toCharArray(var1,19);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, temp.c_str());
  Serial.print("температура DHT22 №2:");
  Serial.println(temp.c_str());
  var = "ESP32Counter/Hum2";
  Serial.println(var);
  char var2[18];
  var.toCharArray(var2,18);
  packetIdPub = mqttClient.publish(var2, 1, true, hum.c_str());
  Serial.print("влажность DHT22 №2:");
  Serial.println(hum.c_str());
  }
// конец обработки датчика 2
//  digitalWrite(DHTRELAYPIN,HIGH); // снимаем питание с датчиков
}
// проверка флага опроса счетчика
if (flag == 1){
  odo();
  voltage();
  current();
  flag = 0;
  }
// проверка флага состояния включения вентилятора
if (relay_flag ==1){
  digitalWrite(RELAY,HIGH); //если флаг 1 реле выключить
  went_flag = "1";
  String var = "ESP32Counter/went1";
  char var2[19];
  var.toCharArray(var2,19);
  uint16_t packetIdPub = mqttClient.publish(var2, 1, true, went_flag.c_str());
}
if (relay_flag ==0){
  // если пришло сообщ вкл вентилятор, проверяем открыта ли заслонка и не в ошибке ли она висит
  if(open_flag=="1" & error_flag=="0"){
  digitalWrite(RELAY,LOW); // если флаг 0 реле включить
  went_flag = "0";
  // сообщаем, что заслонка закрылась
  String var = "ESP32Counter/went1";
  char var2[19];
  var.toCharArray(var2,19);
  uint16_t packetIdPub = mqttClient.publish(var2, 1, true, went_flag.c_str());
  Serial.println("Went start");
} else{
  digitalWrite(RELAY,HIGH); // если флаг 0 реле включить
  went_flag = "1";
  // сообщаем, что заслонка закрылась
  String var = "ESP32Counter/went1";
  char var2[19];
  var.toCharArray(var2,19);
  uint16_t packetIdPub = mqttClient.publish(var2, 1, true, went_flag.c_str());
  Serial.println("Went stop - error");
}
}
if (open_flag_byte == 1){
  vent_open();
}
if (open_flag_byte == 0){
  homing();
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
