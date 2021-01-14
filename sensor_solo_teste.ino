#include <math.h>       // Conversion equation from resistance to %
#include "adc_corrected.h"
#include "ArduinoSort.h"
#include "Statistic.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <HX711.h>
#include "ThingSpeak.h"
#include "Servo.h"
#include <ArduinoJson.h>

unsigned long myChannelNumber = 1110526;
const char * myWriteAPIKey = "2JT0XT1NLS0GOJ4W";

// DEFINIÇÕES DE PINOS
#define pinDTScale1  16
#define pinSCKScale1  17

#define pinDTScale2  5
#define pinSCKScale2  18

// DEFINIÇÕES
#define pesoMin 0.010
#define pesoMax 30.0

// Balança CBH5453
//#define escala1 -271420.0f
#define escala1 -271920
//#define LOADCELL_OFFSET_1 -27300

// Balança SF-400
#define escala2 -265120
//#define LOADCELL_OFFSET_2 -131600
//#define escala2 -264627.15f

// INSTANCIANDO OBJETOS
HX711 scale1, scale2;

Servo meuservo; // Cria o objeto servo para programação

int angulo = 0;
#define movimento  60
#define  passosDelay 30

// Setting up format for reading 3 soil sensors
#define NUM_READS 10                              /* Number of sensor reads for filtering  */
#define uS_TO_S_FACTOR    1000000                 /* Conversion factor for micro seconds to seconds */
#define S_TO_MIN_FACTOR   60 * uS_TO_S_FACTOR     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP     30                      /* Time ESP32 will go to sleep (in seconds) */
int TIMES_TO_MEASURE = 24;                      /* Time ESP32 will go to sleep (in seconds) */
int MEASURE_INTERVAL =  10;                   /* Time ESP32 will go to sleep (in seconds) */
int qttOfMeasures = 0;

struct values {        // Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
  int supplyVoltageRaw;
  float supplyVoltage;
  int sensorVoltageRaw;
  float sensorVoltage;
  int moisture;
  long resistance;
};

struct Sensor {
  int id;
  int moisture;
  int phase_a;
  int phase_b;
  int analog_input;
  int analog_input2;
  adc1_channel_t channel;
  adc1_channel_t channel2;
  long knownResistor;
  float weight;
  values lastMeasure;
};

Sensor sensor1, sensor2;
Sensor *sensor1_ptr, *sensor2_ptr;
const long knownResistor = 4700;  // Constant value of known resistor in Ohms

#define DEFAULT_VREF    1086

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
esp_adc_cal_characteristics_t adc_cal;

// -----------------------------------------------------------------#
#define WIFISSID01 "extensao_iot" // Put your WifiSSID here
#define PASSWORD01 "aluno123" // Put your wifi password here

#define WIFISSID02 "IFPE-PUBLICA" // Put your WifiSSID here
#define PASSWORD02 "aluno123" // Put your wifi password here

#define WIFISSID03 "GVT-F051" // Put your WifiSSID here
#define PASSWORD03 "1316309065" // Put your wifi password here

char servidorMqttElton[] = "eltontorres.asuscomm.com";
uint16_t portaServidorMqttElton = 1883;

char servidorMqtt[] = "192.168.1.187";
uint16_t portaServidorMqtt = 1883;
char tokenMqttDisp[33] = "ESP32_UMIDADE_SOLO";

WiFiMulti wifiMulti;
WiFiClient wifiClient;
WiFiClient wifiClientElton;

char attributesTopic[] = "v1/devices/me/attributes";
char attributesRequestTopic[] = "v1/devices/me/attributes/request/1";
char attributesResponseTopic[] = "v1/devices/me/attributes/response/1";

PubSubClient client(wifiClient);
PubSubClient clientElton(wifiClientElton);

long lastTimeSentToThingspeak = millis();
unsigned long lastSend = 0;

//  ----------------------------------------------------------------------------------- //
RTC_DATA_ATTR int bootCount = 0;

void setup() {
  Serial.begin(9600);
  meuservo.attach(4);
  lastTimeSentToThingspeak = millis();
  setupChannelsAndPins();
  setupScales();
  startWifiClient();
  imprimirInformacoesWifiEMQTT();
  sensor1_ptr = &sensor1;
  sensor2_ptr = &sensor2;
  declareSensor(sensor1_ptr, 5, 22, 23, 35, 34, ADC1_CHANNEL_7, ADC1_CHANNEL_6, knownResistor);
  declareSensor(sensor2_ptr, 6, 19, 21, 33, 32, ADC1_CHANNEL_5, ADC1_CHANNEL_4, knownResistor);
  //setupDeepSleep();
  ThingSpeak.begin(wifiClient);
}

void loop() {
  if (!client.connected() ) {
    conectarMQTT(client);
  }
  if (!clientElton.connected() ) {
    conectarMQTT(clientElton);
  }
  if (isTimeToMeasure()) {
    if (conectarWifi()) {
      //if (qttOfMeasures < TIMES_TO_MEASURE) {
      resetScale();
      excitateSensor(sensor1_ptr);
      excitateSensor(sensor2_ptr);
      putSensorToImpedance(sensor1_ptr);
      putSensorToImpedance(sensor2_ptr);
      measureSensor(sensor1_ptr);
      measureWeight(sensor1_ptr, scale1, 1);
      sendSensorInfoToMQTTServer(sensor1, client);
      sendSensorInfoToMQTTServer(sensor1, clientElton);
      sendSensorInfoToThingspeak(sensor1, 1, 2);

      putSensorToImpedance(sensor1_ptr);
      putSensorToImpedance(sensor2_ptr);
      measureSensor(sensor2_ptr);
      measureWeight(sensor2_ptr, scale2, 2);
      sendSensorInfoToMQTTServer(sensor2, client);
      sendSensorInfoToMQTTServer(sensor2, clientElton);
      sendSensorInfoToThingspeak(sensor2, 3, 4);
      liftUp();
      Serial.println("//--------------------------------------------------------------//");
      //delayMicroseconds (MEASURE_INTERVAL * S_TO_MIN_FACTOR);
      qttOfMeasures++;
      lastSend = millis();
      //} else {
      //      liftUp();
      //      Serial.println("Going to sleep now");
      //      Serial.flush();
      //      esp_deep_sleep_start();
      //}
    }
  }
  client.loop();
  clientElton.loop();
}

boolean isTimeToMeasure() {
  return lastSend == 0 || ((millis() - lastSend) > (MEASURE_INTERVAL * 60000));
}

void excitateSensor(Sensor *sensor) {
  excitateSensorIntern(sensor->phase_a, sensor->phase_b);
}

void excitateSensorIntern (int phase_a, int phase_b) {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  for (int i = 0; i < 100; i++) {
    pinMode(phase_a, OUTPUT);
    pinMode(phase_b, INPUT);
    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off

    delayMicroseconds(100);

    pinMode(phase_a, INPUT);
    pinMode(phase_b, OUTPUT);
    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off
  }
}

void putSensorToImpedance(Sensor *sensor) {
  pinMode(sensor->phase_a, INPUT);
  pinMode(sensor->phase_b, INPUT);
}

void setupDeepSleep() {
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_MIN_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                 " Seconds");
}

void setupChannelsAndPins() {
  adc1_config_width(ADC_WIDTH_BIT_12);//Configura a resolucao
  adc1_config_channel_atten(ADC1_CHANNEL_4, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_5, atten);//Configura a atenuacao

  adc1_config_channel_atten(ADC1_CHANNEL_6, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_7, atten);//Configura a atenuacao

  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_cal);

  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void declareSensor(Sensor *sensor, int id, int phase_a, int phase_b, int analog_input, int analog_input2, adc1_channel_t channel, adc1_channel_t channel2, long knownResistor) {
  sensor->id = id;
  //  sensor.moisture;
  sensor->phase_a = phase_a;
  sensor->phase_b = phase_b;
  sensor->analog_input = analog_input;
  sensor->analog_input2 = analog_input2;
  sensor->channel = channel;
  sensor->channel2 = channel2;
  sensor->knownResistor = knownResistor;
}

void measure (values *results, int phase_a, int phase_b, int analog_input, adc1_channel_t channel, long knownResistor);

void measureSensor(Sensor *sensor) {
  values results[NUM_READS];
  measure(results, sensor->phase_a, sensor->phase_b, sensor->analog_input, sensor->channel, sensor->knownResistor);
  values firstReading = average(results);
  measure(results, sensor->phase_b, sensor->phase_a, sensor->analog_input2, sensor->channel2, sensor->knownResistor);
  values secondReading = average(results);
  sensor->lastMeasure = average(firstReading, secondReading);
  printOutputSensor(*sensor, firstReading.resistance, secondReading.resistance);
  printValues(firstReading);
  printValues(secondReading);
  printValues(sensor->lastMeasure);
}

void printOutputSensor(Sensor sensor, long firstReading, long secondReading) {
  long bias = abs(firstReading - secondReading);
  long readingAvg = (firstReading + secondReading) / 2;
  Serial.print ("Sensor ");
  Serial.print (sensor.id);
  Serial.print ("\t" );
  Serial.print ("read 1\t" );
  Serial.print (firstReading);
  Serial.print ("\t" );
  Serial.print ("read 2\t" );
  Serial.print (secondReading);
  Serial.print ("\t" );
  Serial.print ("bias\t" );
  Serial.print (bias);
  Serial.print ("\t" );
  Serial.print ("value\t");
  Serial.println (readingAvg);
}

void measure (values *results, int phase_a, int phase_b, int analog_input, adc1_channel_t channel, long knownResistor) {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  values curr;
  for (int i = 0; i < NUM_READS; i++) {
    pinMode(phase_a, OUTPUT);
    pinMode(phase_b, INPUT);

    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delay(10);
    curr.supplyVoltageRaw = analogReadByTable(analog_input);   // read the supply voltage
    delayMicroseconds(100);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off
    delay(1);

    pinMode(phase_a, INPUT);
    pinMode(phase_b, OUTPUT);

    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delay(10);
    curr.sensorVoltageRaw = analogReadByAPI(channel);
    delayMicroseconds(100);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off

    // Calculate resistance
    curr.supplyVoltage = rawToVoltageByTable(curr.supplyVoltageRaw);
    curr.sensorVoltage = rawToVoltageByAPI(curr.sensorVoltageRaw);
    //if (curr.sensorVoltage <= 0.17) {//Esse valor foi definido abrindo o circuito (resistor) e aferindo o valor de sensorVoltageFinal
    //  curr.sensorVoltage = 0.1; // Evitar exceção e elevar o valor resistência para próximo do máximo possível aferido.
    //}
    curr.resistance = (knownResistor * (curr.supplyVoltage - curr.sensorVoltage ) / curr.sensorVoltage) ;
    delay(1);
    results[i] = curr;
    //    printValues(curr);
  }
}

void printValues(values value) {
  Serial.print("supplyVoltage: ");
  Serial.print(value.supplyVoltageRaw);
  Serial.print("\t");
  Serial.print(value.supplyVoltage);
  Serial.print("\t");
  Serial.print("sensorVoltage: ");
  Serial.print(value.sensorVoltageRaw);
  Serial.print("\t");
  Serial.print(value.sensorVoltage);
  Serial.print("\t");
  Serial.print("resistance: ");
  Serial.print("\t");
  Serial.println(value.resistance);
}

int analogReadByTable(int analog_input) {
  int voltage = analogRead(analog_input);
  return ADC_LUT[voltage];
}

float rawToVoltageByTable(int value) {
  return value * 3.3 / 4095;
}

int analogReadByAPI(adc1_channel_t channel) {
  return adc1_get_raw(channel);//Obtem o valor RAW do ADC
}

float rawToVoltageByAPI(int value) {
  return esp_adc_cal_raw_to_voltage(value, &adc_cal) / 1000.0;
}

values average(values results[]) {
  values result;
  result.supplyVoltageRaw = 0;
  result.supplyVoltage = 0.0;
  result.sensorVoltageRaw = 0;
  result.sensorVoltage = 0.0;
  result.resistance = 0;
  long sum = 0;
  for (int i = 0; i < NUM_READS; i++) {
    result.supplyVoltageRaw += results[i].supplyVoltageRaw;
    result.supplyVoltage += results[i].supplyVoltage;
    result.sensorVoltageRaw += results[i].sensorVoltageRaw;
    result.sensorVoltage += results[i].sensorVoltage;
    result.resistance += results[i].resistance;
  }
  result.supplyVoltageRaw = result.supplyVoltageRaw / NUM_READS;
  result.supplyVoltage = result.supplyVoltage / NUM_READS;
  result.sensorVoltageRaw = result.sensorVoltageRaw / NUM_READS;
  result.sensorVoltage = result.sensorVoltage / NUM_READS;
  result.resistance = result.resistance / NUM_READS;
  return result;
}

values average(values readingA, values readingB) {
  values result;
  result.supplyVoltageRaw = (readingA.supplyVoltageRaw + readingB.supplyVoltageRaw) / 2;
  result.supplyVoltage = (readingA.supplyVoltage + readingB.supplyVoltage) / 2;
  result.sensorVoltageRaw = (readingA.sensorVoltageRaw + readingB.sensorVoltageRaw) / 2;
  result.sensorVoltage = (readingA.sensorVoltage + readingB.sensorVoltage) / 2;
  result.resistance = (readingA.resistance + readingB.resistance) / 2;
  return result;
}


boolean connected() {
  return WiFi.status() == WL_CONNECTED;
}

bool conectarWifi() {
  bool isWifiConnected = connected();
  if (!isWifiConnected) {
    Serial.println("Reconectando...");
    bool isWifiConnected = connectWifi();
  }
  return isWifiConnected;
}

boolean connectedMQTT(PubSubClient clientMQTT) {
  return clientMQTT.connected();
}

bool conectarMQTT(PubSubClient clientMQTT) {
  bool isMQTTConnected = connectedMQTT(clientMQTT);
  if (!isMQTTConnected) {
    //Serial.println("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    isMQTTConnected = clientMQTT.connect("ESP8266 Device", tokenMqttDisp, NULL);
    if (isMQTTConnected) {
      Serial.println("Subscribing to ThingsBoard node ...");
      clientMQTT.subscribe(attributesTopic);
      clientMQTT.publish(attributesRequestTopic, "{'sharedKeys': 'MEASURE_INTERVAL'}");
      clientMQTT.publish(attributesRequestTopic, "{'sharedKeys': 'TIMES_TO_MEASURE'}");
    }
  }
  return isMQTTConnected;
}

void sendSensorInfoToMQTTServer(Sensor sensor, PubSubClient clientMQTT) {
  bool isConnected = conectarMQTT(clientMQTT);
  if (isConnected) {
    values measure = sensor.lastMeasure;
    String supplyVoltageRawATT = String("supplyVoltageRaw");
    String supplyVoltageATT = String("supplyVoltage");
    String sensorVoltageRawATT = String("sensorVoltageRaw");
    String sensorVoltageATT = String("sensorVoltage");
    String resistanceATT = String("resistance");
    String weightATT = String("weight");

    supplyVoltageRawATT.concat(sensor.id);
    supplyVoltageATT.concat(sensor.id);
    sensorVoltageRawATT.concat(sensor.id);
    sensorVoltageATT.concat(sensor.id);
    resistanceATT.concat(sensor.id);
    weightATT.concat(sensor.id);
    char weightString[20];
    sprintf(weightString, "%.5f", sensor.weight);

    enviarInfoParaServidorMQTT(clientMQTT, supplyVoltageRawATT, String(measure.supplyVoltageRaw));
    enviarInfoParaServidorMQTT(clientMQTT, supplyVoltageATT, String(measure.supplyVoltage));
    enviarInfoParaServidorMQTT(clientMQTT, sensorVoltageRawATT, String(measure.sensorVoltageRaw));
    enviarInfoParaServidorMQTT(clientMQTT, sensorVoltageATT, String(measure.sensorVoltage));
    enviarInfoParaServidorMQTT(clientMQTT, resistanceATT, String(measure.resistance));
    enviarInfoParaServidorMQTT(clientMQTT, weightATT, weightString);
  } else {
    Serial.println("Não foi possivel conectar com o servidor MQTT!");
  }
}

void enviarInfoParaServidorMQTT(PubSubClient clientMQTT, String atributo, String valor) {
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"" + atributo + "\":";
  payload += valor;
  payload += "}";

  // Send payload
  char attributes[120];
  payload.toCharArray( attributes, 120 );
  boolean sent = clientMQTT.publish( "v1/devices/me/telemetry", attributes );
  if (sent) {
    Serial.print("Mensagem enviada com sucesso!");
  } else {
    Serial.println("Mensagem não enviada!");
  }
}

void sendSensorInfoToThingspeak(Sensor sensor, int channelResistor, int channelWeight) {
  int httpCode;
  int trials = 0;
  Serial.println("Esperando 15s entre envios...");
  while (millis() - lastTimeSentToThingspeak < 15000) {
  }
  values measure = sensor.lastMeasure;
  ThingSpeak.setField(channelResistor, measure.resistance);
  ThingSpeak.setField(channelWeight, sensor.weight);
  do {
    httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (httpCode == 200) {
      Serial.println("Channel write successful.");
    } else {
      Serial.println("Problem writing to channel. HTTP error code " + String(httpCode));
    }
    trials++;
  } while (httpCode != 200 && trials < 4);
}

void startWifiClient() {
  Serial.println("Starting wifi...");
  inicializarWiFi();
  client.setServer(servidorMqtt , portaServidorMqtt);
  client.setCallback(on_message);
  clientElton.setServer(servidorMqttElton , portaServidorMqttElton);
  clientElton.setCallback(on_message);
}

bool inicializarWiFi() {
  WiFi.mode(WIFI_STA);
  Serial.println("Starting WIFI...");
  //wifiMulti.addAP(WIFISSID01, PASSWORD01);
  //wifiMulti.addAP(WIFISSID02, PASSWORD02);
  wifiMulti.addAP(WIFISSID03, PASSWORD03);
  Serial.println("Connecting Wifi...");
  return connectWifi();
}

bool connectWifi() {
  bool isConnected = false;
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    isConnected = true;
    imprimirInformacoesWifiEMQTT();
  } else {
    Serial.println("WiFi is not connected!");
  }
  return isConnected;
}

void imprimirInformacoesWifiEMQTT() {
  // Imprimir os valores-padrão das variáveis referentes ao WiFi e servidor MQTT
  Serial.println(tokenMqttDisp);
  Serial.println(servidorMqtt);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wifi status: ");
  Serial.println(WiFi.status());
  Serial.print("Wifi SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("GW: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Mask: ");
  Serial.println(WiFi.subnetMask());
}

void setupScales() {
  Serial.println("Iniciando Setup...");
  scale1.begin(pinDTScale1, pinSCKScale1); // CONFIGURANDO OS PINOS DA BALANÇA
  scale2.begin(pinDTScale2, pinSCKScale2); // CONFIGURANDO OS PINOS DA BALANÇA
  scale1.set_scale(escala1); // ENVIANDO O VALOR DA ESCALA CALIBRADO
  scale2.set_scale(escala2); // ENVIANDO O VALOR DA ESCALA CALIBRADO
  //scale1.tare();
  //Serial.println(scale1.read_average());
  //scale1.set_offset(LOADCELL_OFFSET_1);
  //scale2.tare();
  //Serial.println(scale2.read_average());
  //scale2.set_offset(LOADCELL_OFFSET_2);
  //resetScale();
}

void measureWeight(Sensor *sensor, HX711 scale, int idBalanca) {
  Serial.print("Extraindo o peso da balança ");
  Serial.print(idBalanca);
  Serial.println(" (100 aferições)");
  float medidas[100];
  Statistic stats;
  stats.clear();
  for (int i = 0; i < 100; i++) {
    medidas[i] = scale.get_units(2);
    //Serial.println(medidas[i] , 5);
  }
  sortArray(medidas, 100);
  //printArray("medidas: ", medidas, 100);
  for (int i = 25; i < 75; i++) {
    stats.add(medidas[i]);
  }
  sensor->weight = stats.average();
  Serial.println(sensor->weight , 5);
}

void setTare(HX711 *scale) {
  Serial.print("Setando tara da balança ");
  Serial.println(" (100 aferições)");
  float medidas[100];
  Statistic stats;
  stats.clear();
  for (int i = 0; i < 100; i++) {
    medidas[i] = scale->read_average(1);
  }
  sortArray(medidas, 100);
  //printArray("medidas: ", medidas, 100);
  for (int i = 25; i < 75; i++) {
    stats.add(medidas[i]);
  }
  scale->set_offset(stats.average());
  Serial.println(scale->get_units(2), 5);
}

void printArray(String msg, float* myArray, int size) {
  Serial.println(msg);
  Serial.print("[");
  for (int i = 0; i < size - 1; i++) {
    Serial.print(myArray[i] , 5);
    Serial.print(" , ");
  }
  Serial.print(myArray[size - 1] , 5);
  Serial.println("]");
}

void resetScale() {
  liftUp();
  delay(1000);
  setTare(&scale1);
  setTare(&scale2);
  //scale1.tare();
  //scale2.tare();
  delay(2000);
  liftDown();
  delay(2000);
}

void liftDown() {
  while (angulo <= movimento) {
    meuservo.write(angulo); // Comando para angulo específico
    angulo++;
    delay(passosDelay);
  }
}

void liftUp() {
  while (angulo >= 0) {
    meuservo.write(angulo); // Comando para angulo específico
    angulo -= 1;
    delay(passosDelay);
  }
}

void on_message(const char* topic, byte* payload, unsigned int length) {
  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success()) {
    Serial.println("parseObject() failed");
    return;
  }
  if (strcmp(topic, attributesTopic) == 0) {
    if (data["MEASURE_INTERVAL"]) {
      MEASURE_INTERVAL = data["MEASURE_INTERVAL"];
    }
    if (data["TIMES_TO_MEASURE"]) {
      TIMES_TO_MEASURE = data["TIMES_TO_MEASURE"];
    }
  } else if (strcmp(topic, attributesResponseTopic) == 0) {
    if (data["shared"] && data["shared"]["MEASURE_INTERVAL"]) {
      MEASURE_INTERVAL = data["shared"]["MEASURE_INTERVAL"];
    }
    if (data["shared"] && data["shared"]["TIMES_TO_MEASURE"]) {
      TIMES_TO_MEASURE = data["shared"]["TIMES_TO_MEASURE"];
    }
  } else {
    Serial.println("Tópico não tratado!");
  }
}
