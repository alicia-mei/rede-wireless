#include <RH_ASK.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>


#define TRIG_PIN 5   // Pino TRIG do sensor ultrassonico
#define ECHO_PIN 18  // Pino ECHO do sensor ultrassonico


struct tm data;  //Cria a estrutura que contem as informacoes da data.


// Definir o pino do ESP32 para gerar PWM
const int pwmPin = 22;  // É possível escolher outro pino, se necessário


// Configuração do driver RH_ASK: (bps, pino RX, pino TX)
RH_ASK rf_driver(1000, 4, 22);


void setup() {
  Serial.begin(115200);
  // Configurar o canal de PWM
  ledcSetup(0, 125000, 8);
  ledcAttachPin(pwmPin, 0);
  Serial.println("Gerando sinal PWM de 125 kHz...");
  ledcWrite(0, 127);


  delay(4000);
  pinMode(16, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);


  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  if (!rf_driver.init()) {
    Serial.println("Falha ao inicializar o módulo RF");
    while (1) delay(10000);
  }


  rf_driver.setThisAddress(0x01);
  //Serial.println("Transmissor RF pronto! Digite sua mensagem:");


  timeval tv;
  tv.tv_sec = 1746791026;
  settimeofday(&tv, NULL);
}


String get_time_stamp() {
  time_t tt = time(NULL);
  data = *gmtime(&tt);
  char data_formatada[64];
  strftime(data_formatada, 64, "%y-%m-%dT%H:%M", &data);
  return String(data_formatada);
}


float get_distance_cm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2000);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(4000);
  digitalWrite(TRIG_PIN, LOW);


  long duracao = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout de 30ms
  if (duracao == 0) return -1;


  float distancia = duracao * 0.0343 / 2;
  return distancia;
}


void loop() {
 
    String time = get_time_stamp();
   
   
    float distancia = get_distance_cm();  // Lê a distância
    //Serial.print("Distância medida: ");
    //Serial.print(distancia);
    //Serial.println(" cm");  // Exibe no monitor serial
   
    StaticJsonDocument<200> docOut;
    docOut["dist_cm"] = get_distance_cm();
    docOut["timestamp"] = time;


    char jsonStr[200];
    serializeJson(docOut, jsonStr);


    rf_driver.setHeaderTo(0xFF);
    rf_driver.setHeaderFrom(0x01);
    rf_driver.setHeaderId(0x01);
    rf_driver.setHeaderFlags(0x00);
   
    rf_driver.send((uint8_t *)jsonStr, strlen(jsonStr));
    rf_driver.waitPacketSent();
    Serial.println("Mensagem transmitida: " + String(jsonStr));
    delay(2000);
    uint8_t buf[200] = { 0 };
    uint8_t buflen = sizeof(buf);
    uint8_t id = 0xFF;


    unsigned long startTime = millis();
    const unsigned long timeout = 5000;
    bool respostaRecebida = false;


    while (millis() - startTime < timeout) {
      buflen = sizeof(buf);
      if (rf_driver.recv(buf, &buflen)) {
        id = rf_driver.headerFrom();
        Serial.println(id, HEX);


        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, (char *)buf);
        float distancia = -999;
        String timestamp = "erro";
        if (!error) {
          distancia = doc["dist_cm"] | -999;
          timestamp = doc["timestamp"] | "erro";
        }
        Serial.printf(">> Conteúdo: %s dist: %.2f cm\n", timestamp.c_str(), distancia);
        Serial.print(time);
        Serial.print(timestamp);
        if (id == 0x02 ) {
          respostaRecebida = true;
          break;
        }
      }
    }


    if (!respostaRecebida) {
      Serial.println("⚠️");
    } else {
      Serial.println("✅");
    }
 


  delay(500);
}



