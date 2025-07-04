
#include <RH_ASK.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
// Pinos do sensor ultrassônico
#define TRIG_PIN 5
#define ECHO_PIN 18

// PWM controlado por interrupção
const int inputPin = 16;
const int pwmPin = 17;  // PWM no pino 17
const int pwmChannel = 0;
const int pwmFreq = 125000;
const int pwmResolution = 8;

//LED
const int PINO_LED = 2; // PINO D15

// Driver RF: (bps, RX, TX)
RH_ASK rf_driver(1000, 4, 22);

// Estrutura de data
struct tm data;


void TaskPWMControl(void *pvParameters) {
  (void)pvParameters;
  pinMode(inputPin, INPUT);  // Garante que está como entrada

  while (true) {
    bool inputHigh = digitalRead(inputPin);

    if (inputHigh) {
      ledcWrite(pwmChannel, 127);  // 50% duty (ativa PWM)
    } else {
      ledcWrite(pwmChannel, 0);  // Desliga PWM
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // Verifica a cada 1 ms
  }
}


void setup() {
  Serial.begin(115200);

  // Configura pino 16 como entrada com interrupção
  pinMode(inputPin, INPUT);

  // PWM no pino 17
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  ledcWrite(pwmChannel, 0);  // Começa desligado

  // Pino 22 como saída digital normal
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  // Sensor ultrassônico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //LED
  pinMode(PINO_LED, OUTPUT); // Define o PINO do LED como saída

  // Inicializa RF
  if (!rf_driver.init()) {
    Serial.println("Falha ao inicializar o módulo RF");
    while (1) delay(10000);
  }
  rf_driver.setThisAddress(0x01);

  // Ajusta horário inicial
  timeval tv;
  tv.tv_sec = 1746791026;
  settimeofday(&tv, NULL);

  xTaskCreatePinnedToCore(
  TaskPWMControl,  // Função da task
  "PWM Control",   // Nome
  2048,            // Stack size
  NULL,            // Param
  1,               // Prioridade
  NULL,            // Handle (opcional)
  0                // Core (0 ou 1)
); 

digitalWrite(PINO_LED, LOW);
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

  long duracao = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracao == 0) return -1;
  return duracao * 0.0343 / 2;
}

void loop() {
  String time = get_time_stamp();
  float distancia = get_distance_cm();

  // Cria JSON com distância e timestamp
  StaticJsonDocument<200> docOut;
  docOut["dist_cm"] = distancia;
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

  // Aguarda resposta
  uint8_t buf[200] = { 0 };
  uint8_t buflen = sizeof(buf);
  uint8_t id = 0xFF;
  bool respostaRecebida = false;
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {
    buflen = sizeof(buf);
    if (rf_driver.recv(buf, &buflen)) {
      id = rf_driver.headerFrom();
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, (char *)buf);
      if (!error) {
        float distRx = doc["dist_cm"] | -999;
        String timestampRx = doc["timestamp"] | "erro";
        Serial.printf(">> Conteúdo: %s dist: %.2f cm\n", timestampRx.c_str(), distRx);
        if (id == 0x02) {
          respostaRecebida = true;
          break;
          
        }
      }
    }
  }
  if(respostaRecebida) Serial.println("✅");
  else{
    for(int i = 0; i <4; i++){
    digitalWrite(PINO_LED, HIGH); // Liga o LED
    delay(500); // Espera 1 segundo
    digitalWrite(PINO_LED, LOW); // Desliga o LED
    delay(500);
    }
  }
  Serial.println(respostaRecebida ? "✅" : "⚠️");
  esp_sleep_enable_timer_wakeup(30 * 1000000);  // 30 segundos em microssegundos
  esp_light_sleep_start();

}
