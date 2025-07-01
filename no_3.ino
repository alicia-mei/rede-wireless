#include <RH_ASK.h>  // Biblioteca RadioHead para ASK
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#define TRIG_PIN 5   // Pino TRIG do sensor ultrassonico
#define ECHO_PIN 18  // Pino ECHO do sensor ultrassonico

// PWM controlado por interrupção
const int inputPin = 16;
const int pwmPin = 17;  // PWM no pino 17
const int pwmChannel = 0;
const int pwmFreq = 125000;
const int pwmResolution = 8;

struct tm data;

RH_ASK rf_driver(1000, 4, 22);  // bit rate, RX, TX

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

  pinMode(4, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!rf_driver.init()) {
    while (1) delay(10000);  // Loop infinito se falhar
  }

  rf_driver.setHeaderTo(0xFF);
  rf_driver.setHeaderFrom(0x03);
  rf_driver.setHeaderId(0x03);
  rf_driver.setHeaderFlags(0x00);

  ledcWrite(0, 127);  // 50% duty cycle

  timeval tv;
  tv.tv_sec = 1746791026;  // Timestamp fixo
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
  // **Recepção** - Verifica se há pacotes recebidos e imprime a mensagem recebida
  uint8_t buf[100] = {0};  // Buffer para armazenar a mensagem recebida
  uint8_t buflen = sizeof(buf);  // Tamanho do buffer
  // Pequeno atraso para garantir que o receptor esteja pronto
  delayMicroseconds(100);

  if (rf_driver.recv(buf, &buflen)) {
    uint8_t id = rf_driver.headerFrom();
    // Verifica o cabeçalho para garantir que é do endereço correto
    if (id == 0x02) {
      Serial.println((char*)buf);

      const char *msg = (char*)buf;  // Mensagem a ser transmitida
      unsigned long startTime = millis();
      const unsigned long timeout = 5000;

      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, (char *)buf);
      float dist = -999;
      String timestamp = "erro";

      if (!error) {
        dist = doc["dist_cm"];
        Serial.println(dist);
        timestamp = doc["timestamp"] | "erro";
      }
      
      Serial.printf(">> Conteúdo: dist: %.2f, timestamp: %s\n", dist, timestamp.c_str());
      rf_driver.send((uint8_t *)msg, strlen(msg) + 1);  // Envia a mensagem
      rf_driver.waitPacketSent();  // Espera até que a transmissão seja concluída
      delay(100);
      esp_sleep_enable_timer_wakeup(1 * 1000000);  // 1 segundo em microssegundos
      esp_light_sleep_start();

    } else {
      Serial.println(rf_driver.headerFrom(), HEX); 
    }
  }
  
}
