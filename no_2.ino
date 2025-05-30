#include <RH_ASK.h>  // Biblioteca RadioHead para ASK
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>

struct tm data;

// PWM controlado por interrupção
const int inputPin = 16;
const int pwmPin = 17;  // PWM no pino 17
const int pwmChannel = 0;
const int pwmFreq = 125000;
const int pwmResolution = 8;

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

  if (!rf_driver.init()) {
    while (1) delay(10000);  // Loop infinito se falhar
  }

  rf_driver.setHeaderTo(0xFF);
  rf_driver.setHeaderFrom(0x02);
  rf_driver.setHeaderId(0x02);
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

void loop() {
  uint8_t buf[50] = { 0 };
  uint8_t buflen = sizeof(buf);
  delayMicroseconds(100);

  if (rf_driver.recv(buf, &buflen)) {
    uint8_t id = rf_driver.headerFrom();
    if (id == 0x01) {
      Serial.printf("Recebido de ID %02X: %s\n", id, (char*)buf);

      const char *msg = (char *)buf;
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

      String originalTime = timestamp;
      bool respostaRecebida = false;

      while (millis() - startTime < timeout) {
        rf_driver.send((uint8_t *)msg, strlen(msg) + 1);
        rf_driver.waitPacketSent();
        Serial.println(msg);
        rf_driver.setModeRx();
        delay(1000);

        if (rf_driver.recv(buf, &buflen)) {
          id = rf_driver.headerFrom();
          StaticJsonDocument<200> docResp;
          DeserializationError errorResp = deserializeJson(docResp, (char *)buf);
          float distResp = -999;
          String timestampResp = "erro";

          if (!errorResp) {
            distResp = docResp["dist_cm"] | -999;
            timestampResp = docResp["timestamp"] | "erro";
          }

          Serial.printf(">> Conteúdo: dist: %.2f, timestamp: %s\n", distResp, timestampResp.c_str());
          if (id == 0x03) {
            respostaRecebida = true;
            break;
          }
        }
      }

      if (!respostaRecebida) {
        Serial.println("⚠️ Sem resposta correspondente");
      } else {
        Serial.println("✅ Resposta confirmada");
      }
    }
  }

  delay(500);
}
