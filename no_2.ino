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
const int pwmResolution = 8

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
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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
        delay(250);
        rf_driver.setHeaderFrom(0x02);
        // se a confimação do 3 for recebida, 2 envia seus dados próprios

        String time2 = get_time_stamp();
        float distancia2 = get_distance_cm();  // Lê a distância

        StaticJsonDocument<200> docOut;
        docOut["dist_cm"] = distancia2;
        docOut["timestamp"] = time2;

        char jsonStr[200];
        serializeJson(docOut, jsonStr); 
        
        const char *msg2=(char*)jsonStr;
        rf_driver.send((uint8_t *)msg2, strlen(msg2) + 1);
        rf_driver.waitPacketSent();
        Serial.println("Mensagem transmitida: " + String(msg2));
        delay(1000);
        uint8_t buf2[200] = { 0 };
        uint8_t buflen2 = sizeof(buf2);
        uint8_t id2 = 0x02;


        unsigned long startTime2 = millis();
        const unsigned long timeout2 = 10000;
        bool respostaRecebida2 = false;


        while (millis() - startTime2 < timeout2) {
           rf_driver.send((uint8_t *)msg2, strlen(msg2) + 1);
            rf_driver.waitPacketSent();
            Serial.println("Mensagem transmitida: " + String(msg2));
          buflen2 = sizeof(buf2);
          if (rf_driver.recv(buf2, &buflen2)) {
            id2 = rf_driver.headerFrom();
            Serial.println(id2, HEX);
           

            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, (char *)buf2);
            float dist2 = -999;
            String timestamp2 = "erro";

            if (!error) {
              dist2 = doc["dist_cm"] | -999;
              timestamp2 = doc["timestamp"] | "erro";
            }

            if (id2 == 0x03) {
              Serial.printf(">> Conteúdo: %s dist: %.2f cm\n", timestamp.c_str(), dist2);
              Serial.print(time2);
              Serial.print(timestamp2);
              respostaRecebida2 = true;
              break;
            }
          }
        }


        if (!respostaRecebida2) {
          Serial.println("❌");
        } else {
          Serial.println("✔");
        }
      }
      }
    }
  }

  delay(500);
}  
