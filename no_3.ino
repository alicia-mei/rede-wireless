#include <RH_ASK.h>  // Biblioteca RadioHead para ASK
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>

struct tm data;

const int pwmPin = 22;  // Pino para sinal PWM
RH_ASK rf_driver(1000, 4, 22);  // bit rate, RX, TX

void setup() {
  Serial.begin(115200);

  ledcSetup(0, 125000, 8);      // Canal 0, 125 kHz, 8 bits
  ledcAttachPin(pwmPin, 0);     // Pino 22 no canal 0
  Serial.println("Gerando sinal PWM de 125 kHz...");
  delay(4000);

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
  uint8_t buf[20] = {0};  // Buffer para armazenar a mensagem recebida
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

    } else {
      Serial.println(rf_driver.headerFrom(), HEX); 
    }
  }
  //else { }
  delay(250);
}
