#include <RH_ASK.h>
#include <SPI.h>

// Configuração do driver RH_ASK: (bps, pino RX, pino TX)
RH_ASK rf_driver(1000, 16, 22);

void setup() {
  Serial.begin(115200);
  delay(4000); // tempo para estabilizar o serial
  pinMode(16, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  if (!rf_driver.init()) {
    Serial.println("Falha ao inicializar o módulo RF");
    while (1) delay(10000);
  }

  rf_driver.setThisAddress(0x01); // nosso endereço
  Serial.println("Transmissor RF pronto! Digite sua mensagem:");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // remove espaços em branco

    // Envia mensagem
    rf_driver.setHeaderTo(0xFF);     // broadcast
    rf_driver.setHeaderFrom(0x01);   // remetente
    rf_driver.setHeaderId(0x01);     // ID da mensagem
    rf_driver.setHeaderFlags(0x00);  // sem flags

    rf_driver.send((uint8_t *)input.c_str(), input.length() + 1);
    rf_driver.waitPacketSent();
    Serial.println("Mensagem transmitida: " + input);

    // Agora escuta a resposta
    uint8_t buf[50] = {0};
    uint8_t buflen = sizeof(buf);
    uint8_t id = 0xFF;

    unsigned long startTime = millis();
    const unsigned long timeout = 5000;

    bool respostaRecebida = false;

    while (millis() - startTime < timeout) {
      //rf_driver.send((uint8_t *)input.c_str(), input.length() + 1);
      //rf_driver.waitPacketSent();
      //delay (1000);
      buflen = sizeof(buf);
      if (rf_driver.recv(buf, &buflen)) {
        id = rf_driver.headerFrom();
        //Serial.print(">> Resposta de 0x");
        Serial.println(id, HEX);
        Serial.print(">> Conteúdo: ");
        Serial.println((char *)buf);

        if (id == 0x02 ) {
          respostaRecebida = true;
          break;
        }
      }
      //delay(200); // aguarda antes de tentar novamente
    }
    if (!respostaRecebida) {
      Serial.println("⚠️");
    } else {
      Serial.println("✅");
    }
    //Serial.println("----- Fim da transmissão -----\n");
  }

  delay(500);
}


