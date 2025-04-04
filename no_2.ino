#include <RH_ASK.h>  // Inclui a biblioteca RadioHead para Amplitude Shift Keying
#include <SPI.h>     // Inclui a biblioteca SPI necessária para a comunicação


// Configuração do driver de RF para ESP32 com 433MHz (RX: GPIO4, TX: GPIO22)
RH_ASK rf_driver(1000, 4, 22);  


void setup() {
  Serial.begin(115200);  // Inicializa o monitor serial a 115200 baud rate
  delay(4000);  // Atraso inicial para garantir a estabilização do sistema
  //Serial.println("ESP32 433MHz Transmitter and Receiver");


  // Configura os pinos RX e TX explicitamente para evitar falha de sincronização
  pinMode(4, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);  // Define o pino TX como LOW inicialmente para evitar interferência


  // Inicialização do transmissor e receptor
  if (!rf_driver.init()) {
    //Serial.println("init failed");
    while (1) delay(10000);  // Se falhar na inicialização, entra em loop
  }
  //Serial.println("rf_driver initialised");

  //rf_driver.setThisAddress(0x02);
  // Configura o cabeçalho para transmissão
  rf_driver.setHeaderTo(0xFF);     // Destinatário
  rf_driver.setHeaderFrom(0x02);   // Remetente
  rf_driver.setHeaderId(0x02);     // ID da mensagem
  rf_driver.setHeaderFlags(0x00);  // Sem flags específicas
}


void loop() {
  // **Recepção** - Verifica se há pacotes recebidos e imprime a mensagem recebida
  uint8_t buf[20] = {0};  // Buffer para armazenar a mensagem recebida
  uint8_t buflen = sizeof(buf);  // Tamanho do buffer
  // Pequeno atraso para garantir que o receptor esteja pronto
  delayMicroseconds(100);

  if (rf_driver.recv(buf, &buflen)) {
    // Verifica o cabeçalho para garantir que é do endereço correto
    uint8_t id = rf_driver.headerFrom();
    if (id == 0x01) {
      Serial.println(rf_driver.headerFrom(), HEX);
      Serial.println((char*)buf);
       // **Transmissão**
      const char *msg = (char*)buf;  // Mensagem a ser transmitida
      while(id != 0x03){
        //Serial.println("oi");
        rf_driver.send((uint8_t *)msg, strlen(msg) + 1);  // Envia a mensagem
        rf_driver.waitPacketSent();  // Espera até que a transmissão seja concluída
        if (rf_driver.recv(buf, &buflen)) {
          Serial.print("id: ");
          id = rf_driver.headerFrom();
          Serial.println(id);
        }
        delay(500);
      }
      Serial.println("recebi");
    }
  } 
  //else { }
  delay(250);
}
