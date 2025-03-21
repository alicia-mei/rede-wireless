#include <RH_ASK.h>  // Inclui a biblioteca RadioHead para Amplitude Shift Keying
#include <SPI.h>     // Inclui a biblioteca SPI necessária para a comunicação


// Configuração do driver de RF para ESP32 com 433MHz (RX: GPIO4, TX: GPIO22)
RH_ASK rf_driver(2000, 4, 22);  


void setup() {
  Serial.begin(115200);  // Inicializa o monitor serial a 115200 baud rate
  delay(4000);  // Atraso inicial para garantir a estabilização do sistema
  Serial.println("ESP32 433MHz Transmitter and Receiver");


  // Configura os pinos RX e TX explicitamente para evitar falha de sincronização
  pinMode(4, INPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);  // Define o pino TX como LOW inicialmente para evitar interferência


  // Inicialização do transmissor e receptor
  if (!rf_driver.init()) {
    Serial.println("init failed");
    while (1) delay(10000);  // Se falhar na inicialização, entra em loop
  }
  Serial.println("rf_driver initialised");


  // Configura o cabeçalho para transmissão
  rf_driver.setHeaderTo(0xFF);     // Destinatário
  rf_driver.setHeaderFrom(0x01);   // Remetente
  rf_driver.setHeaderId(0x01);     // ID da mensagem
  rf_driver.setHeaderFlags(0x00);  // Sem flags específicas
}


void loop() {
  // **Transmissão** - Envia mensagem a cada 5 segundos
  
  Serial.println("Transmitting packet");
  const char *msg = "oi debora";  // Mensagem a ser transmitida
  rf_driver.send((uint8_t *)msg, strlen(msg) + 1);  // Envia a mensagem
  rf_driver.waitPacketSent();  // Espera até que a transmissão seja concluída
  Serial.println("Message Transmitted");


  // **Recepção** - Verifica se há pacotes recebidos e imprime a mensagem recebida
  uint8_t buf[20] = {0};  // Buffer para armazenar a mensagem recebida
  uint8_t buflen = sizeof(buf);  // Tamanho do buffer


  // Pequeno atraso para garantir que o receptor esteja pronto
  delayMicroseconds(100);


  if (rf_driver.recv(buf, &buflen)) {
    // Verifica o cabeçalho para garantir que é do endereço correto
    if (rf_driver.headerFrom() == 0x02) {
      Serial.print("Message Received: ");
      Serial.println((char*)buf);
      Serial.print("From: ");
      Serial.println(rf_driver.headerFrom(), HEX);
      Serial.print("To: ");
      Serial.println(rf_driver.headerTo(), HEX);
    } else {
      Serial.print("Ignored message from wrong address: ");
      Serial.println(rf_driver.headerFrom(), HEX);
    }
  } else {
    Serial.println("No message received");  // Caso nenhum pacote tenha sido recebido
  }


  delay(2000);  // Atraso adicional de 2 segundos antes de tentar novamente
}
