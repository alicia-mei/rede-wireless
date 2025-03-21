#include <RH_ASK.h>  // Inclui a biblioteca RadioHead para Amplitude Shift Keying
#include <SPI.h>     // Inclui a biblioteca SPI necessária para a comunicação

// Configuração do driver de RF para ESP32 com 433MHz (RX: GPIO4, TX: GPIO22)
RH_ASK rf_driver(2000, 4, 22);  

void setup() {
  Serial.begin(115200);  // Inicializa o monitor serial a 115200 baud rate
  delay(4000);  // Atraso inicial para garantir a estabilização do sistema
  Serial.println("ESP32 433MHz Transmitter and Receiver");

  // Inicialização do transmissor e receptor
  if (!rf_driver.init()) {
    Serial.println("init failed");
    while (1) delay(10000);  // Se falhar na inicialização, entra em loop
  }
  Serial.println("rf_driver initialised");
}

void loop() {
  // **Transmissão** - Envia mensagem a cada 5 segundos
  Serial.println("Transmitting packet");
  const char *msg = "A3 oi leo";  // Mensagem a ser transmitida
  rf_driver.send((uint8_t *)msg, strlen(msg) + 1);  // Envia a mensagem
  rf_driver.waitPacketSent();  // Espera até que a transmissão seja concluída
  Serial.println("Message Transmitted");

  // Adicionar um pequeno atraso entre a transmissão e recepção para garantir que o módulo tenha tempo
  delay(100);  // Atraso para dar tempo ao receptor processar a mensagem

  // **Recepção** - Verifica se há pacotes recebidos e imprime a mensagem recebida
  uint8_t buf[20] = {0};  // Buffer para armazenar a mensagem recebida
  uint8_t buflen = sizeof(buf);  // Tamanho do buffer

  // Se uma mensagem for recebida com sucesso, imprime a mensagem no monitor serial
  if (rf_driver.recv(buf, &buflen)) {
    Serial.print("Message Received: ");
    // Verifica se a mensagem é para "A2"
    if (buf[0] == 'A' && buf[1] == '2') {
      Serial.println((char*)buf);  // Exibe a mensagem recebida no monitor serial
    } else {
      Serial.println("Não é para você!");
    }
  } else {
    Serial.println("No message received");  // Caso nenhum pacote tenha sido recebido
  }

  delay(2000);  // Atraso adicional de 2 segundos antes de tentar novamente
}
