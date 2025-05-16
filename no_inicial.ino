#include <RH_ASK.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

struct tm data;//Cria a estrutura que contem as informacoes da data.

// Definir o pino do ESP32 para gerar PWM
const int pwmPin = 22;  // É possível escolher outro pino, se necessário

// Configuração do driver RH_ASK: (bps, pino RX, pino TX)
RH_ASK rf_driver(1000, 16, 22);

void setup() {
  Serial.begin(115200);
   // Configurar o canal de PWM
  // ledcSetup canal, frequência, resolução
  ledcSetup(0, 125000, 8);  // Canal 0, frequência de 125 kHz, resolução de 8 bits (0-255)
  ledcAttachPin(pwmPin, 0); // Atribui o pino do pwm ao canal 0

  Serial.println("Gerando sinal PWM de 125 kHz...");
  // Gerar um sinal PWM com ciclo de trabalho de 50% (valor 127 de 0 a 255)
  ledcWrite(0, 127);  // Valor de 127 representa 50% de ciclo de trabalho (duty cycle)

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

  timeval tv;//Cria a estrutura temporaria para funcao abaixo.
  tv.tv_sec = 	1746791026;//Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
  settimeofday(&tv, NULL);//Configura o RTC para manter a data atribuida atualizada.
}

String get_time_stamp(){
  time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
  data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
  
  char data_formatada[64];
  strftime(data_formatada, 64, "%d/%m %H:%M", &data);//Cria uma String formatada da estrutura "data"

  return String(data_formatada);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // remove espaços em branco

    String timestamp = get_time_stamp();
    String mensagem = timestamp + " " + input;

    // Envia mensagem
    rf_driver.setHeaderTo(0xFF);     // broadcast
    rf_driver.setHeaderFrom(0x01);   // remetente
    rf_driver.setHeaderId(0x01);     // ID da mensagem
    rf_driver.setHeaderFlags(0x00);  // sem flags

    rf_driver.send((uint8_t *)mensagem.c_str(), mensagem.length() + 1);
    rf_driver.waitPacketSent();
    Serial.println("Mensagem transmitida: " + mensagem);

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
