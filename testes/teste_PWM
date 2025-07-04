const int txPin = 22;  // Pino conectado ao transmissor 433 MHz
const int pwmChannel = 0;
const int pwmFrequency = 125000;
const int pwmResolution = 8;


void setup() {
  Serial.begin(115200);


  // Configura canal PWM mas não o ativa ainda
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(txPin, pwmChannel);
  ledcWrite(pwmChannel, 0); // Inicia com duty cycle 0 (LOW)
}


// Envia um único bit: 1 = onda 125kHz, 0 = nível baixo
void sendBit(bool bit) {
  if (bit) {
    ledcWrite(pwmChannel, 127);  // 50% duty cycle
  } else {
    ledcWrite(pwmChannel, 0);    // Sem sinal
  }
  delayMicroseconds(1000);  // Duração de 1 bit: 1000 µs (1 ms) → 1 kbps
}


void sendByte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    bool bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
}


uint8_t data = 0b10101010; // Exemplo de dado: alternância de 1s e 0s


void loop() {
  sendByte(data);
}
