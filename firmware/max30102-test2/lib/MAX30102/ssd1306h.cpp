#include <Wire.h>
#include "MAX30102.h"
#include "Pulse.h"

// --- Configurações do Hardware para ESP32 ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- Objetos das suas bibliotecas customizadas ---
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm; // Esta classe está definida dentro de Pulse.h

// --- Constantes e Variáveis Globais ---
// A tabela de SpO2 ainda é necessária para o cálculo
const uint8_t spo2_table[184] =
{ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
  99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
  97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
  90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
  80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
  66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
  49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
  28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
  3, 2, 1
};

int beatAvg;
int SPO2;
uint8_t sleep_counter = 0;
bool fingerOnSensor = false;

// Função go_sleep adaptada para o ESP32
void go_sleep() {
  Serial.println("Nenhuma atividade. Entrando em modo de baixo consumo (Deep Sleep)...");
  sensor.off(); // Assumindo que sua lib tem essa função para desligar o sensor
  delay(100); // Pequena pausa para garantir que a mensagem serial seja enviada
  esp_deep_sleep_start();
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("\nOxímetro de Pulso - Inicializando...");

  // Inicializa I2C com os pinos customizados para o ESP32
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Inicializando o sensor MAX30102...");
  if (!sensor.begin()) {
    Serial.println("ERRO: Sensor MAX30102 não encontrado!");
    while (1); // Trava a execução se o sensor falhar
  }
  sensor.setup();

  Serial.println("Setup concluído. Por favor, posicione o dedo no sensor.");
}

long lastBeat = 0;

void loop() {
  sensor.check();
  long now = millis();

  // Verifica se há dados disponíveis no sensor
  if (!sensor.available()) {
    return;
  }

  uint32_t irValue = sensor.getIR();
  sensor.nextSample(); // Lê a próxima amostra para manter o buffer do sensor limpo

  // Se o valor de IR é muito baixo, significa que não há dedo no sensor.
  if (irValue < 50000) {
    if (fingerOnSensor) {
      Serial.println("Dedo removido. Aguardando...");
      fingerOnSensor = false;
      beatAvg = 0; // Reseta a média de batimentos
    }

    // Lógica para entrar em modo de baixo consumo após inatividade
    sleep_counter++;
    if (sleep_counter > 100) {
      go_sleep();
    }
    delay(100); // Evita spam no terminal
  } else {
    // Se o dedo foi acabado de colocar, imprime uma mensagem
    if (!fingerOnSensor) {
      Serial.println("Dedo detectado. Realizando medição...");
      fingerOnSensor = true;
    }

    sleep_counter = 0; // Reseta o contador de inatividade

    // Processamento do sinal para encontrar o batimento
    int16_t IR_signal = pulseIR.ma_filter(pulseIR.dc_filter(irValue));
    bool beatIR = pulseIR.isBeat(IR_signal);

    if (beatIR) {
      uint32_t redValue = sensor.getRed(); // Pega o valor do Red apenas quando há batimento

      // Calcula o BPM
      long btpm = 60000 / (now - lastBeat);
      if (btpm > 40 && btpm < 200) { // Filtra valores irreais
        beatAvg = bpm.filter((int16_t)btpm);
      }
      lastBeat = now;

      // Calcula o SpO2
      long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
      long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
      int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;

      if ((RX100 >= 0) && (RX100 < 184)) {
        SPO2 = spo2_table[RX100];
      }

      // Imprime os valores no Monitor Serial
      if (beatAvg > 0 && SPO2 > 0) {
        Serial.print("BPM: ");
        Serial.print(beatAvg);
        Serial.print(" | SpO2: ");
        Serial.print(SPO2);
        Serial.println("%");
      }
    }
  }
}