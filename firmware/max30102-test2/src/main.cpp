#include <Arduino.h> // Adicionado para definir __FlashStringHelper

// --- Classe Falsa (Dummy Class) para resolver erros de linkagem ---
// Isso satisfaz as chamadas para o SSD1306 que ainda existem na biblioteca Pulse.h
class SSD1306 {
public:
    SSD1306() {}
    void drawChar(int, int, unsigned char, int) {}
    void fill(unsigned char) {}
    void off() {}
    void firstPage() {}
    bool nextPage() { return false; }
    void drawStr(int, int, const __FlashStringHelper*, int) {}
    void init() {}
};

#include <Wire.h>
#include "MAX30102.h"
#include "Pulse.h"

// --- Configurações do Hardware para ESP32 ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- NOVOS LIMITES COM HISTERESE para estabilizar a detecção ---
#define FINGER_ON_THRESHOLD  75000 // Valor para TER CERTEZA que o dedo está no sensor
#define FINGER_OFF_THRESHOLD 50000 // Valor para TER CERTEZA que o dedo foi removido

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
bool fingerOnSensor = false;
// Variáveis de debug para análise
int lastRatio = 0;
long lastRedAC = 0;
long lastIrAC = 0;


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
  
  // Ajusta o sensor para uma melhor leitura. Se sua função 'setup' for diferente,
  // você pode precisar adaptar esta linha. Por exemplo, ajustando a corrente do LED
  // para obter um sinal mais forte e estável, o que pode melhorar a detecção.
  sensor.setup(); 

  Serial.println("Setup concluído. Por favor, posicione o dedo no sensor.");
}

long lastBeat = 0;
long lastPrintTime = 0; // Para controlar a frequência de impressão no terminal

void loop() {
  sensor.check();
  long now = millis();

  if (!sensor.available()) {
    return;
  }

  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed(); // Ler ambos os valores para os filtros
  sensor.nextSample();

  // Lógica de detecção com histerese para evitar oscilação
  if (irValue < FINGER_OFF_THRESHOLD && fingerOnSensor) {
    fingerOnSensor = false;
    beatAvg = 0;
    SPO2 = 0;
    Serial.println("\nDedo removido. Aguardando...");
  } else if (irValue > FINGER_ON_THRESHOLD && !fingerOnSensor) {
    fingerOnSensor = true;
    lastBeat = millis(); // Reseta o timer da batida quando o dedo é detectado
    Serial.println("Dedo detectado. Realizando medição...");
  }

  // Se o dedo estiver no sensor, processa os dados
  if (fingerOnSensor) {
    // Processamento do sinal para encontrar o batimento
    int16_t IR_signal = pulseIR.ma_filter(pulseIR.dc_filter(irValue));
    bool beatIR = pulseIR.isBeat(IR_signal);

    // *** CORREÇÃO PARA SpO2 ***
    // Processa o sinal vermelho da mesma forma, chamando isBeat() para forçar
    // o cálculo interno do valor AC (avgAC) na biblioteca.
    int16_t Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
    pulseRed.isBeat(Red_signal); // A chamada é necessária, mesmo sem usar o resultado.

    if (beatIR) {
      long beatInterval = now - lastBeat;
      
      if (beatInterval > 600) { 
          lastBeat = now;
          
          long btpm = 60000 / beatInterval;
          if (btpm > 40 && btpm < 200) {
            beatAvg = bpm.filter((int16_t)btpm);
          }
          
          // Calcula o SpO2 somente quando temos um batimento válido
          long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
          long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
          int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;

          if ((RX100 >= 0) && (RX100 < 184)) {
            SPO2 = spo2_table[RX100];
          }
      }
    }

    // Imprime os valores no Monitor Serial a cada 1 segundo para não poluir o terminal
    if (now - lastPrintTime > 1000) {
      lastPrintTime = now;
      if (beatAvg > 0 && SPO2 > 0) {
        Serial.print("BPM: ");
        Serial.print(beatAvg);
        Serial.print(" | SpO2: ");
        Serial.print(SPO2);
        Serial.println("%");
        
        /* --- Linhas de Debug (Comentadas) ---
        Serial.print(" | Ratio: ");
        Serial.print(lastRatio);
        Serial.print(" | Red_AC: ");
        Serial.print(lastRedAC);
        Serial.print(" | IR_AC: ");
        Serial.println(lastIrAC);
        */

      } else {
        Serial.println("Calculando..."); // Feedback para o usuário
      }
    }
  } else {
    // Quando não há dedo, apenas aguarda. A lógica de sleep foi removida.
    delay(100);
  }
}

