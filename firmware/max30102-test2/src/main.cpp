#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_MLX90614.h>
#include <vector>
#include <algorithm>

// Definição dos pinos
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define RELE_PIN 14

#define DEBUGMODE 1
#define SIMULATE_SENSORS 0 // Mude para 0 para usar os sensores reais
#define SerialPi Serial2 
#define RX2_PIN 16
#define TX2_PIN 17

// Definição dos estados
enum State {
    IDLE,
    MEASURE_TEMP,
    MEASURE_OXI,
    SERVO1_FORWARD,
    SERVO1_BACKWARD,
    SERVO2_FORWARD,
    SERVO2_BACKWARD,
    RELE_CONTROL
};

// Objetos globais
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Servo servo1;
Servo servo2;

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

// Timeout para o dedo ser inserido no sensor durante a medição de oxigenação (ms)
#define OXI_FINGER_TIMEOUT 5000 // 5 segundos

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
int lastRatio = 0;
long lastRedAC = 0;
long lastIrAC = 0;

// Variáveis globais de controle
State currentState = IDLE;
std::vector<float> tempSamples;
std::vector<int> oxiSamples;
unsigned long lastSampleTime = 0;
unsigned long releStartTime = 0;
unsigned long lastBeat = 0;
unsigned long lastPrintTime = 0;
unsigned long oxiStartTime = 0; // armazena quando entramos em MEASURE_OXI
int sampleCount = 0;
uint8_t try_count = 0;

void setup(void) {
    // Serial UART0 - USB (Debug)
    Serial.begin(115200); 
    
    // SerialPi UART2 - Raspberry Pi
    SerialPi.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

    #if DEBUGMODE
        Serial.println("Iniciando sistema... (Debug USB)");
    #endif
    
    #if !SIMULATE_SENSORS
        // Inicializa I2C com os pinos customizados para o ESP32
        Wire.begin(I2C_SDA, I2C_SCL); // Descomente se não estiver em outro lugar

        // Inicializa sensor MAX30102
        if (!sensor.begin()) {
            Serial.println("ERRO: Sensor MAX30102 não encontrado!");
        }
        sensor.setup();

        // Inicializa MLX90614
        if (!mlx.begin()) {
            Serial.println("Erro: MLX90614 não encontrado!");
        }
    #else
        #if DEBUGMODE
            Serial.println("******************************************");
            Serial.println("ATENCAO: SENSORES EM MODO DE SIMULACAO!");
            Serial.println("******************************************");
        #endif
    #endif


    // Inicializa Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo1.setPeriodHertz(100);
    servo2.setPeriodHertz(100);
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo1.write(120);
    servo2.write(0);

    // Inicializa Relé
    pinMode(RELE_PIN, OUTPUT);
    digitalWrite(RELE_PIN, LOW);

    #if DEBUGMODE
        Serial.println("Sistema Inicializado.");
    #endif
}

float getMedian(std::vector<float>& samples) {
    if (samples.empty()) return 0;
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

int getMedianInt(std::vector<int>& samples) {
    if (samples.empty()) return 0;
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

void processSerialCommands() {
    if (SerialPi.available()) {
        String command = SerialPi.readStringUntil('\n');
        
        #if DEBUGMODE
            Serial.print("Comando recebido do Pi: ");
            Serial.println(command);
        #endif

        if (command == "T") {
            currentState = MEASURE_TEMP;
            tempSamples.clear();
            sampleCount = 0;
            lastSampleTime = millis();
        }
        else if (command == "O") {
            currentState = SERVO1_FORWARD;
        }
        else if (command == "P1") {
            currentState = SERVO2_FORWARD;
        }
        else if (command == "P2") {
            currentState = SERVO2_BACKWARD;
        }
        else if (command == "P") {
            currentState = RELE_CONTROL;
            releStartTime = millis();
            digitalWrite(RELE_PIN, HIGH);
        }
    }
}

void loop() {
    processSerialCommands();
    unsigned long currentTime = millis();

    switch (currentState) 
    {
        case IDLE:
            // Nada a fazer, aguardando comandos
            if (currentTime - lastPrintTime > 5000) {
                lastPrintTime = currentTime;
                # if DEBUGMODE
                    Serial.println("Waiting for commands...");
                # endif
            }
            break;

        case MEASURE_TEMP:
            if (currentTime - lastSampleTime >= 100) {
                
                #if SIMULATE_SENSORS
                    float temp = 36.5; // Valor de simulação
                #else
                    float temp = mlx.readObjectTempC();
                #endif

                tempSamples.push_back(temp);
                sampleCount++;
                lastSampleTime = currentTime;

                if (sampleCount >= 5) {
                    float medianTemp = getMedian(tempSamples);
                    SerialPi.printf("T:OK:%.2f\n", medianTemp);
                    currentState = IDLE;
                }
            }
            break;

        case SERVO1_FORWARD:
            servo1.write(40);
            delay(500); // Aguarda o servo se posicionar
            currentState = MEASURE_OXI;
            //currentState = SERVO1_BACKWARD;
            oxiSamples.clear();
            sampleCount = 0;
            lastSampleTime = currentTime;
            oxiStartTime = currentTime; // inicia timeout para o dedo ser inserido
            fingerOnSensor = false;
            
            // Reseta o buffer do sensor para limpá-lo de dados antigos
            #if !SIMULATE_SENSORS
                sensor.setup(); // Reinicializa o sensor, limpando o FIFO
            #endif
            break;

        case MEASURE_OXI:
            
            #if SIMULATE_SENSORS
                if (currentTime - lastSampleTime >= 100) { 
                    int fakeSPO2 = 98;
                    oxiSamples.push_back(fakeSPO2);
                    sampleCount++;
                    lastSampleTime = currentTime;
                    
                    #if DEBUGMODE
                        Serial.print("Simulando SpO2... Amostra: ");
                        Serial.println(sampleCount);
                    #endif

                    if (sampleCount >= 5) {
                        int medianSPO2 = getMedianInt(oxiSamples);
                        SerialPi.printf("O:OK:%d\n", medianSPO2);
                        currentState = SERVO1_BACKWARD; 
                    }
                }
            
            #else
                sensor.check(); 
                // Se ainda não detectamos o dedo, verificamos timeout
                if (!fingerOnSensor && oxiStartTime > 0 && (currentTime - oxiStartTime >= OXI_FINGER_TIMEOUT))
                {
                    #if DEBUGMODE
                        Serial.println("Timeout: dedo nao inserido. Cancelando medicao de Oxi.");
                    #endif
                    currentState = SERVO1_BACKWARD;
                    oxiStartTime = 0;
                    break;
                }

                if (currentTime - lastSampleTime >= 100) 
                {
                    if (sensor.available()) 
                    {
                        uint32_t irValue = sensor.getIR();
                        uint32_t redValue = sensor.getRed(); // Ler ambos os valores para os filtros
                        sensor.nextSample();

                        // Lógica de detecção com histerese para evitar oscilação
                        if (irValue < FINGER_OFF_THRESHOLD && fingerOnSensor) {
                            fingerOnSensor = false;
                            beatAvg = 0;
                            SPO2 = 0;
                            #if DEBUGMODE
                            Serial.println("\nDedo removido. Aguardando...");
                            #endif
                        } else if (irValue > FINGER_ON_THRESHOLD && !fingerOnSensor) {
                            fingerOnSensor = true;
                            lastBeat = millis(); // Reseta o timer da batida quando o dedo é detectado
                            #if DEBUGMODE
                                Serial.println("Dedo detectado. Realizando medição...");
                            #endif
                        }
                        
                        if (fingerOnSensor) 
                        {
                            // Processamento do sinal para encontrar o batimento
                            int16_t IR_signal = pulseIR.ma_filter(pulseIR.dc_filter(irValue));
                            bool beatIR = pulseIR.isBeat(IR_signal);

                            // *** CORREÇÃO PARA SpO2 ***
                            // Processa o sinal vermelho da mesma forma, chamando isBeat() para forçar
                            // o cálculo interno do valor AC (avgAC) na biblioteca.
                            int16_t Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
                            pulseRed.isBeat(Red_signal); // A chamada é necessária, mesmo sem usar o resultado.

                            if (beatIR) 
                            {
                                long beatInterval = currentTime - lastBeat;
                                
                                if (beatInterval > 600) 
                                { 
                                    lastBeat = currentTime;
                                    
                                    long btpm = 60000 / beatInterval;
                                    if (btpm > 40 && btpm < 200) {
                                        beatAvg = bpm.filter((int16_t)btpm);
                                    }
                                    
                                    // Calcula o SpO2 somente quando temos um batimento válido
                                    long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
                                    long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
                                    int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;

                                    if ((RX100 >= 0) && (RX100 < 184)) 
                                    {
                                        SPO2 = spo2_table[RX100];
                                        oxiSamples.push_back(SPO2);
                                        sampleCount++;
                                    }
                                }
                            }

                            // Imprime os valores no Monitor Serial a cada 1 segundo para não poluir o terminal
                            lastPrintTime = currentTime;
                            #if DEBUGMODE
                                if (beatAvg > 0 && SPO2 > 0) 
                                {
                                    Serial.print("BPM: ");
                                        Serial.print(beatAvg);
                                        Serial.print(" | SpO2: ");
                                        Serial.print(SPO2);
                                        Serial.println("%");
                                    }
                    
                                if (sampleCount >= 5) 
                                {
                                    int medianSPO2 = getMedianInt(oxiSamples);
                                    SerialPi.printf("O:OK:%d\n", medianSPO2); // Resposta para o Pi
                                    
                                    Serial.printf("O:OK:%d\n", medianSPO2);
                                    currentState = SERVO1_BACKWARD;
                                    oxiStartTime = 0; // sucesso, limpa timeout
                                }
                
                                if (currentTime - lastPrintTime > 1000) 
                                {
                                    lastPrintTime = currentTime;
                                    Serial.print("Measuring SpO2... Samples: ");
                                    Serial.println(sampleCount);
                                }
                            #endif
                            lastSampleTime = currentTime;
                        }
                    }
                    else
                    {
                        sensor.check();
                        
                            if(try_count > 200)
                            {   
                                SerialPi.printf("O:NACK:%d\n");
                                #if DEBUGMODE
                                    Serial.printf("O:NACK:%d\n");
                                #endif
                                try_count = 0;
                                currentState = SERVO1_BACKWARD;
                                oxiStartTime = 0; // garante reset do timeout ao sair
                            }
                            else
                            {   
                                #if DEBUGMODE
                                    if(try_count % 10 == 0)
                                        Serial.println("WAIT ...");
                                #endif
                                try_count ++;
                            }
                        
                    }
                }
            #endif
            break;

        case SERVO1_BACKWARD:
            Serial.println("Retire o o dedo do sensor.");
            delay(1600);
            servo1.write(120);
            delay(500);
            currentState = IDLE;
            break;

        case SERVO2_FORWARD:
            servo2.write(160);
            delay(500);
            SerialPi.println("P1:OK");
            currentState = IDLE;
            break;

        case SERVO2_BACKWARD:
            servo2.write(0);
            delay(500);
            SerialPi.println("P2:OK");
            currentState = IDLE;
            break;

        case RELE_CONTROL:
            if (currentTime - releStartTime >= 400) 
            {
                digitalWrite(RELE_PIN, LOW);
                SerialPi.println("P:OK");
                currentState = IDLE;
            }
            break;
    }
}