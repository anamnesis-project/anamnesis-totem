#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "driver/gpio.h"

const gpio_num_t PIN_SDA = GPIO_NUM_21;
const gpio_num_t PIN_SCL = GPIO_NUM_22;

/**
 * @brief Payload of each measurement save in eeprom memory
 */
typedef union __attribute__((packed)) {

  struct{
    uint8_t SysPressure; // Systolic Pressure - 25  [mmHg]
    uint8_t DiaPressure; // Diastolic Pressure      [mmHg]
    uint8_t Pulse;       // Pulse                   [bpm]
    uint8_t padding0[7]; // data padding always: 0x15 0x20 0x04 0x3F 0x10 0x80 0x00 
    uint8_t id;          // id of measurement  (0-9)
    uint8_t padding1;    // data padding always 0x00
    uint16_t CRC16;
  }data_measure;
} PayloadMeasure_t;

enum EventType : uint8_t {
  EVT_SCL_RISE = 0,
  EVT_START = 1,
  EVT_STOP  = 2
};

struct Event {
  uint8_t type;   // EventType
  uint8_t sda;    // 0 ou 1 (válido para SCL_RISE)
};
#define BYTE_BUF_SIZE 256
volatile uint8_t byteBuf[BYTE_BUF_SIZE];
volatile uint16_t byteHead = 0;
volatile uint16_t byteTail = 0;

volatile Event eventBuf[1024];
volatile uint16_t evHead = 0;
volatile uint16_t evTail = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

inline uint8_t fastRead(gpio_num_t pin) {
  return (GPIO.in >> pin) & 0x1;
}

IRAM_ATTR void pushEvent(uint8_t type, uint8_t sda) {
  uint16_t next = (evHead + 1) & (sizeof(eventBuf)/sizeof(eventBuf[0]) - 1);
  if (next == evTail) { // buffer cheio, descarta o mais antigo
    evTail = (evTail + 1) & (sizeof(eventBuf)/sizeof(eventBuf[0]) - 1);
  }
  eventBuf[evHead].type = type;
  eventBuf[evHead].sda = sda;
  evHead = next;
}

void IRAM_ATTR onSclRise() {
  uint8_t sda = fastRead(PIN_SDA);
  pushEvent(EVT_SCL_RISE, sda);
}

void IRAM_ATTR onSdaChange() {
  uint8_t scl = fastRead(PIN_SCL);
  uint8_t sda = fastRead(PIN_SDA);
  if (scl) {
    if (sda == 0) pushEvent(EVT_START, 0);
    else pushEvent(EVT_STOP, 1);
  }
}

bool popEvent(Event &out) {
  bool has = false;
  portENTER_CRITICAL(&mux);
  if (evTail != evHead) {
    out.type = eventBuf[evTail].type;
    out.sda = eventBuf[evTail].sda;
    evTail = (evTail + 1) & (sizeof(eventBuf)/sizeof(eventBuf[0]) - 1);
    has = true;
  }
  portEXIT_CRITICAL(&mux);
  return has;
}

void processEvent(const Event &ev) {
 static enum { IDLE, RECEIVING } state = IDLE;
  static uint8_t bitCount = 0;
  static uint8_t currentByte = 0;
  static bool awaitingAck = false;

  if (ev.type == EVT_START) {
    state = RECEIVING;
    bitCount = 0;
    currentByte = 0;
    awaitingAck = false;
    return;
  } else if (ev.type == EVT_STOP) {
    state = IDLE;
    return;
  } else if (ev.type == EVT_SCL_RISE) {
    if (state == IDLE) return;

    if (awaitingAck) {
      awaitingAck = false;
      bitCount = 0;
      currentByte = 0;
      return;
    }

    currentByte = (currentByte << 1) | (ev.sda & 1);
    bitCount++;
    if (bitCount == 8) {
      uint16_t next = (byteHead + 1) & (BYTE_BUF_SIZE - 1);
      if (next != byteTail) { // evita sobrescrever dados não lidos
        byteBuf[byteHead] = currentByte;
        byteHead = next;
      }
      awaitingAck = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== I2C Sniffer (ESP32) ===");
  Serial.printf("SDA pin: %d, SCL pin: %d\n", PIN_SDA, PIN_SCL);

  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  attachInterrupt(digitalPinToInterrupt((int)PIN_SCL), onSclRise, RISING);
  attachInterrupt(digitalPinToInterrupt((int)PIN_SDA), onSdaChange, CHANGE);

  Serial.println("Sniffer pronto. Esperando tráfego I2C...");
}

void loop() {
  Event ev;
  bool printed = false;
  while (popEvent(ev)) {
    processEvent(ev);
  }

  while (byteTail != byteHead) {
    uint8_t b = byteBuf[byteTail];
    byteTail = (byteTail + 1) & (BYTE_BUF_SIZE - 1);
    Serial.printf("Byte capturado: 0x%02X\n", b);
    printed = true;
  }
  if( printed) {
    printed = false;
    Serial.printf("first transmition.\n");
  }
}