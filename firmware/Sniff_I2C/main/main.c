#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "I2C_SNIFFER"

#define PIN_SDA 21
#define PIN_SCL 22

typedef enum {
    EVT_SCL_RISE = 0,
    EVT_START = 1,
    EVT_STOP  = 2
} EventType;

typedef struct {
    uint8_t type;   // EventType
    uint8_t sda;    // 0 ou 1 (válido para SCL_RISE)
} Event;

#define EVENT_BUF_SIZE 1024
static Event eventBuf[EVENT_BUF_SIZE];
static volatile uint16_t evHead = 0;
static volatile uint16_t evTail = 0;

#define BYTE_BUF_SIZE 256
static uint8_t byteBuf[BYTE_BUF_SIZE];
static volatile uint16_t byteHead = 0;
static volatile uint16_t byteTail = 0;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static inline uint8_t fastRead(gpio_num_t pin) {
    return gpio_get_level(pin);
}

static void IRAM_ATTR pushEvent(uint8_t type, uint8_t sda) {
    uint16_t next = (evHead + 1) & (EVENT_BUF_SIZE - 1);
    if (next == evTail) { // buffer cheio, descarta o mais antigo
        evTail = (evTail + 1) & (EVENT_BUF_SIZE - 1);
    }
    eventBuf[evHead].type = type;
    eventBuf[evHead].sda = sda;
    evHead = next;
}

static void IRAM_ATTR onSclRise(void* arg) {
    uint8_t sda = fastRead(PIN_SDA);
    pushEvent(EVT_SCL_RISE, sda);
}

static void IRAM_ATTR onSdaChange(void* arg) {
    uint8_t scl = fastRead(PIN_SCL);
    uint8_t sda = fastRead(PIN_SDA);
    if (scl) {
        if (sda == 0) pushEvent(EVT_START, 0);
        else pushEvent(EVT_STOP, 1);
    }
}

static bool popEvent(Event *out) {
    bool has = false;
    taskENTER_CRITICAL(&mux);
    if (evTail != evHead) {
        out->type = eventBuf[evTail].type;
        out->sda = eventBuf[evTail].sda;
        evTail = (evTail + 1) & (EVENT_BUF_SIZE - 1);
        has = true;
    }
    taskEXIT_CRITICAL(&mux);
    return has;
}

static void processEvent(const Event *ev) {
    static enum { IDLE, RECEIVING } state = IDLE;
    static uint8_t bitCount = 0;
    static uint8_t currentByte = 0;
    static bool awaitingAck = false;

    if (ev->type == EVT_START) {
        state = RECEIVING;
        bitCount = 0;
        currentByte = 0;
        awaitingAck = false;
        return;
    } else if (ev->type == EVT_STOP) {
        state = IDLE;
        return;
    } else if (ev->type == EVT_SCL_RISE) {
        if (state == IDLE) return;

        if (awaitingAck) {
            awaitingAck = false;
            bitCount = 0;
            currentByte = 0;
            return;
        }

        currentByte = (currentByte << 1) | (ev->sda & 1);
        bitCount++;
        if (bitCount == 8) {
            uint16_t next = (byteHead + 1) & (BYTE_BUF_SIZE - 1);
            if (next != byteTail) {
                byteBuf[byteHead] = currentByte;
                byteHead = next;
            }
            awaitingAck = true;
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== I2C Sniffer (ESP-IDF) ===");
    ESP_LOGI(TAG, "SDA pin: %d, SCL pin: %d", PIN_SDA, PIN_SCL);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_SDA) | (1ULL << PIN_SCL),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_set_intr_type(PIN_SCL, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PIN_SDA, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_SCL, onSclRise, NULL);
    gpio_isr_handler_add(PIN_SDA, onSdaChange, NULL);

    ESP_LOGI(TAG, "Sniffer pronto. Esperando tráfego I2C...");

    while (1) {
        Event ev;
        while (popEvent(&ev)) {
            processEvent(&ev);
        }

        while (byteTail != byteHead) {
            uint8_t b = byteBuf[byteTail];
            byteTail = (byteTail + 1) & (BYTE_BUF_SIZE - 1);
            ESP_LOGI(TAG, "Byte capturado: 0x%02X", b);
            // Aqui você pode mostrar no display, se desejar
        }
        vTaskDelay(1);
    }
}