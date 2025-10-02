#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ====================== UART1 (PA9=TX, PA10=RX) ====================== */
void uart1_init(void) {
    // Clock GPIOA + USART1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 -> AF7 (TX)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |=  (2 << (9 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |=  (7 << ((9 - 8) * 4));

    // PA10 -> AF7 (RX)
    GPIOA->MODER &= ~(3 << (10 * 2));
    GPIOA->MODER |=  (2 << (10 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOA->AFR[1] |=  (7 << ((10 - 8) * 4));

    // Baudrate ~115200 com APB2=84MHz
    USART1->BRR = 0x8B;  

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void uart1_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (c & 0xFF);
}

void uart1_print(const char *s) {
    while (*s) uart1_putc(*s++);
}

/* ====================== Estrutura de eventos ====================== */
typedef enum {
    EVT_SCL_RISE,
    EVT_START,
    EVT_STOP
} EventType;

typedef struct {
    EventType type;
    uint8_t sda;
    uint32_t ts;
} Event;

#define BUF_SIZE 256
volatile Event evbuf[BUF_SIZE];
volatile uint16_t ev_head = 0, ev_tail = 0;

static inline void pushEvent(EventType t, uint8_t sda) {
    uint16_t next = (ev_head + 1) % BUF_SIZE;
    if (next == ev_tail) { // buffer cheio, descarta mais antigo
        ev_tail = (ev_tail + 1) % BUF_SIZE;
    }
    evbuf[ev_head].type = t;
    evbuf[ev_head].sda = sda;
    evbuf[ev_head].ts = SysTick->VAL; // timestamp aproximado
    ev_head = next;
}

bool popEvent(Event *out) {
    if (ev_tail == ev_head) return false;
    *out = evbuf[ev_tail];
    ev_tail = (ev_tail + 1) % BUF_SIZE;
    return true;
}

/* ====================== GPIO + EXTI para I2C Sniff ====================== */
// Exemplo: PB8=SCL, PB9=SDA
#define PIN_SCL 8
#define PIN_SDA 9

void i2c_sniffer_init(void) {
    // Habilita clock GPIOB e SYSCFG
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PB8 e PB9 como entrada
    GPIOB->MODER &= ~(3 << (PIN_SCL * 2));
    GPIOB->MODER &= ~(3 << (PIN_SDA * 2));

    // Conecta PB8 -> EXTI8, PB9 -> EXTI9
    SYSCFG->EXTICR[2] &= ~((0xF << 0) | (0xF << 4));
    SYSCFG->EXTICR[2] |= (1 << 0); // PB8
    SYSCFG->EXTICR[2] |= (1 << 4); // PB9

    // Configura EXTI8 (SCL) para rising edge
    EXTI->IMR  |= (1 << PIN_SCL);
    EXTI->RTSR |= (1 << PIN_SCL);

    // Configura EXTI9 (SDA) para change edge
    EXTI->IMR  |= (1 << PIN_SDA);
    EXTI->RTSR |= (1 << PIN_SDA);
    EXTI->FTSR |= (1 << PIN_SDA);

    // NVIC
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ISR para PB8/PB9 */
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1 << PIN_SCL)) {
        pushEvent(EVT_SCL_RISE, (GPIOB->IDR >> PIN_SDA) & 1);
        EXTI->PR = (1 << PIN_SCL);
    }
    if (EXTI->PR & (1 << PIN_SDA)) {
        uint8_t scl = (GPIOB->IDR >> PIN_SCL) & 1;
        uint8_t sda = (GPIOB->IDR >> PIN_SDA) & 1;
        if (scl && sda == 0) pushEvent(EVT_START, sda);
        if (scl && sda == 1) pushEvent(EVT_STOP, sda);
        EXTI->PR = (1 << PIN_SDA);
    }
}

/* Dummy (necessária porque PA9..PA15 também chamam este vetor) */
void EXTI15_10_IRQHandler(void) { }

/* ====================== Decodificação simples ====================== */
void processEvent(const Event *ev) {
    static int bitCount = 0;
    static uint8_t byte = 0;
    static bool ackWait = false;
    static bool firstByte = false;

    switch (ev->type) {
    case EVT_START:
        //uart1_print("\r\n[START]\r\n");
        bitCount = 0;
        byte = 0;
        ackWait = false;
        firstByte = true;
        break;
    case EVT_STOP:
        //uart1_print("[STOP]\r\n");
        break;
    case EVT_SCL_RISE:
        if (ackWait) {
            if (firstByte) {
                char buf[64];
                uint8_t addr = (byte >> 1) & 0x7F;
                uint8_t rw   = byte & 1;
                snprintf(buf, sizeof(buf), "ADDR 0x%02X RW=%d %s\r\n",
                         addr, rw, ev->sda ? "NACK" : "ACK");
                uart1_print(buf);
                firstByte = false;
            } else {
                char buf[64];
                snprintf(buf, sizeof(buf), "DATA 0x%02X %s\r\n",
                         byte, ev->sda ? "NACK" : "ACK");
                uart1_print(buf);
            }
            byte = 0;
            bitCount = 0;
            ackWait = false;
        } else {
            byte = (byte << 1) | (ev->sda & 1);
            bitCount++;
            if (bitCount == 8) {
                ackWait = true;
            }
        }
        break;
    }
}

/* ====================== main ====================== */
int main(void) {
    uart1_init();
    uart1_print("\r\n=== I2C Sniffer Blackpill (USART1 A9/A10) ===\r\n");

    i2c_sniffer_init();

    while (1) {
        Event ev;
        if (popEvent(&ev)) {
            processEvent(&ev);
        }
    }
}
