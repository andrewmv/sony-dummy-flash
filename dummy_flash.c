/**
 * 2023/04 Andrew Villeneuve
 * Emulate a multi-interface shoe flash to a Sony camera body
 * 
 * Pins: MI-SPI CLK on GPIO 0, MI-SPI DATA on GPIO 1
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "dummy_flash.pio.h"

// Define GPIOs
#define CLK 2
#define DATA 3
#define TRIG 4
#define F1 4

// Define timing constants
#define CLOCK_US 6
#define MISO_INIT_US 80
#define MOSI_INIT_US 150
#define SYNC_US 260

// Define machine states
#define STATE_IDLE 0
#define STATE_TX_MISO 1

const int miso_packet_length = 26;
const uint8_t miso_packet[] = {
    0x01, 
    0x02, 
    0x03, 
    0x04, 
    0xC9, 
    0xFF, 
    0xBF, 
    0xF7, 
    0x70, 
    0x06, 
    0x7F, 
    0xBF, 
    0x4C, 
    0x00, 
    0xA8, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x1B, 
    0xFF, 
    0x1A
};
// const uint8_t miso_packet[] = {0xFF, 0xFD, 0xFF, 0xFF, 0xC9, 0xFF, 0xBF, 0xF7, 0x70, 0x06, 0x7F, 0xBF, 0x4C, 0x00, 0xA8, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x1B, 0xFF, 0x1A};
// const byte miso_packet[] = {0xAA, 0x55};

volatile unsigned long risetime = 0;
volatile int bytecount = 0;
volatile int bitcount = 0;
volatile int state = STATE_IDLE;

void __isr miso_complete_handler() {
    printf("DMA Transfer Complete");
}

// Configure DMA to feed data to the PIO state machine for
// shifting into the body
void miso_dma_setup(PIO pio, uint sm, uint dma_chan) {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_irq_quiet(&c, true);

    dma_channel_configure(
        dma_chan,           // The channel to configure
        &c,                 // Configuration struct
        &pio->txf[sm],      // Destination pointer (PIO)
        NULL,               // Source - will set later
        miso_packet_length, // Transfer count (size of source array)
        false               // Don't start yet
    );

    // Setup interrupt return handlers to service end-of-data
    irq_set_exclusive_handler(DMA_IRQ_0, miso_complete_handler);
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_enabled(dma_chan, true);
}

void miso_dma_send_packet(PIO pio, uint sm, uint dma_chan) {
    // Point DMA chan back at beginning of miso data array, reset its counter,
    // and start it
    printf("Starting DMA Channel\n");
    dma_channel_set_read_addr(dma_chan, miso_packet, true);
    // dma_channel_set_trans_count(dma_chan, miso_packet_length, true);
}

void generate_clock_byte() {
    for (int i = 0; i < 8; i++) {
        gpio_put(CLK, 1);
        sleep_us(16);
        gpio_put(CLK, 0);
        sleep_us(16);
    }
}

void generate_clock_multibyte(int count) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    for (int i = 0; i < count; i++) {
        generate_clock_byte();
        sleep_us(160);
    }
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

int main() {
    // Setup Serial
    stdio_init_all();
    printf("Ready\n");

    // Setup GPIO
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(CLK);
    gpio_set_dir(CLK, GPIO_OUT);

    // Setup PIO State Machine
    uint miso_offset = pio_add_program(pio0, &dummy_flash_program);
    // uint miso_sm = pio_claim_unused_sm(pio, true);
    uint miso_sm = 0;
    dummy_flash_program_init(pio0, miso_sm, miso_offset, CLK, DATA);

    // Setup DMA
    uint dma_chan = dma_claim_unused_channel(true);
    miso_dma_setup(pio0, 0, dma_chan);

    // Start SM - it will start waiting for data in TX FIFO
    pio_sm_set_enabled(pio0, miso_sm, true);

    // Manually load some data into the SM's TX FIFO
    // Only the MSB 8 bits of each word are used
    // pio_sm_put_blocking(pio0, miso_sm, (uint32_t)0xAA000000);
    // pio_sm_put_blocking(pio0, miso_sm, (uint32_t)0x55000000);
    // pio_sm_put_blocking(pio0, miso_sm, (uint32_t)0xAA000000);
    // pio_sm_put_blocking(pio0, miso_sm, (uint32_t)0x55000000);

    while(true) {
        miso_dma_send_packet(pio0, miso_sm, 0);
        generate_clock_multibyte(miso_packet_length);
        sleep_ms(250);
    }
}
