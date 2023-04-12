/**
 * 2023/04 Andrew Villeneuve
 * Emulate a multi-interface shoe flash to a Sony camera body
 * 
 * Pins: MI-SPI CLK on GPIO 0, MI-SPI DATA on GPIO 1
 */

#include <stdio.h>
#include "dummy_flash.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "dummy_flash.pio.h"

// Callbacks for edge changes on CLK
void clock_edge_callback(uint gpio, uint32_t events) {
    uint64_t duration_us = 0;
    if (events & GPIO_IRQ_EDGE_RISE) {
        risetime = get_absolute_time();
        return;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        absolute_time_t falltime = get_absolute_time();
        duration_us = absolute_time_diff_us(risetime, falltime);

        if (duration_us == 0) {
            // Ignore first falling edge if we powered on while CLK was high
            return;
        } else if (duration_us > SYNC_US) {
            state = STATE_IDLE;
            // Not implemented
        } else if (duration_us > MOSI_INIT_US) {
            state = STATE_TX_MOSI;
            // Not implemented
        } else if (duration_us > MISO_INIT_US) {
            // MISO Start signal detected - change state and start TX DMA
            state = STATE_TX_MISO;
            miso_dma_send_packet();
        }

    }
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
}

void miso_dma_send_packet() {
    // Point DMA chan back at beginning of miso data array, reset its counter,
    // and start it
    dma_channel_set_read_addr(miso_dma_chan, miso_packet, true);
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

    // Setup IRQ callbacks
    gpio_set_irq_enabled_with_callback(CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &clock_edge_callback);

    // Setup PIO State Machine
    uint miso_offset = pio_add_program(miso_pio, &dummy_flash_program);
    dummy_flash_program_init(miso_pio, miso_sm, miso_offset, CLK, DATA);

    // Setup DMA
    miso_dma_chan = dma_claim_unused_channel(true);
    miso_dma_setup(miso_pio, miso_sm, miso_dma_chan);

    // Start SM - it will start waiting for data in TX FIFO
    pio_sm_set_enabled(miso_pio, miso_sm, true);

    while(true) {
        // miso_dma_send_packet(pio0, miso_sm, 0);
        generate_clock_multibyte(miso_packet_length);
        sleep_ms(250);
    }
}

/***
 * Bind CLK as input to CPU
 * Put ISR on both edges of CLK
 * Record start time on rise
 * Record duration on fall
 * > CLOCK_US: do nothing
 * > MISO_US: reset/start MISO DMA
 * > MOSI_US: stop/abort MISO DMA
***/
