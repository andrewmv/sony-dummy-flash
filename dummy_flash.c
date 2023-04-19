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
#include "tx-miso.pio.h"
#include "rx-mosi.pio.h"

// Callback for edge changes on CLK
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
            //pio_sm_set_enabled(miso_pio, miso_sm, false);   // Disable MISO state machine
            //gpio_init(DATA);                                // Set DATA pin function to GPIO
        } else if (duration_us > MOSI_INIT_US) {         // MOSI start signal detected
            start_mosi_rx();
        } else if (duration_us > MISO_INIT_US) {         // MISO Start signal detected
            start_miso_tx();
        } // ...else, a regular bit pulse. These are handled by the PIOs
    }
}

/* Callback for RX DMA control chain (packet fully received)
 * When we enter this under normal circumstances, the CPU will be idle
 * for at least 12ms, so we should have time to print the last capture
 * out over serial
 */
void dma_callback() {
    // if (!memcmp(old_mosi_packet, new_mosi_packet, mosi_packet_length)) {
    //     memcpy(old_mosi_packet, new_mosi_packet, mosi_packet_length);
    //     for (int i = 0; i < mosi_packet_length; i++) {
    //         printf("%02X ", new_mosi_packet[i]);
    //     }
    //     printf("New MOSI Packet\n");
    // }
}

void start_miso_tx() {
    // Track what state we're in (not using this yet)
    state = STATE_TX_MISO;

    // Disable RX/MOSI state machine
    pio_sm_set_enabled(mosi_pio, mosi_sm, true);    

    // Stop RX/MOSI DMA
    dma_channel_abort(mosi_dma_chan);

    // Attach DATA pin function to TX PIO and set direction
    pio_gpio_init(miso_pio, DATA);                  
    pio_sm_set_consecutive_pindirs(miso_pio, miso_sm, DATA, 1, true);

    // Nuke any unshifted data from FIFO
    pio_sm_clear_fifos(miso_pio, miso_sm);

    // Start PIO SM
    pio_sm_set_enabled(miso_pio, miso_sm, true);    
 
    // Force SM to beginning of program
    pio_sm_exec_wait_blocking(miso_pio, miso_sm, pio_encode_jmp(miso_offset)); 

    // Nuke any unshifted data from OSR 
    pio_sm_exec_wait_blocking(miso_pio, miso_sm, pio_encode_out(pio_null, 32)); 

    // Start DMA to fill TX FIFO
    dma_channel_set_read_addr(miso_dma_chan, miso_packet, true);
}

void start_mosi_rx() {
    // Track what state we're in (not using this yet)
    state = STATE_RX_MOSI;

    // Disable TX/MISO state machine
    pio_sm_set_enabled(miso_pio, miso_sm, false);

    // Stop TX/MISO DMA, in case TX was interrupted early
    dma_channel_abort(miso_dma_chan);

    // Switch DATA pin function from PIO to GPIO
    gpio_init(DATA);
    gpio_set_dir(DATA, GPIO_OUT);

    // Drive DATA high at 2mA for the entire transmission - the body will
    // assert by pulling it low.
    // Note that the RP2040's built-in pull-up function is too weak to keep
    // the line high.
    gpio_set_drive_strength(DATA, GPIO_DRIVE_STRENGTH_2MA);
    gpio_put(DATA, 1);

    // Start RX/MOSI PIO state machine
    // pio_sm_set_enabled(mosi_pio, mosi_sm, true);

    // Force SM to beginning of program
    // pio_sm_exec_wait_blocking(mosi_pio, mosi_sm, pio_encode_jmp(mosi_offset));

    // Nuke any unpushed data from ISR. This could happen if last MOSI
    // shift was interrupted on a non-byte boundary
    // pio_sm_exec_wait_blocking(miso_pio, miso_sm, pio_encode_in(pio_null, 32)); 

    // Start DMA to empty RX FIFO
    // dma_channel_set_write_addr(mosi_dma_chan, new_mosi_packet, true);
}

// Configure DMA to feed data to the PIO state machine for
// shifting into the body
void miso_dma_setup(PIO pio, uint sm, uint dma_chan) {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    // Disable IRQs - transfer will stop automatically
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

// Configure DMA to read data shifted out of the body from
// the PIO state machine
void mosi_dma_setup(PIO pio, uint sm, uint dma_chan) {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    // Raise an IRQ at the end of the DMA transfer sequence (full packet received)
    channel_config_set_irq_quiet(&c, false);

    dma_channel_configure(
        dma_chan,           // The channel to configure
        &c,                 // Configuration struct
        NULL,               // Destination - will set later
        &pio->rxf[sm],      // Source - PIO RX FIFO
        mosi_packet_length, // Transfer count (size of source array)
        false               // Don't start yet
    );

    // Raise IRQ line 0 when the DMA transfer finishes
    // dma_channel_set_irq0_enabled(dma_chan, true);

    // Connect callback to that IRQ, and enable it
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_callback);
    // irq_set_enabled(DMA_IRQ_0, true);
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

void generate_miso_packet_clock() {
    // Drive a start pulse   
    gpio_put(CLK, 1);
    sleep_us(MISO_INIT_US + 10);
    gpio_put(CLK, 0);
    sleep_us(400);
    // Now clock some bits
    generate_clock_multibyte(miso_packet_length);
}

void generate_mosi_packet_clock() {
    // Drive a start pulse
    gpio_put(CLK, 1);
    sleep_us(MOSI_INIT_US + 10);
    gpio_put(CLK, 0);
    sleep_us(400);
    generate_clock_multibyte(14);
}

int main() {
    // Setup Serial
    stdio_init_all();
    printf("Ready\n");

    // Setup GPIO
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(CLK);
#ifdef TESTCLOCK
    gpio_set_dir(CLK, GPIO_OUT);
#else
    gpio_set_dir(CLK, GPIO_IN);
#endif

    // Setup PIO State Machines
    uint miso_offset = pio_add_program(miso_pio, &miso_program);
    miso_program_init(miso_pio, miso_sm, miso_offset, CLK, DATA);

    // uint mosi_offset = pio_add_program(mosi_pio, &mosi_program);
    // mosi_program_init(mosi_pio, mosi_sm, mosi_offset, CLK, DATA);

    // Setup DMA
    miso_dma_chan = dma_claim_unused_channel(true);
    miso_dma_setup(miso_pio, miso_sm, miso_dma_chan);

    mosi_dma_chan = dma_claim_unused_channel(true);
    mosi_dma_setup(mosi_pio, mosi_sm, mosi_dma_chan);

    // Setup IRQ callbacks
    gpio_set_irq_enabled_with_callback(CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &clock_edge_callback);

    while(true) {
#ifdef TESTCLOCK        
        generate_miso_packet_clock();
        sleep_ms(250);
        generate_mosi_packet_clock();
        sleep_ms(250);
#else
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
    }
}
