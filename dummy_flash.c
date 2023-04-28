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
    // Check if we entered this callback due to a TRIG/F1 fall,
    // reset state if so.
    if (gpio == TRIG) {
        state = STATE_RECHARGE;
        printf("FIRE\n");
        return;
    }

    // ...otherwise we're here becaue of a CLOCK edge
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
        } else if (duration_us > FLASH_READY_US) {
            // We asserted this - no action necessary
        } else if (duration_us > MOSI_INIT_US) {         // MOSI start signal detected
            start_mosi_rx();
        } else if (duration_us > MISO_INIT_US) {         // MISO Start signal detected
            if (state == STATE_METERING_PF) {
                printf("PF\n");
            } else {
                start_miso_tx();
            }
        } // ...else, a regular bit pulse. These are handled by the PIOs
    }
}

/* Callback for RX DMA control chain (packet fully received)
 * When we enter this under normal circumstances, the CPU will be idle
 * for at least 12ms, so we should have time to print the last capture
 * out over a serial UART >= 115200 baud
 */
void dma_callback() {
    // This should only ever be asserted by the MOSI DMA
    dma_channel_acknowledge_irq0(mosi_dma_chan);
    if (memcmp(old_mosi_packet, new_mosi_packet, mosi_packet_length) != 0) {
        memcpy(old_mosi_packet, new_mosi_packet, mosi_packet_length);
        for (int i = 0; i < mosi_packet_length; i++) {
            printf("%02X ", new_mosi_packet[i]);
        }
        printf("\n");
        // Check to see what kind of packet the body just sent
        if (new_mosi_packet[5] == 0x7d) {
            // This is part of a flash metering transaction
            if (new_mosi_packet[7] == 0xc0) {
                // This is the final flash exposure metering data
                state = STATE_METERING_EF;
                // Tell the body we're ready to fire the exposure flash
                assert_ready_pulse();
            } else {
                // This is pre-flash metering data
                state = STATE_METERING_PF;
                // Tell the body we're ready to fire the pre-flash
                assert_ready_pulse();
            }
        } else {
            // This is a boring old body state packet
            state = STATE_IDLE;
        }
    }
}

// Reverse CLK direction and assert a 270us "Ready" signal
void assert_ready_pulse() {
    gpio_set_dir(CLK, GPIO_OUT);
    gpio_put(CLK, 1);
    // We'll be inside an ISR, so sleep_us will panic
    busy_wait_us(FLASH_READY_US);
    gpio_put(CLK, 0);
    gpio_set_dir(CLK, GPIO_IN);
}

// Configure the PIOs and DMA for a flash-to-body transfer
void start_miso_tx() {
    // Track what state we're in (not using this yet)
    state = STATE_TX_MISO;

    // Disable RX/MOSI state machine
    pio_sm_set_enabled(mosi_pio, mosi_sm, false);

    // Stop RX/MOSI DMA
    dma_channel_abort(mosi_dma_chan);

    // Attach DATA pin function to TX PIO and set direction
    pio_gpio_init(miso_pio, DATA);
    pio_sm_set_consecutive_pindirs(miso_pio, miso_sm, DATA, 1, true);

    // Nuke any unshifted data from FIFO
    pio_sm_clear_fifos(miso_pio, miso_sm);

    // Restart and Enable PIO SM (sets OSR shift counter to Empty)
    uint32_t restart_mask = (1u << PIO_CTRL_SM_RESTART_LSB << miso_sm);
    restart_mask |= (1u << PIO_CTRL_SM_ENABLE_LSB << miso_sm);
    miso_pio->ctrl = restart_mask;

    // Force SM to beginning of program
    pio_sm_exec_wait_blocking(miso_pio, miso_sm, pio_encode_jmp(miso_offset)); 

    // Start DMA to fill TX FIFO
    dma_channel_set_read_addr(miso_dma_chan, miso_packet, true);      
}

// Configure the PIOs and DMA for a body-to-flash transfer
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

    // Restart and Enable PIO SM (sets ISR shift counter to Empty)
    uint32_t restart_mask = (1u << PIO_CTRL_SM_RESTART_LSB << mosi_sm);
    restart_mask |= (1u << PIO_CTRL_SM_ENABLE_LSB << mosi_sm);
    mosi_pio->ctrl = restart_mask;

    // Force SM to beginning of program
    pio_sm_exec_wait_blocking(mosi_pio, mosi_sm, pio_encode_jmp(mosi_offset));

    // Start DMA to empty RX FIFO
    dma_channel_set_write_addr(mosi_dma_chan, new_mosi_packet, true);
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
        &pio->txf[sm],      // Destination pointer - PIO TX FIFO
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
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    // Raise an IRQ at the end of the DMA transfer sequence (full packet received)
    channel_config_set_irq_quiet(&c, false);

    const volatile void *rx_fifo_addr = (io_rw_8*)&pio->rxf[sm] + 3;

    dma_channel_configure(
        dma_chan,           // The channel to configure
        &c,                 // Configuration struct
        NULL,               // Destination - will set later
        rx_fifo_addr,       // Source - leftmost octet of PIO RX FIFO
        mosi_packet_length, // Transfer count (size of source array)
        false               // Don't start yet
    );

    // Raise IRQ line 0 when the DMA transfer finishes
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Connect callback to that IRQ, and enable it
    irq_set_exclusive_handler(DMA_IRQ_0, dma_callback);
    irq_set_enabled(DMA_IRQ_0, true);
}

int main() {
    // Initialize last read packet to a reasonable value
    for (int i = 0; i < mosi_packet_length; i++) {
        old_mosi_packet[i] = 0;
    }

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
    gpio_init(TRIG);
    gpio_set_dir(TRIG, GPIO_IN);
    gpio_pull_up(TRIG);

    // Setup PIO State Machine
    miso_offset = pio_add_program(miso_pio, &miso_program);
    miso_program_init(miso_pio, miso_sm, miso_offset, CLK, DATA);

    mosi_offset = pio_add_program(mosi_pio, &mosi_program);
    mosi_program_init(mosi_pio, mosi_sm, mosi_offset, DATA);

    // Setup DMA
    miso_dma_chan = dma_claim_unused_channel(true);
    miso_dma_setup(miso_pio, miso_sm, miso_dma_chan);

    mosi_dma_chan = dma_claim_unused_channel(true);
    mosi_dma_setup(mosi_pio, mosi_sm, mosi_dma_chan);

    // Setup IRQ callbacks
    gpio_set_irq_enabled_with_callback(CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &clock_edge_callback);
    gpio_set_irq_enabled(TRIG, GPIO_IRQ_EDGE_FALL, true);

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

// *** Test Clock Stuff *** //

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
    generate_clock_multibyte(mosi_packet_length);
}
