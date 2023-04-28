/**
 * 2023/04 Andrew Villeneuve
 * Emulate a multi-interface shoe flash to a Sony camera body
 * 
 * Pins: MI-SPI CLK on GPIO 0, MI-SPI DATA on GPIO 1
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

// Uncomment to internally generate a testing clock for use
// while NOT connected to a camera body
// #define TESTCLOCK true

// Define GPIOs
// Data must be a lower pin number than clock for the rx-miso PIO to map correctly
#define DATA 2
#define CLK 3
#define TRIG 4
#define F1 4

// Define timing constants
#define CLOCK_US 6
#define MISO_INIT_US 80
#define MOSI_INIT_US 150
#define FLASH_READY_US 260

// Define machine states
#define STATE_IDLE 0
#define STATE_TX_MISO 1
#define STATE_RX_MOSI 2
#define STATE_METERING_PF 3
#define STATE_METERING_EF 4
#define STATE_RECHARGE 5

#define mosi_packet_length 14
uint8_t old_mosi_packet[mosi_packet_length];
uint8_t new_mosi_packet[mosi_packet_length];

const int miso_packet_length = 26;
const uint8_t miso_packet[] = {
0xFF, 0xF9, 0xE7, 0xE6, 0xCA, 0xFD, 0xAC, 0xF7, 0x50, 0x04, 0x7F, 0x86, 0xCF, 0x88, 0xA0, 0x99, 0x99, 0x99, 0x99, 0xA7, 0xA7, 0xA7, 0xA7, 0xFF, 0x40, 0xFF
};

volatile absolute_time_t risetime;
volatile uint8_t state = STATE_IDLE;

// DMA and PIO variables
const PIO miso_pio = pio0;
const uint8_t miso_sm = 0;
uint8_t miso_dma_chan;
uint8_t miso_offset;

const PIO mosi_pio = pio1;
const uint8_t mosi_sm = 0;
uint8_t mosi_dma_chan;
uint8_t mosi_offset;

void clock_edge_callback(uint gpio, uint32_t events);
void miso_dma_setup(PIO pio, uint sm, uint dma_chan);
void generate_clock_byte();
void generate_clock_multibyte(int count);
void start_miso_tx();
void start_mosi_rx();
void assert_ready_pulse();

// Functions used for generating testing a testing clock when TESTCLOCK is defined
void generate_miso_packet_clock();
void generate_mosi_packet_clock();
void generate_clock_byte();
void generate_clock_multibyte(int count);