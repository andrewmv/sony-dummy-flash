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
#define STATE_TX_MOSI 2

const int miso_packet_length = 26;
const uint8_t miso_packet[] = {
    0xFF, 
    0xFD, 
    0xFF, 
    0xFF, 
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

volatile absolute_time_t risetime = 0;
volatile uint8_t bytecount = 0;
volatile uint8_t bitcount = 0;
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
void miso_dma_send_packet();
void generate_clock_byte();
void generate_clock_multibyte(int count);