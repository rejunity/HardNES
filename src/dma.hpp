#include "m6502.h"
#include <cstdint>
#pragma once

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

struct dma_t
{
    u16 readAddr;
    u16 counter;
};

void dma_init(dma_t* dma);
void dma_request(dma_t* dma, u8 bank, u16 count);
cpu_pins_t dma_tick(dma_t* dma, cpu_pins_t pins);
