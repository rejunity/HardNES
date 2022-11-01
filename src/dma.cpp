#include "dma.hpp"
#include <cassert>

void dma_init(dma_t* dma)
{
    dma->counter = 0;
}

void dma_request(dma_t* dma, u8 bank, u16 count)
{
    dma->readAddr = bank << 8;
    dma->counter = count * 2;
}

cpu_pins_t dma_tick(dma_t* dma, cpu_pins_t pins)
{
    pins.rdy = (dma->counter > 0);
    if (!pins.rdy)
        return pins;

    assert(dma->counter > 0);
    dma->counter--;
    if (dma->counter % 2 == 1)
    { // read
        pins.a = dma->readAddr++;
        pins.rw = true;
    }
    else
    { // write
        pins.a = 0x2004;
        pins.rw = false;
    }
    return pins;
}
