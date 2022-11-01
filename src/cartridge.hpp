#include "ppu.hpp"
#include <cstdint>
#pragma once

struct cartridge_t
{
    uint8_t* rom;
    uint8_t* prg;
    uint8_t* prgRam;
    uint8_t* chr;
    PPU::Mirroring ciRamMirroring;
    bool hasChrRam;
};

void cartridge_empty(cartridge_t* cart);
void cartridge_load(cartridge_t* cart, const char* fileName);
void cartridge_free(cartridge_t* cart);
