#include "cartridge.hpp"
#include <cassert>
#include <stdio.h>
#include <stdlib.h>

void cartridge_empty(cartridge_t* cart)
{
    // @TODO: initialise ROM with a simple test
    cart->rom = cart->prg = cart->prgRam = cart->chr = 0;
    cart->hasChrRam = false;
}

// Load the ROM from a file
void cartridge_load(cartridge_t* cart, const char* fileName)
{
    printf("Loading ROM: %s!\n", fileName);
    FILE* f = fopen(fileName, "rb");

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* rom = (uint8_t*)malloc(size);
    fread(rom, size, 1, f);
    fclose(f);

    // Read ROM file header:
    int mapperType    = (rom[7] & 0xF0) | (rom[6] >> 4);
    size_t prgSize    = rom[4] * 0x4000;
    size_t prgRamSize = rom[8] ? rom[8] * 0x2000 : 0x2000;
    size_t chrSize    = rom[5] ? rom[5] * 0x2000 : 0x2000;
    bool hasChrRam    = rom[5] == 0;
    bool mirroring    = (rom[6] & 1);

    cart->rom    = rom;
    cart->prg    = rom + 16;
    cart->prgRam = (uint8_t*)malloc(prgRamSize);
    cart->chr    = (hasChrRam) ? (uint8_t*)malloc(chrSize): cart->prg + prgSize;
    cart->hasChrRam = hasChrRam;

    printf("ROM PRG: %lu, CHR: %lu, mapper: %d, mirroring: %s\n", prgSize, chrSize, mapperType, mirroring ? "|" : "--");
    printf("RAM PRG: %lu, CHR: %lu\n", prgRamSize, (hasChrRam) ? chrSize : 0LU);

    cart->ciRamMirroring = mirroring ? PPU::VERTICAL : PPU::HORIZONTAL;

    if (mapperType != 0)
    {
        fprintf(stderr, "%s: mapper %d not supported\n", fileName, mapperType);
        exit(0);
    }
}

void cartridge_free(cartridge_t* cart)
{
    free(cart->rom);
    free(cart->prgRam);
    if (cart->hasChrRam)
        free(cart->chr);
}
