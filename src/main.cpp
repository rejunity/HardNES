#include <stdio.h>
#include <stdlib.h>
#include <SDL2/SDL.h>
#define CHIPS_IMPL
#include "m6502.h"
#include "ppu.hpp"

void logNull(...) {}
#define LOG logNull
//#define LOG printf

uint64_t cpuPins;
uint8_t* ram;
uint8_t* rom;
uint8_t* prg;
uint8_t* prgRam;
uint8_t* chr;

// Load the ROM from a file
void load(const char* fileName)
{
    printf("Loading ROM: %s!\n", fileName);
    FILE* f = fopen(fileName, "rb");

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    rom = (uint8_t*)malloc(size);
    fread(rom, size, 1, f);
    fclose(f);

    // Read ROM file header:
    int mapperType    = (rom[7] & 0xF0) | (rom[6] >> 4);
    size_t prgSize    = rom[4] * 0x4000;
    size_t chrSize    = rom[5] * 0x2000;
    size_t prgRamSize = rom[8] ? rom[8] * 0x2000 : 0x2000;
    bool mirroring    = (rom[6] & 1);

    prg    = rom + 16;
    prgRam = (uint8_t*)malloc(prgRamSize);

    // CHR ROM:
    if (chrSize)
    {
        chr = rom + 16 + prgSize;
    }
    // CHR RAM:
    else
    {
        // chrRam = true;
        chrSize = 0x2000;
        chr = (uint8_t*)malloc(chrSize);
    }

    printf("ROM PRG:%lu, CHR:%lu, mapper:%d, mirroring: %s\n", prgSize, chrSize, mapperType, mirroring ? "|" : "--");
    printf("RAM PRG:%lu, CHR:%lu\n", prgRamSize, 0LU);

    PPU::set_mirroring(mirroring ? PPU::VERTICAL : PPU::HORIZONTAL);

    if (mapperType != 0)
    {
        fprintf(stderr, "%s: mapper %d not supported\n", fileName, mapperType);
        exit(0);
    }
}

// Access to memory
u8 read(u16 addr)
{
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  return ram[addr % 0x800];                              // RAM
        case 0x2000 ... 0x3FFF:  return PPU::access<false>(addr % 8, 0);                // PPU
        case 0x6000 ... 0x7FFF:  return prgRam[addr - 0x6000];                          // NROM-256 cartridge
        case 0x8000 ... 0xFFFF:  return prg[addr - 0x8000];                             // NROM-256 cartridge
    }
    return 0;
}

void dma_oam(u8 bank);
void write(u16 addr, u8 v)
{    
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  ram[addr % 0x800] = v;          break;                 // RAM
        case 0x2000 ... 0x3FFF:  PPU::access<true>(addr % 8, v); break;                 // PPU
        case            0x4014:  dma_oam(v);                     break;                 // OAM DMA
        case 0x6000 ... 0x7FFF:  prgRam[addr - 0x6000] = v;      break;                 // NROM-256 cartridge
        case 0x8000 ... 0xFFFF:  prg[addr - 0x8000] = v;         break;                 // NROM-256 cartridge
    }
}

size_t dmaCycles = 0;
void dma_oam(u8 bank)
{
    for (int i = 0; i < 256; i++)
        write(0x2014, read(bank*0x100 + i));
    dmaCycles = 256 * 2;
}

u8 chr_read(u16 addr)
{
    return chr[addr % 0x2000];
}

void chr_write(u16 addr, u8 v) {}
void cpu_set_nmi() { cpuPins |= M6502_NMI; }
void signal_scanline() {}

void tick(m6502_t& cpu, size_t cycle)
{
    u16 pc = m6502_pc(&cpu);
    u8 opcode = read(pc);

    cpuPins = m6502_tick(&cpu, cpuPins);
    const uint16_t addr = M6502_GET_ADDR(cpuPins);
    if (cpuPins & M6502_RW)
    { // read
        u8 v = read(addr);
        LOG("CPU t: %lu, op:%02X@%04X, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X => %02X\n", cycle, opcode, pc,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            addr, v);
        M6502_SET_DATA(cpuPins, v);
    }
    else
    { // write
        LOG("CPU t: %lu, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X <= %02X\n", cycle,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            addr, M6502_GET_DATA(cpuPins));
        write(addr, M6502_GET_DATA(cpuPins));
    }

    cpuPins &= ~M6502_NMI;
}

// Super Mario Bros. PRG
// 8000 78           SEI
// 8001 D8           CLD
// 8002 A9 10        LDA #10
// 8004 8D 00 20     STA $2000
// 8007 A2 FF        LDX #FF
// 8009 9A           TXS        -- Transfer X to Stack
// 800A AD 02 20     LDA $2002
// 800D 10 FB        BPL $FB    -- (-5)
// 800F AD 02 20     LDA $2002

int main(int argc, char** argv)
{
    printf("Hello, NES!\n");

    if (argc > 1)
        load(argv[1]);

    // initialize a 6502 instance:
    m6502_t cpu;
    m6502_desc_t desc;
    cpuPins = m6502_init(&cpu, &desc);
    PPU::reset();

    ram = (uint8_t*)malloc(0x800);
    memset(ram, 0xFF, 0x800);
    m6502_set_a(&cpu, 0x00);
    m6502_set_x(&cpu, 0x00);
    m6502_set_y(&cpu, 0x00);
    m6502_set_s(&cpu, 0x00);
    m6502_set_p(&cpu, 0x04);

    // Initialize graphics system
    const int WIDTH  = 256;
    const int HEIGHT = 240;
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_JOYSTICK);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    const int windowSize = 2;
    SDL_Window* window = SDL_CreateWindow("HardNES", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH * windowSize, HEIGHT * windowSize, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_Texture* canvas = SDL_CreateTexture (renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
    SDL_RenderSetLogicalSize(renderer, WIDTH, HEIGHT);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);    

    // Framerate control:
    u32 frameStart, frameTime;
    const int FPS   = 60;
    const int DELAY = 1000.0f / FPS;

    bool quit = false;
    while (!quit)
    {
        frameStart = SDL_GetTicks();
        LOG("frame start: %ul\n", frameStart);

        // Handle events:
        SDL_Event e;
        while (SDL_PollEvent(&e))
            switch (e.type)
            {
                case SDL_QUIT: quit = true; break;
            }

        const size_t TOTAL_CYCLES = 29781;
        for (size_t i = 0; i < TOTAL_CYCLES; i++)
        {
            PPU::step();
            PPU::step();
            PPU::step();
            tick(cpu, i);
            
            i += dmaCycles;
            dmaCycles = 0;
        }

        SDL_RenderClear(renderer);
        SDL_UpdateTexture(canvas, NULL, PPU::get_pixels(), WIDTH * sizeof(u32));
        SDL_RenderCopy(renderer, canvas, NULL, NULL);
        SDL_RenderPresent(renderer);

        // Wait to mantain framerate:
        frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < DELAY)
            SDL_Delay((int)(DELAY - frameTime));

        LOG("frame time: %ul, pixels: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", frameTime,
            PPU::get_pixels()[0], PPU::get_pixels()[1], PPU::get_pixels()[3], PPU::get_pixels()[4],
            PPU::get_pixels()[5], PPU::get_pixels()[6], PPU::get_pixels()[7], PPU::get_pixels()[8],
            PPU::get_pixels()[9], PPU::get_pixels()[10], PPU::get_pixels()[11], PPU::get_pixels()[12]);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    free(rom);
    free(ram);
    free(prgRam);

    printf("A:%02X X:%02X Y:%02X S:%02X P:%02X PC:%04X\n",
           m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu), m6502_pc(&cpu));

    return 0;
}
