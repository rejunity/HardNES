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
ppu_pins_t ppuPins;
uint8_t* ram;
uint8_t* rom;
uint8_t* prg;
uint8_t* prgRam;
uint8_t* chr;
uint8_t* ciRam;
PPU::Mirroring ciRamMirroring;

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

    ciRamMirroring = mirroring ? PPU::VERTICAL : PPU::HORIZONTAL;

    if (mapperType != 0)
    {
        fprintf(stderr, "%s: mapper %d not supported\n", fileName, mapperType);
        exit(0);
    }
}

/* Get CIRAM address according to mirroring */
u16 nt_mirror(u16 addr)
{
    switch (ciRamMirroring)
    {
        case PPU::VERTICAL:     return addr % 0x800;
        case PPU::HORIZONTAL:   return ((addr / 2) & 0x400) + (addr % 0x400);
        default:                return addr - 0x2000;
    }
}

// Access to memory
void cpu_read(u16 addr)
{
    ppuPins.cs = false;
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  M6502_SET_DATA(cpuPins, ram[addr % 0x800]); break;                     // RAM
        case 0x2000 ... 0x3FFF:  ppuPins.cs = true; ppuPins.rw = true; ppuPins.a = addr % 8;  break;    // PPU
        case 0x6000 ... 0x7FFF:  M6502_SET_DATA(cpuPins, prgRam[addr - 0x6000]); break;                 // NROM-256 cartridge
        case 0x8000 ... 0xFFFF:  M6502_SET_DATA(cpuPins, prg[addr - 0x8000]); break;                    // NROM-256 cartridge
    }
}

void dma_oam(u8 bank);
void cpu_write(u16 addr)
{
    ppuPins.cs = false;
    u8 v =  M6502_GET_DATA(cpuPins);
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  ram[addr % 0x800] = v;          break;                         // RAM
        case 0x2000 ... 0x3FFF:  ppuPins.cs = true; ppuPins.rw = false;
                                 ppuPins.a = addr % 8; ppuPins.d = v; break;                    // PPU
        case            0x4014:  dma_oam(v);                     break;                         // OAM DMA
        case 0x6000 ... 0x7FFF:  prgRam[addr - 0x6000] = v;      break;                         // NROM-256 cartridge
        case 0x8000 ... 0xFFFF:  prg[addr - 0x8000] = v;         break;                         // NROM-256 cartridge
    }
}

void ppu_read(u16 addr)
{
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  ppuPins.ad = chr[addr % 0x2000]; break;                        // CHR-ROM/RAM
        case 0x2000 ... 0x3EFF:  ppuPins.ad = ciRam[nt_mirror(addr)]; break;                    // Nametables
    }
}

void ppu_write(u16 addr)
{
    u8 v = ppuPins.ad;
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  chr[addr % 0x2000] = v; break;                                 // CHR-ROM/RAM
        case 0x2000 ... 0x3EFF:  ciRam[nt_mirror(addr)] = v; break;                             // Nametables
    }
}

u16 dmaAddr;
size_t dmaCounter;
void dma_oam(u8 bank)
{
    dmaAddr = bank*0x100;
    dmaCounter = 256 * 2;
}

void tick(m6502_t& cpu, size_t cycle)
{
    u16 pc = m6502_pc(&cpu);
    u8 opcode = prg[pc - 0x8000];

    cpuPins = m6502_tick(&cpu, cpuPins);
    const uint16_t addr = M6502_GET_ADDR(cpuPins);
    if (cpuPins & M6502_RW)
    { // read
        cpu_read(addr);
        LOG("CPU t: %lu, op:%02X@%04X, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X => %02X\n", cycle, opcode, pc,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            addr, M6502_GET_DATA(cpuPins));
    }
    else
    { // write
        LOG("CPU t: %lu, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X <= %02X\n", cycle,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            addr, M6502_GET_DATA(cpuPins));
        cpu_write(addr);
    }
}

void tick(ppu_t& ppu, size_t cycle)
{
    ppuPins = ppu_tick(&ppu, ppuPins);
    if (ppuPins.rd)
    { // read
        ppu_read((ppuPins.pa << 8) + ppuPins.ad);
    }
    else if (ppuPins.wr)
    { // write
        static u8 addressLatch = 0;
        if (ppuPins.ale) // Address Latch Enable
            addressLatch = ppuPins.ad;
        else
            ppu_write((ppuPins.pa << 8) + addressLatch);
    }
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

    ppu_t ppu;
    ppuPins = ppu_init(&ppu);

    ram = (uint8_t*)malloc(0x800);
    memset(ram, 0xFF, 0x800);
    m6502_set_a(&cpu, 0x00);
    m6502_set_x(&cpu, 0x00);
    m6502_set_y(&cpu, 0x00);
    m6502_set_s(&cpu, 0x00);
    m6502_set_p(&cpu, 0x04);

    ciRam = (uint8_t*)malloc(0x800);
    memset(ciRam, 0xFF, 0x800);

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
            bool cs = ppuPins.cs; ppuPins.cs = false;
            tick(ppu, i);
            tick(ppu, i); ppuPins.cs = cs; // quick&dirty way to simulate M2 for now, should be handled by LS139
            tick(ppu, i);
            if (ppuPins.irq) cpuPins |= M6502_NMI; else cpuPins &= ~M6502_NMI;
            if (ppuPins.cs && ppuPins.rw) // should be handled by LS139 https://wiki.nesdev.com/w/index.php/74139
                M6502_SET_DATA(cpuPins, ppuPins.d);
            ppuPins.cs = false;
            if (dmaCounter > 0)
            {
                if (dmaCounter % 2 == 0)
                    cpu_read(dmaAddr++);
                else
                    cpu_write(0x2014);
                dmaCounter--;
            }
            else
                tick(cpu, i);
        }

        SDL_RenderClear(renderer);
        SDL_UpdateTexture(canvas, NULL, ppu_pixels(), WIDTH * sizeof(u32));
        SDL_RenderCopy(renderer, canvas, NULL, NULL);
        SDL_RenderPresent(renderer);

        // Wait to mantain framerate:
        frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < DELAY)
            SDL_Delay((int)(DELAY - frameTime));

        LOG("frame time: %ul, pixels: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", frameTime,
            ppu_pixels()[0], ppu_pixels()[1],  ppu_pixels()[3],  ppu_pixels()[4],
            ppu_pixels()[5], ppu_pixels()[6],  ppu_pixels()[7],  ppu_pixels()[8],
            ppu_pixels()[9], ppu_pixels()[10], ppu_pixels()[11], ppu_pixels()[12]);
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
