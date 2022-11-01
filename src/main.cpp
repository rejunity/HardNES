#include <stdio.h>
#include <stdlib.h>
#include <SDL2/SDL.h>
#define CHIPS_IMPL
#include "m6502.h"
#include "ppu.hpp"
#include <cassert>

void logNull(...) {}
#define LOG logNull
//#define LOG printf

struct dma_t
{
    u16 readAddr;
    u16 counter;
};

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

cpu_pins_t cpuPins;
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
    size_t prgRamSize = rom[8] ? rom[8] * 0x2000 : 0x2000;
    size_t chrSize    = rom[5] ? rom[5] * 0x2000 : 0x2000;
    bool hasChrRam    = rom[5] == 0;
    bool mirroring    = (rom[6] & 1);

    prg    = rom + 16;
    prgRam = (uint8_t*)malloc(prgRamSize);
    chr    = (hasChrRam) ? (uint8_t*)malloc(chrSize): prg + prgSize;

    printf("ROM PRG:%lu, CHR:%lu, mapper:%d, mirroring: %s\n", prgSize, chrSize, mapperType, mirroring ? "|" : "--");
    printf("RAM PRG:%lu, CHR:%lu\n", prgRamSize, (hasChrRam) ? chrSize : 0LU);

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
    ppuPins.rw = true;
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  cpuPins.d  = ram[addr & 0x7FF];            break;  // RAM
        case 0x2000 ... 0x3FFF:  ppuPins.cs = true; ppuPins.a = addr & 0x7; break;  // PPU
        case 0x6000 ... 0x7FFF:  cpuPins.d  = prgRam[addr & 0x1FFF];        break;  // NROM-256 cartridge RAM
        case 0x8000 ... 0xFFFF:  cpuPins.d  = prg[addr & 0x7FFF];           break;  // NROM-256 cartridge
    }
}

void dma_oam(u8 bank);
void cpu_write(u16 addr)
{
    ppuPins.cs = false;
    ppuPins.rw = false;
    u8 v = cpuPins.d;
    switch (addr)
    {
        case            0x4014:  dma_oam(v);                                break;  // OAM DMA
        case 0x0000 ... 0x1FFF:  ram[addr & 0x7FF] = v;                     break;  // RAM
        case 0x2000 ... 0x3FFF:  ppuPins.cs = true;
                                 ppuPins.a = addr & 0x7; ppuPins.d = v;     break;  // PPU
        case 0x6000 ... 0x7FFF:  prgRam[addr & 0x1FFF] = v;                 break;  // NROM-256 cartridge
    }
}

void ppu_read(u16 addr)
{
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  ppuPins.ad = chr[addr % 0x2000];           break;  // CHR-ROM/RAM
        case 0x2000 ... 0x3EFF:  ppuPins.ad = ciRam[nt_mirror(addr)];       break;  // CIRAM nametables
    }
}

void ppu_write(u16 addr)
{
    u8 v = ppuPins.ad;
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  chr[addr % 0x2000] = v;                    break;  // CHR ROM/RAM
        case 0x2000 ... 0x3EFF:  ciRam[nt_mirror(addr)] = v;                break;  // CIRAM nametables
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

    cpuPins.bits = m6502_tick(&cpu, cpuPins.bits);
    if (cpuPins.rw)
    { // read
        cpu_read(cpuPins.a);
        LOG("CPU t: %lu, op:%02X@%04X, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X => %02X\n", cycle, opcode, pc,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            cpuPins.a, cpuPins.d);
    }
    else
    { // write
        LOG("CPU t: %lu, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X <= %02X\n", cycle,
            m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
            cpuPins.a, cpuPins.d);
        cpu_write(cpuPins.a);
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
    cpuPins.bits = m6502_init(&cpu, &desc);

    dma_t dma;
    dma_init(&dma);

    ppu_t ppu;
    ppuPins = ppu_init(&ppu);

    u8 ppuAddressLatch = 0; // 74LS373 address latch

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
    size_t cycles = 0;
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

        // https://wiki.nesdev.com/w/index.php/CPU_pin_out_and_signal_description
        // CLK : 21.47727 MHz (NTSC) or 26.6017 MHz (PAL) clock input.
        // Internally, this clock is divided by 12 (NTSC 2A03) or 16 (PAL 2A07) to feed the 6502's clock input φ0, which is in turn inverted to form φ1, which is then inverted to form φ2.
        // φ1 is high during the first phase (half-cycle) of each CPU cycle, while φ2 is high during the second phase.
        // M2 : Can be considered as a "signals ready" pin. It is a modified version the 6502's φ2 (which roughly corresponds to the CPU input clock φ0) that allows for slower ROMs. CPU cycles begin at the point where M2 goes low.
        // In the NTSC 2A03, M2 has a duty cycle of 5/8th, or 350ns/559ns. Equivalently, a CPU read (which happens during the second, high phase of M2) takes 1 and 7/8th PPU cycles. The internal φ2 duty cycle is exactly 1/2 (one half).
        // In the PAL 2A07, the duty cycle is not known, but suspected to be 19/32.

        // http://nesdev.com/2A03%20technical%20reference.txt
        // PHI2: this output is the divide-by-12 result of the CLK signal (1.79 MHz).
        // The internal 6502 along with function generating hardware, is clocked off
        // this frequency, and is available externally here so that it can be used as a
        // data bus enable signal (when at logic level 1) for external 6502 address
        // decoder logic. The signal has a 62.5% duty cycle.

        // /$4017R: goes active (zero) when A0-A15 = $4017, R/W = 0, and PHI2 = 1. This
        // informs an external 3-state inverter to throw controller port data onto the
        // D0-D7 lines.
        //
        // /$4016R: goes active (zero) when A0-A15 = $4016, R/W = 0, and PHI2 = 1.
        //
        // $4016W.0, $4016W.1, $4016W.2: these signals represent the real-time status
        // of a 3 bit writable register located at $4016 in the 6502 memory map. In
        // NES/FC consoles, $4016W.0 is used as a strobe line for the CMOS 4021 shift
        // register used inside NES/FC controllers.

if (true)
{
        bool verboseNMI = true;
        bool verboseIRQ = true;

        const size_t MASTER_CLOCK_CYCLES_PER_FRAME = 21477270 / 60;

//        1789772,5
//        1.7897725 Mhz
//        357954,5
        //printf("PAL@60Hz, master clock cycles per frame: %zu\n", MASTER_CLOCK_CYCLES_PER_FRAME);
        //for (size_t i = 0; i < MASTER_CLOCK_CYCLES_PER_FRAME; i++, cycles++)
        const size_t TOTAL_CYCLES = 29781 * 12;
        const size_t CYCLES_PER_ITERATION = 2;
        const bool SUBPIXEL_ITERATIONS = (CYCLES_PER_ITERATION < 4);
        for (size_t i = 0; i < TOTAL_CYCLES; i += CYCLES_PER_ITERATION)
        {
            cycles = i;
            bool cpuClkPosEdge = (cycles % 12 == 0);
            bool ppuClkPosEdge = (cycles % 4 == 0);
            bool ppuClkNegEdge = SUBPIXEL_ITERATIONS ? (cycles % 2 == 0): ppuClkPosEdge;
            bool m2 = (cycles % 12 > 7); // NTSC 2A03, M2 has a duty cycle of 5/8th,

            // 74LS139 address decoder
            ppuPins.cs = m2 && (cpuPins.a & 0x2000) && !(cpuPins.a & 0x4000) && !(cpuPins.a & 0x8000);
            ppuPins.rw = cpuPins.rw;
            ppuPins.a = cpuPins.a & 0b111;

            if (ppuClkPosEdge)
                ppuPins = ppu_tick(&ppu, ppuPins);
            cpuPins.nmi = ppuPins.irq;

            if (cpuClkPosEdge)
                cpuPins = dma_tick(&dma, cpuPins); // shares pins with CPU

            // @TODO: RDY should be handled inside m6502_tick()
            if (cpuClkPosEdge && !cpuPins.rdy)
                cpuPins.bits = m6502_tick(&cpu, cpuPins.bits);

            if (cpuClkPosEdge)
                LOG("CPU t: %zu, op:%02X@%04X, a:%02X x:%02X y:%02X s:%02X p:%02X, addr:%04X %s %02X\n", cycles/12,
                    prg[m6502_pc(&cpu)&0x7FFF], m6502_pc(&cpu),
                    m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu),
                    cpuPins.a, (cpuPins.rw ? "=>" : "<="), cpuPins.d);

            // @TODO: m2 should drive SRAM U1 CS pin through LS139 out pin 4
            // NOTE: SRAM U1 OE is shorted to ground
            if (m2 && cpuPins.rw) switch (cpuPins.a) // read
            {
                case 0x0000 ... 0x1FFF:  cpuPins.d = ram[cpuPins.a & 0x7FF];     break; // RAM
                case 0x2000 ... 0x3FFF:  cpuPins.d = ppuPins.d;                  break; // PPU
                case 0x6000 ... 0x7FFF:  cpuPins.d = prgRam[cpuPins.a & 0x1FFF]; break; // NROM-256 cartridge
                case 0x8000 ... 0xFFFF:  cpuPins.d = prg[cpuPins.a & 0x7FFF];    break; // NROM-256 cartridge
            }
            if (!cpuPins.rw) switch (cpuPins.a) // write
            {
                case            0x4014:  dma_request(&dma, cpuPins.d, 256);      break; // OAM DMA
                case 0x0000 ... 0x1FFF:  ram[cpuPins.a & 0x7FF]     = cpuPins.d; break; // RAM
                case 0x2000 ... 0x3FFF:  ppuPins.d                  = cpuPins.d; break; // PPU
                case 0x6000 ... 0x7FFF:  prgRam[cpuPins.a & 0x1FFF] = cpuPins.d; break; // RAM on cartridge
                //case 0x8000 ... 0xFFFF: cpuPins.d &= prg[cpuPins.a & 0x7FFF]; break;  // ROM cartridge bus collision
            }

            // 74LS373 address latch
            if (ppuPins.ale) // Address Latch Enable
                ppuAddressLatch = ppuPins.ad;

            // @TODO: nametable address swizzling (aka mirroring) should be done here
            u16 ppuAddr = ((ppuPins.pa & 0x3F) << 8) + (ppuAddressLatch & 0xFF);

            // @TODO: find/measure precise ALE/RD/WR subpixel timing
            // @TODO: move ALE/RD/WR subpixel logic into ppu_tick()
            // NOTE: (ALE) is high for one half pixel, or 94ns
            bool ppuRD = ((SUBPIXEL_ITERATIONS) ? !ppuPins.ale: true) && ppuPins.rd;
            bool ppuWR = ((SUBPIXEL_ITERATIONS) ? !ppuPins.ale: true) && ppuPins.wr;

            if (ppuRD) switch (ppuAddr) // read
            {
                case 0x0000 ... 0x1FFF: ppuPins.ad = chr[ppuAddr];              break; // CHR-ROM/RAM
                case 0x2000 ... 0x3EFF: ppuPins.ad = ciRam[nt_mirror(ppuAddr)]; break; // CIRAM nametables
            }
            if (ppuWR) switch (ppuAddr) // write
            {
                case 0x0000 ... 0x1FFF: chr[ppuAddr]              = ppuPins.ad; break; // CHR-ROM/RAM
                case 0x2000 ... 0x3EFF: ciRam[nt_mirror(ppuAddr)] = ppuPins.ad; break; // CIRAM nametables
            }

            // @TODO: move ALE/RD/WR subpixel logic into ppu_tick()
            if (ppuClkNegEdge)
                ppuPins.ale = false;

            if (cpuPins.nmi && verboseNMI) { printf("NMI @ t: %zu\n", cycles); verboseNMI = false; }
            if (cpuPins.irq && verboseIRQ) { printf("IRQ @ t: %zu\n", cycles); verboseIRQ = false; }
        }
}
else
{
        const size_t TOTAL_CYCLES = 29781;
        for (size_t i = 0; i < TOTAL_CYCLES; i++)
        {
            bool cs = ppuPins.cs; ppuPins.cs = false;
            tick(ppu, i);
            tick(ppu, i); ppuPins.cs = cs; // @TEMP: quick&dirty way to simulate M2 for now, should be handled by LS139
            tick(ppu, i);
            cpuPins.nmi = ppuPins.irq;
            if (ppuPins.irq) printf("NMI\n");
            if (ppuPins.cs && ppuPins.rw) // @TEMP: should be handled by LS139 https://wiki.nesdev.com/w/index.php/74139
                cpuPins.d = ppuPins.d;
            ppuPins.cs = false;
            if (dmaCounter > 0)
            {
                if (dmaCounter % 2 == 0)
                    cpu_read(dmaAddr++);
                else
                    cpu_write(0x2004);
                dmaCounter--;
            }
            else
                tick(cpu, i);
        }
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
