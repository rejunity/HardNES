#include <SDL2/SDL.h>
#define CHIPS_IMPL
#include "m6502.h"
#include "dma.hpp"
#include "ppu.hpp"
#include "cartridge.hpp"
#include <assert.h>
#include <stdio.h>
#include <time.h>

void logNull(...) {}
#define LOG logNull
//#define LOG printf

u16 ciram_mirror(Mirroring ciRamMirroring, u16 addr);

// @TODO: extract into separate file
typedef FILE vcd_t;
vcd_t* vcd_open(const char* fileName, char id = 'A')
{
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    char s[64];
    strftime(s, sizeof(s), "%c", tm);

    printf("Dump VCD: %s!\n", fileName);
    FILE* f = fopen(fileName, "w+");

    fprintf(f,  "$version\n\tHardNES VCD dump\n$end\n"
                "$date\n\t%s\n$end\n", s);
    fprintf(f,  "$timescale 47ns $end\n"); // @TODO: calculate based on MASTER_CLOCK_CYCLES_PER_FRAME

    // union cpu_pins_t
    //         unsigned a : 16;
    //         unsigned d : 8;
    //         unsigned rw : 1;
    //         unsigned sync : 1;
    //         unsigned irq : 1;
    //         unsigned nmi : 1;
    //         unsigned rdy : 1;
    //         unsigned dummy : 1;
    //         unsigned rst : 1;

    fprintf(f,  "$scope module TOP $end\n"
                "$scope module CPU $end\n"
                "$var wire 16 %c addr $end\n"
                "$var wire  8 %c data $end\n"
                "$var wire  1 %c rw $end\n"
                "$var wire  1 %c sync $end\n"
                "$var wire  1 %c irq $end\n"
                "$var wire  1 %c nmi $end\n"
                "$var wire  1 %c rdy $end\n"
                "$var wire  1 %c rst $end\n"
                "$upscope $end\n",
                id  , id+1, id+2, id+3,
                id+4, id+5, id+6, id+7);

    // struct ppu_pins_t
    //     unsigned rw : 1;
    //     unsigned d : 8;
    //     unsigned a : 3;
    //     unsigned cs : 1; // aka DataBusEnable (DBE) in Famicom circuit diagram
    //     unsigned ext : 4;
    //     unsigned irq : 1;
    //     unsigned ale : 1;
    //     unsigned ad : 8;
    //     unsigned pa : 6;
    //     unsigned rd : 1;
    //     unsigned wr : 1;
    //     unsigned rst : 1;
    //     u32 video;

    fprintf(f,  "$scope module PPU $end\n"
                "$var wire  1 %c rw $end\n"
                "$var wire  8 %c data $end\n"
                "$var wire  3 %c reg $end\n"
                "$var wire  1 %c cs $end\n"
                "$var wire  1 %c irq $end\n"
                "$var wire  1 %c ale $end\n"
                "$var wire  8 %c ppu_addr_data $end\n"
                "$var wire  6 %c ppu_addr $end\n"
                "$var wire  1 %c rd $end\n"
                "$var wire  1 %c wr $end\n"
                "$var wire  1 %c rst $end\n"
                "$upscope $end\n"
                "$upscope $end\n",
                id+8, id+9, id+10, id+11,
                id+12, id+13, id+14, id+15,
                id+16, id+17, id+18);

    fprintf(f,  "$enddefinitions $end\n");

    return f;
}

const char *BIT4[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};
const char *BIT3[8] = {
    [ 0] = "000", [ 1] = "001", [ 2] = "010", [ 3] = "011",
    [ 4] = "100", [ 5] = "101", [ 6] = "110", [ 7] = "111",
};
const char *BIT2[4] = {
    [ 0] = "00", [ 1] = "01", [ 2] = "10", [ 3] = "11",
};
const char  BIT1[2] = {
    [ 0] = '0', [ 1] = '1'
};

void vcd_dump(vcd_t* f, size_t cycle, cpu_pins_t cpuPins, ppu_pins_t ppuPins, char id = 'A')
{
    fprintf(f,  "#%zu\n", cycle);
    // union cpu_pins_t
    //         unsigned a : 16;
    //         unsigned d : 8;
    //         unsigned rw : 1;
    //         unsigned sync : 1;
    //         unsigned irq : 1;
    //         unsigned nmi : 1;
    //         unsigned rdy : 1;
    //         unsigned dummy : 1;
    //         unsigned rst : 1;

    fprintf(f,  "b%s%s%s%s %c\n",   BIT4[(cpuPins.a >> 12) & 0xF],
                                    BIT4[(cpuPins.a >>  8) & 0xF],
                                    BIT4[(cpuPins.a >>  4) & 0xF],
                                    BIT4[(cpuPins.a      ) & 0xF], id++);   // 1
    fprintf(f,  "b%s%s %c\n",       BIT4[(cpuPins.d >>  4) & 0xF],
                                    BIT4[(cpuPins.d      ) & 0xF], id++);   // 2
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.rw            ], id++);   // 3
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.sync          ], id++);   // 4
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.irq           ], id++);   // 5
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.nmi           ], id++);   // 6
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.rdy           ], id++);   // 7
    fprintf(f,  "%c%c\n",          BIT1[ cpuPins.rst           ], id++);   // 8

    // struct ppu_pins_t
    //     unsigned rw : 1;
    //     unsigned d : 8;
    //     unsigned a : 3;
    //     unsigned cs : 1; // aka DataBusEnable (DBE) in Famicom circuit diagram
    //     unsigned ext : 4;
    //     unsigned irq : 1;
    //     unsigned ale : 1;
    //     unsigned ad : 8;
    //     unsigned pa : 6;
    //     unsigned rd : 1;
    //     unsigned wr : 1;
    //     unsigned rst : 1;
    //     u32 video;

    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.rw            ], id++);   // 1
    fprintf(f,  "b%s%s %c\n",       BIT4[(ppuPins.d >>  4) & 0xF],
                                    BIT4[(ppuPins.d      ) & 0xF], id++);   // 2
    fprintf(f,  "b%s %c\n",         BIT3[ ppuPins.a             ], id++);   // 3
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.cs            ], id++);   // 4
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.irq           ], id++);   // 5
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.ale           ], id++);   // 6
    fprintf(f,  "b%s%s %c\n",       BIT4[(ppuPins.ad >> 4) & 0xF],
                                    BIT4[(ppuPins.ad     ) & 0xF], id++);   // 7
    fprintf(f,  "b%s%s %c\n",       BIT2[(ppuPins.pa >> 4) & 0x3],
                                    BIT4[(ppuPins.pa     ) & 0xF], id++);   // 8
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.rd            ], id++);   // 9
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.wr            ], id++);   // 10
    fprintf(f,  "%c%c\n",          BIT1[ ppuPins.rst           ], id++);   // 11
}

void vcd_close(vcd_t* f)
{
    fclose(f);
}

u16 ciram_mirror(Mirroring ciRamMirroring, u16 addr);

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
    FILE* vcd = vcd_open("mario.vcd");

    cartridge_t cart;
    if (argc > 1)
        cartridge_load(&cart, argv[1]);
    else
        cartridge_empty(&cart);

    uint8_t* prg    = cart.prg;
    uint8_t* prgRam = cart.prgRam;
    uint8_t* chr    = cart.chr;

    // initialize a 6502 instance:
    m6502_desc_t cpuDesc;
    cpuDesc.bcd_disabled = true;
    m6502_t cpu;
    cpu_pins_t cpuPins;
    cpuPins.bits = m6502_init(&cpu, &cpuDesc);

    dma_t dma;
    dma_init(&dma);

    ppu_t ppu;
    ppu_pins_t ppuPins;
    ppuPins = ppu_init(&ppu);

    u8 ppuAddressLatch = 0; // 74LS373 address latch

    uint8_t* ram = (uint8_t*)malloc(0x800);
    memset(ram, 0xFF, 0x800);
    m6502_set_a(&cpu, 0x00);
    m6502_set_x(&cpu, 0x00);
    m6502_set_y(&cpu, 0x00);
    m6502_set_s(&cpu, 0x00);
    m6502_set_p(&cpu, 0x04);

    uint8_t* ciRam = (uint8_t*)malloc(0x800);
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

#if 0
    const size_t MASTER_CLOCK_CYCLES_PER_SECOND = 21477270; // 21.47727 MHz (NTSC) 525 lines @ 29.97 frames per second
    const size_t FRAMES_PER_SECOND = 60;                    // 60 fps (almost NSTC)
    const size_t MASTER_CLOCK_CYCLES_PER_FRAME = MASTER_CLOCK_CYCLES_PER_SECOND / FRAMES_PER_SECOND;
#else
    const size_t MASTER_CLOCK_CYCLES_PER_FRAME = 29780.5 * 12; // 341 dots * 262 lines @ 60fps (21.441960 MHz)
#endif
    printf("NTSC@60Hz, master clock cycles per frame: %zu\n", MASTER_CLOCK_CYCLES_PER_FRAME);

    while (!quit)
    {
        frameStart = SDL_GetTicks();
        LOG("frame start: %ul\n", frameStart);

        // Handle events:
        SDL_Event e;
        while (SDL_PollEvent(&e)) switch (e.type)
        {
            case SDL_QUIT: quit = true; break;
        }

        bool verboseNMI = true;
        bool verboseIRQ = true;

        const size_t CYCLES_PER_ITERATION = 2;
        const bool SUBPIXEL_ITERATIONS = (CYCLES_PER_ITERATION < 4);
        for (size_t i = 0; i < MASTER_CLOCK_CYCLES_PER_FRAME; i += CYCLES_PER_ITERATION, cycles += CYCLES_PER_ITERATION)
        {
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
                case 0x0000 ... 0x1FFF:  cpuPins.d = ram[cpuPins.a & 0x7FF];        break; // RAM
                case 0x2000 ... 0x3FFF:  cpuPins.d = ppuPins.d;                     break; // PPU
                case 0x6000 ... 0x7FFF:  cpuPins.d = prgRam[cpuPins.a & 0x1FFF];    break; // NROM-256 cartridge
                case 0x8000 ... 0xFFFF:  cpuPins.d = prg[cpuPins.a & 0x7FFF];       break; // NROM-256 cartridge
            }
            if (!cpuPins.rw) switch (cpuPins.a) // write
            {
                case            0x4014:  dma_request(&dma, cpuPins.d, 256);         break; // OAM DMA
                case 0x0000 ... 0x1FFF:  ram[cpuPins.a & 0x7FF]     = cpuPins.d;    break; // RAM
                case 0x2000 ... 0x3FFF:  ppuPins.d                  = cpuPins.d;    break; // PPU
                case 0x6000 ... 0x7FFF:  prgRam[cpuPins.a & 0x1FFF] = cpuPins.d;    break; // RAM on cartridge
                //case 0x8000 ... 0xFFFF: cpuPins.d &= prg[cpuPins.a & 0x7FFF];     break; // ROM cartridge bus collision
            }

            // 74LS373 address latch
            if (ppuPins.ale) // Address Latch Enable
                ppuAddressLatch = ppuPins.ad;

            // @TODO: nametable address swizzling (aka mirroring) should be done here
            u16 ppuAddr = ((ppuPins.pa & 0x3F) << 8) + (ppuAddressLatch & 0xFF);

            // @TODO: find/measure precise ALE/RD/WR subpixel timing from a real PPU
            // @TODO: move ALE/RD/WR subpixel logic into ppu_tick()
            // NOTE: (ALE) is high for one half pixel, or 94ns
            bool ppuRD = ((SUBPIXEL_ITERATIONS) ? !ppuPins.ale: true) && ppuPins.rd;
            bool ppuWR = ((SUBPIXEL_ITERATIONS) ? !ppuPins.ale: true) && ppuPins.wr;

            #define CIRAM_ADDR(a) ciram_mirror(cart.ciRamMirroring, a)
            if (ppuRD) switch (ppuAddr) // read
            {
                case 0x0000 ... 0x1FFF: ppuPins.ad = chr[ppuAddr];                  break; // CHR-ROM/RAM
                case 0x2000 ... 0x3EFF: ppuPins.ad = ciRam[CIRAM_ADDR(ppuAddr)];    break; // CIRAM nametables
            }
            if (ppuWR) switch (ppuAddr) // write
            {
                case 0x0000 ... 0x1FFF: chr[ppuAddr]                = ppuPins.ad;   break; // CHR-ROM/RAM
                case 0x2000 ... 0x3EFF: ciRam[CIRAM_ADDR(ppuAddr)]  = ppuPins.ad;   break; // CIRAM nametables
            }
            #undef CIRAM_ADDR

            // @TODO: make configurable through command arguments
            if (cycles >= MASTER_CLOCK_CYCLES_PER_FRAME*60 && cycles < MASTER_CLOCK_CYCLES_PER_FRAME*61)
                vcd_dump(vcd, cycles, cpuPins, ppuPins);

            if (cpuPins.nmi && verboseNMI) { printf("NMI @ cpu: %zu, dot: (%zu,%zu)\n", cycles/12, i/(341*4), (i%(341*4))/4); verboseNMI = false; }
            if (cpuPins.irq && verboseIRQ) { printf("IRQ @ cpu: %zu, dot: (%zu,%zu)\n", cycles/12, i/(341*4), (i%(341*4))/4); verboseIRQ = false; }

            // @TODO: move ALE/RD/WR subpixel logic into ppu_tick()
            if (ppuClkNegEdge)
                ppuPins.ale = false;
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

    vcd_close(vcd);

    free(ram);
    free(ciRam);
    cartridge_free(&cart);

    printf("A:%02X X:%02X Y:%02X S:%02X P:%02X PC:%04X\n",
           m6502_a(&cpu), m6502_x(&cpu), m6502_y(&cpu), m6502_s(&cpu), m6502_p(&cpu), m6502_pc(&cpu));

    return 0;
}

// Get CIRAM address according to mirroring
u16 ciram_mirror(Mirroring ciRamMirroring, u16 addr)
{
    // @TODO: implement as a more readable bit swizzling
    switch (ciRamMirroring)
    {
        case VERTICAL:      return addr % 0x800;
        case HORIZONTAL:    return ((addr / 2) & 0x400) + (addr % 0x400);
        default:            return addr % 0x2000;
    }
}
