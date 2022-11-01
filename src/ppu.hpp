#include <cstdint>
#pragma once

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

// PPU pin out
//              .--\/--.
//       R/W -> |01  40| -- +5
//    CPU D0 <> |02  39| -> ALE
//    CPU D1 <> |03  38| <> PPU AD0
//    CPU D2 <> |04  37| <> PPU AD1
//    CPU D3 <> |05  36| <> PPU AD2
//    CPU D4 <> |06  35| <> PPU AD3
//    CPU D5 <> |07  34| <> PPU AD4
//    CPU D6 <> |08  33| <> PPU AD5
//    CPU D7 <> |09  32| <> PPU AD6
//    CPU A2 -> |10  31| <> PPU AD7
//    CPU A1 -> |11  30| -> PPU A8
//    CPU A0 -> |12  29| -> PPU A9
//       /CS -> |13  28| -> PPU A10
//      EXT0 <> |14  27| -> PPU A11
//      EXT1 <> |15  26| -> PPU A12
//      EXT2 <> |16  25| -> PPU A13
//      EXT3 <> |17  24| -> /RD
//       CLK -> |18  23| -> /WR
//      /INT <- |19  22| <- /RST
//       GND -- |20  21| -> VOUT
//              `------'
//

struct ppu_pins_t
{
    unsigned rw : 1;
    unsigned d : 8;
    unsigned a : 3;
    unsigned cs : 1; // aka DataBusEnable (DBE) in Famicom circuit diagram
    unsigned ext : 4;
    unsigned irq : 1;
    unsigned ale : 1;
    unsigned ad : 8;
    unsigned pa : 6;
    unsigned rd : 1;
    unsigned wr : 1;
    unsigned rst : 1;
    u32 video;
};

namespace PPU {

enum Scanline  { VISIBLE, POST, NMI, PRE };
enum Mirroring { VERTICAL, HORIZONTAL };

/* Sprite buffer */
struct Sprite
{
    u8 id;     // Index in OAM.
    u8 x;      // X position.
    u8 y;      // Y position.
    u8 tile;   // Tile index.
    u8 attr;   // Attributes.
    u8 dataL;  // Tile data (low).
    u8 dataH;  // Tile data (high).
};

/* PPUCTRL ($2000) register */
union Ctrl
{
    struct
    {
        unsigned nt     : 2;  // Nametable ($2000 / $2400 / $2800 / $2C00).
        unsigned incr   : 1;  // Address increment (1 / 32).
        unsigned sprTbl : 1;  // Sprite pattern table ($0000 / $1000).
        unsigned bgTbl  : 1;  // BG pattern table ($0000 / $1000).
        unsigned sprSz  : 1;  // Sprite size (8x8 / 8x16).
        unsigned slave  : 1;  // PPU master/slave.
        unsigned nmi    : 1;  // Enable NMI.
    };
    u8 r;
};

/* PPUMASK ($2001) register */
union Mask
{
    struct
    {
        unsigned gray    : 1;  // Grayscale.
        unsigned bgLeft  : 1;  // Show background in leftmost 8 pixels.
        unsigned sprLeft : 1;  // Show sprite in leftmost 8 pixels.
        unsigned bg      : 1;  // Show background.
        unsigned spr     : 1;  // Show sprites.
        unsigned red     : 1;  // Intensify reds.
        unsigned green   : 1;  // Intensify greens.
        unsigned blue    : 1;  // Intensify blues.
    };
    u8 r;
};

/* PPUSTATUS ($2002) register */
union Status
{
    struct
    {
        unsigned bus    : 5;  // Not significant.
        unsigned sprOvf : 1;  // Sprite overflow.
        unsigned sprHit : 1;  // Sprite 0 Hit.
        unsigned vBlank : 1;  // In VBlank?
    };
    u8 r;
};

/* Loopy's VRAM address */
union Addr
{
    struct
    {
        unsigned cX : 5;  // Coarse X.
        unsigned cY : 5;  // Coarse Y.
        unsigned nt : 2;  // Nametable.
        unsigned fY : 3;  // Fine Y.
    };
    struct
    {
        unsigned l : 8;
        unsigned h : 7;
    };
    unsigned addr : 14;
    unsigned r : 15;
};

}

struct ppu_t
{
    u8 cgRam[0x20];                 // VRAM for palettes
    u8 oamMem[0x100];               // VRAM for sprite properties
    PPU::Sprite oam[8], secOam[8];  // Sprite buffers

    PPU::Addr vAddr, tAddr;         // Loopy V, T
    u8 fX;                          // Fine X
    u8 oamAddr;                     // OAM address

    PPU::Ctrl ctrl;                 // PPUCTRL   ($2000) register
    PPU::Mask mask;                 // PPUMASK   ($2001) register
    PPU::Status status;             // PPUSTATUS ($2002) register

    // Background latches:
    u8 nt, at, bgL, bgH;
    // Background shift registers:
    u8 atShiftL, atShiftH; u16 bgShiftL, bgShiftH;
    bool atLatchL, atLatchH;

    // Rendering counters:
    int scanline, dot;
    bool frameOdd;
};

ppu_pins_t ppu_init(ppu_t* ppu);
ppu_pins_t ppu_tick(ppu_t* ppu, ppu_pins_t pins);
u32* ppu_pixels();
