#include "ppu.hpp"
#include <cassert>
#include <cstring>
#include <stdio.h>

extern void logNull(...);
#define LOG logNull
//#define LOG printf
//#define ERR logNull
#define ERR printf

#define NTH_BIT(x, n) (((x) >> (n)) & 1)

namespace PPU {
#include "palette.inc"

u8 cgRam[0x20];             // VRAM for palettes
u8 oamMem[0x100];           // VRAM for sprite properties
Sprite oam[8], secOam[8];   // Sprite buffers
u32 pixels[256 * 240];      // Video buffer

Addr vAddr, tAddr;          // Loopy V, T
u8 fX;                      // Fine X
u8 oamAddr;                 // OAM address

Ctrl ctrl;                  // PPUCTRL   ($2000) register
Mask mask;                  // PPUMASK   ($2001) register
Status status;              // PPUSTATUS ($2002) register

// Background latches:
u8 nt, at, bgL, bgH;
// Background shift registers:
u8 atShiftL, atShiftH; u16 bgShiftL, bgShiftH;
bool atLatchL, atLatchH;

// Rendering counters:
int scanline, dot;
bool frameOdd;
u16 busAddr;
u8 busData;
bool busRead;
int busWrite; // 2 cycle write
u32 video;
bool nmi;

inline bool rendering() { return mask.bg || mask.spr; }
inline int spr_height() { return ctrl.sprSz ? 16 : 8; }

/* Access PPU memory */
u8 rdPalette(u8 index)
{
    if ((index & 0x13) == 0x10) index &= ~0x10;
    return cgRam[index & 0x1F] & (mask.gray ? 0x30 : 0xFF);
}

u8 rd(u16 addr)
{
    if (busRead || busWrite)
        ERR("PPU bus read collision @(%dx%d), newaddr:%04X, oldadrr:%04X read:%d write:%d\n", scanline, dot, addr, busAddr, (int)busRead, (int)busWrite);
    switch (addr)
    {
        case 0x3F00 ... 0x3FFF: // on-chip palette RAMs
            return rdPalette(addr & 0xFF);

        default:
            busAddr = addr;
            busRead = true;
            return busData; // NOTE: returns stale data from the last cycle!
    }
}
void wr(u16 addr, u8 v)
{
    if (busRead || busWrite)
        ERR("PPU bus write collision @(%dx%d), value: %02X newaddr:%04X, oldadrr:%04X read:%d write:%d\n", scanline, dot, v, addr, busAddr, (int)busRead, (int)busWrite);
    switch (addr)
    {
        case 0x3F00 ... 0x3FFF:  // on-chip palette RAMs
            if ((addr & 0x13) == 0x10) addr &= ~0x10;
            cgRam[addr & 0x1F] = v;
            break;

        default:
            busAddr = addr;
            busData = v;
            busWrite = 2; // 2 cycles write
    }
}

/* Access PPU through registers. */
template <bool write> u8 access(u16 index, u8 v)
{
    static u8 res;      // Result of the operation.
    static bool latch;  // Detect second reading.

    const char* registerName = "unknown";
    switch (index)
    {
        case 0: registerName = "PPUCTRL ($2000)"; break;
        case 1: registerName = "PPUMASK ($2001)"; break;
        case 2: registerName = "PPUSTAT ($2002)"; break;
        case 3: registerName = "OAMADDR ($2003)"; break;
        case 4: registerName = "OAMDATA ($2004)"; break;
        case 5: registerName = "PPUSCRL ($2005)"; break;
        case 6: registerName = "PPUADDR ($2006)"; break;
        case 7: registerName = "PPUDATA ($2007)"; break;
    }

    /* Write into register */
    if (write)
    {
        res = v;

        LOG("PPU %s <= %02X\n", registerName, v);

        switch (index)
        {
            case 0:  ctrl.r = v; tAddr.nt = ctrl.nt; break;                 // PPUCTRL   ($2000)
            case 1:  mask.r = v; break;                                     // PPUMASK   ($2001)
            case 3:  oamAddr = v; break;                                    // OAMADDR   ($2003)
            case 4:  oamMem[oamAddr++] = v; break;                          // OAMDATA   ($2004)
            case 5:                                                         // PPUSCROLL ($2005)
                if (!latch) { fX = v & 7; tAddr.cX = v >> 3; }              // First write
                else  { tAddr.fY = v & 7; tAddr.cY = v >> 3; }              // Second write
                latch = !latch; break;
            case 6:                                                         // PPUADDR   ($2006)
                if (!latch) { tAddr.h = v & 0x3F; }                         // First write
                else        { tAddr.l = v; vAddr.r = tAddr.r; }             // Second write
                latch = !latch; break;
            case 7:  wr(vAddr.addr, v); vAddr.addr += ctrl.incr ? 32 : 1;   // PPUDATA ($2007)
                break;
        }
    }
    /* Read from register */
    else
    {
        switch (index)
        {
            // PPUSTATUS ($2002):
            case 2: res = (res & 0x1F) | status.r; status.vBlank = 0; latch = 0; break;
            case 4: res = oamMem[oamAddr]; break;   // OAMDATA ($2004).
            case 7:                                 // PPUDATA ($2007).
                    res = rd(vAddr.addr);
                    vAddr.addr += ctrl.incr ? 32 : 1;
                    break;
        }

        LOG("PPU %s => %02X\n", registerName, res);
    }
    return res;
}
template u8 access<0>(u16, u8); template u8 access<1>(u16, u8);

/* Calculate graphics addresses */
inline u16 nt_addr() { return 0x2000 | (vAddr.r & 0xFFF); }
inline u16 at_addr() { return 0x23C0 | (vAddr.nt << 10) | ((vAddr.cY / 4) << 3) | (vAddr.cX / 4); }
inline u16 bg_addr() { return (ctrl.bgTbl * 0x1000) + (nt * 16) + vAddr.fY; }
/* Increment the scroll by one pixel */
inline void h_scroll() { if (!rendering()) return; if (vAddr.cX == 31) vAddr.r ^= 0x41F; else vAddr.cX++; }
inline void v_scroll()
{
    if (!rendering()) return;
    if (vAddr.fY < 7) vAddr.fY++;
    else
    {
        vAddr.fY = 0;
        if      (vAddr.cY == 31)   vAddr.cY = 0;
        else if (vAddr.cY == 29) { vAddr.cY = 0; vAddr.nt ^= 0b10; }
        else                       vAddr.cY++;
    }
}
/* Copy scrolling data from loopy T to loopy V */
inline void h_update() { if (!rendering()) return; vAddr.r = (vAddr.r & ~0x041F) | (tAddr.r & 0x041F); }
inline void v_update() { if (!rendering()) return; vAddr.r = (vAddr.r & ~0x7BE0) | (tAddr.r & 0x7BE0); }
/* Put new data into the shift registers */
inline void reload_shift()
{
    bgShiftL = (bgShiftL & 0xFF00) | bgL;
    bgShiftH = (bgShiftH & 0xFF00) | bgH;

    atLatchL = (at & 1);
    atLatchH = (at & 2);
}

/* Clear secondary OAM */
void clear_oam()
{
    for (int i = 0; i < 8; i++)
    {
        secOam[i].id    = 64;
        secOam[i].y     = 0xFF;
        secOam[i].tile  = 0xFF;
        secOam[i].attr  = 0xFF;
        secOam[i].x     = 0xFF;
        secOam[i].dataL = 0;
        secOam[i].dataH = 0;
    }
}

/* Fill secondary OAM with the sprite infos for the next scanline */
void eval_sprites()
{
    int n = 0;
    for (int i = 0; i < 64; i++)
    {
        int line = (scanline == 261 ? -1 : scanline) - oamMem[i*4 + 0];
        // If the sprite is in the scanline, copy its properties into secondary OAM:
        if (line >= 0 and line < spr_height())
        {
            secOam[n].id   = i;
            secOam[n].y    = oamMem[i*4 + 0];
            secOam[n].tile = oamMem[i*4 + 1];
            secOam[n].attr = oamMem[i*4 + 2];
            secOam[n].x    = oamMem[i*4 + 3];

            if (++n >= 8)
            {
                status.sprOvf = true;
                break;
            }
        }
    }
}

/* Get sprite tile data address. */
u16 sprite_addr(int i)
{
    // Different address modes depending on the sprite height:
    u16 addr;
    if (spr_height() == 16)
        addr = ((oam[i].tile & 1) * 0x1000) + ((oam[i].tile & ~1) * 16);
    else
        addr = ( ctrl.sprTbl      * 0x1000) + ( oam[i].tile       * 16);

    unsigned sprY = (scanline - oam[i].y) % spr_height();  // Line inside the sprite.
    if (oam[i].attr & 0x80)
        sprY ^= spr_height() - 1;      // Vertical flip.
    addr += sprY + (sprY & 8);  // Select the second tile if on 8x16.

    return addr;
}

/* Process a pixel, draw it if it's on screen */
void pixel()
{
    u8 palette = 0, objPalette = 0;
    bool objPriority = 0;
    int x = dot - 2;

    video = 0; // @TODO: output proper background!
    if (scanline < 240 and x >= 0 and x < 256)
    {
        if (mask.bg and not (!mask.bgLeft && x < 8))
        {
            // Background:
            palette = (NTH_BIT(bgShiftH, 15 - fX) << 1) |
                       NTH_BIT(bgShiftL, 15 - fX);
            if (palette)
                palette |= ((NTH_BIT(atShiftH,  7 - fX) << 1) |
                             NTH_BIT(atShiftL,  7 - fX))      << 2;
        }
        // Sprites:
        if (mask.spr and not (!mask.sprLeft && x < 8))
            for (int i = 7; i >= 0; i--)
            {
                if (oam[i].id == 64) continue;  // Void entry.
                unsigned sprX = x - oam[i].x;
                if (sprX >= 8) continue;            // Not in range.
                if (oam[i].attr & 0x40) sprX ^= 7;  // Horizontal flip.

                u8 sprPalette = (NTH_BIT(oam[i].dataH, 7 - sprX) << 1) |
                                 NTH_BIT(oam[i].dataL, 7 - sprX);
                if (sprPalette == 0) continue;  // Transparent pixel.

                if (oam[i].id == 0 && palette && x != 255)
                    status.sprHit = true;
                sprPalette |= (oam[i].attr & 3) << 2;
                objPalette  = sprPalette + 16;
                objPriority = oam[i].attr & 0x20;
            }
        // Evaluate priority:
        if (objPalette && (palette == 0 || objPriority == 0)) palette = objPalette;

        video = nesRgb[rdPalette((rendering() ? palette : 0))];
        pixels[scanline*256 + x] = video;
    }
    // Perform background shifts:
    bgShiftL <<= 1; bgShiftH <<= 1;
    atShiftL = (atShiftL << 1) | atLatchL;
    atShiftH = (atShiftH << 1) | atLatchH;
}

/* Execute a cycle of a scanline */
template<Scanline s> void scanline_cycle()
{
    if (s == NMI and dot == 1)
        status.vBlank = true;
    else if (s == VISIBLE or s == PRE)
    {
        int spriteIndex;
        // Sprites:
        if (mask.spr) switch (dot)
        {
            case   1: clear_oam(); if (s == PRE) { status.sprOvf = status.sprHit = false; } break;

            case 256: eval_sprites(); break;
            //case 257 ... 321:
            case 260 ... 321:
                spriteIndex = (dot - 260) / 8;
                switch ((dot - 260) % 8)
                {
                    case 0: oam[spriteIndex] = secOam[spriteIndex];
                            busAddr = sprite_addr(spriteIndex); busRead = true; break;
                    case 1: oam[spriteIndex].dataL = busData; break;
                    case 2: busAddr += 8; busRead = true; break;
                    case 3: oam[spriteIndex].dataH = busData; break;
                }
                break;
        }
        // Background:
        if (mask.bg) switch (dot)
        {
            case 2 ... 255: case 322 ... 337:
                pixel();
                switch (dot % 8)
                {
                    // Nametable:
                    case 1:  busAddr  = nt_addr(); busRead = true; reload_shift(); break;
                    case 2:  nt       = busData;   break;
                    // Attribute:
                    case 3:  busAddr  = at_addr(); busRead = true; break;
                    case 4:  at       = busData;
                                                if (vAddr.cY & 2) at >>= 4;
                                                if (vAddr.cX & 2) at >>= 2; break;
                    // Background (low bits):
                    case 5:  busAddr  = bg_addr(); busRead = true; break;
                    case 6:  bgL      = busData;   break;
                    // Background (high bits):
                    case 7:  busAddr += 8;      busRead = true; break;
                    case 0:  bgH     = busData; h_scroll(); break;
                } break;
            case         256:  pixel(); bgH = busData;  v_scroll(); break;      // Vertical bump.
            case         257:  pixel(); reload_shift(); h_update(); break;                      // Update horizontal position.
            case 280 ... 304:  if (s == PRE)                            v_update(); break;      // Update vertical position.

            // No shift reloading:
            case             1:  busAddr = nt_addr(); busRead = true; if (s == PRE) status.vBlank = false; break;
            case 321: case 339:  busAddr = nt_addr(); busRead = true; break;
            // Nametable fetch instead of attribute:
            case           338:  nt = busData; break;
            case           340:  nt = busData; if (s == PRE && rendering() && frameOdd) dot++;
        }
    }

    nmi = status.vBlank && ctrl.nmi;
    // https://wiki.nesdev.com/w/index.php/NMI
    // Two 1-bit registers inside the PPU control the generation of NMI signals. Frame timing and accesses to the PPU's PPUCTRL
    // and PPUSTATUS registers change these registers as follows, regardless of whether rendering is enabled:
    //   1. Start of vertical blanking: Set NMI_occurred in PPU to true.
    //   2. End of vertical blanking, sometime in pre-render scanline: Set NMI_occurred to false.
    //   3. Read PPUSTATUS: Return old status of NMI_occurred in bit 7, then set NMI_occurred to false.
    //   4. Write to PPUCTRL: Set NMI_output to bit 7.
    // The PPU pulls /NMI low if and only if both NMI_occurred and NMI_output are true. By toggling NMI_output (PPUCTRL.7) during
    // vertical blank without reading PPUSTATUS, a program can cause /NMI to be pulled low multiple times, causing multiple NMIs
    // to be generated.
}

/* Execute a PPU cycle. */
void step()
{
    switch (scanline)
    {
        case 0 ... 239:  scanline_cycle<VISIBLE>(); break;
        case       240:  scanline_cycle<POST>();    break;
        case       241:  scanline_cycle<NMI>();     break;
        case       261:  scanline_cycle<PRE>();     break;
    }
    // Update dot and scanline counters:
    if (++dot > 340)
    {
        dot %= 341;
        if (++scanline > 261)
        {
            scanline = 0;
            frameOdd ^= 1;
        }
    }
}

void reset()
{
    frameOdd = false;
    scanline = dot = 0;
    ctrl.r = mask.r = status.r = 0;

    memset(pixels, 0x00, sizeof(pixels));
    memset(oamMem, 0x00, sizeof(oamMem));
}

}


ppu_pins_t ppu_init(ppu_t* ppu)
{
    PPU::reset();
    ppu_pins_t pins;
    pins.rst = pins.irq = pins.rd = pins.wr = pins.ale = pins.cs = false;
    return pins;
}

ppu_pins_t ppu_tick(ppu_t* ppu, ppu_pins_t pins)
{
    if (PPU::busRead) PPU::busData = pins.ad;
    PPU::busRead = false; //PPU::busWrite = false;

    PPU::step();

    // CPU <=> PPU
    if (pins.cs && !pins.rw) // write
        PPU::access<1>(pins.a, pins.d);
    if (pins.cs && pins.rw) // read
        pins.d = PPU::access<0>(pins.a, 0);

    pins.irq = PPU::nmi;
    pins.video = PPU::video;

    // @TODO: find out when RD and WR signals go high
    // @HACK: meanwhile use !ALE when accessing the memory
    if (PPU::busRead)
    {
        assert(pins.rd == false && pins.wr == false && pins.ale == false);
        pins.rd = true;
        pins.ale = true; // Address Latch Enable (ALE) is high for one half pixel, or 94ns
    }
    else if (PPU::busWrite == 2)
    {
        assert(pins.rd == false && pins.wr == false && pins.ale == false);
        pins.wr = true;
        pins.ale = true; // Address Latch Enable (ALE) is high for one half pixel, or 94ns
        PPU::busWrite--;
    }
    else if (PPU::busWrite == 1)
    {
        assert(pins.rd == false);
        assert(pins.wr == true);
        pins.ale = false;
        PPU::busWrite--;
    }
    else if (pins.ale)
    {
        pins.ale = false;
    }
    else
    {
        assert(pins.ale == false && PPU::busRead == false && PPU::busWrite == false);
        pins.rd = false;
        pins.wr = false;
    }

    if (pins.ale)
        pins.ad = PPU::busAddr & 0xFF;
    else
        pins.ad = PPU::busData;
    pins.pa = PPU::busAddr >> 8;

    return pins;
}

u32* ppu_pixels()
{
    return PPU::pixels;
}
