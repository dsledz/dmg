/**
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Simulated Gameboy graphics driver. Right before VBlank, we read the
 * graphics memory and produce an image similar to what the gameboy
 * would display.
 */

#include "graphics.h"
#include "cpu.h"

using namespace DMG;

enum Color {
    Color0 = 0xffffffff,
    Color1 = 0xffbbbbbb,
    Color2 = 0xff666666,
    Color3 = 0xff000000,
    ColorZ = 0x00000000,
};

unsigned global_palette[] = { Color0, Color1, Color2, Color3 };
unsigned obj0_palette[] = { ColorZ, Color1, Color2, Color3 };
unsigned obj1_palette[] = { ColorZ, Color1, Color2, Color3 };

#define SCREEN_POSX 64
#define SCREEN_POSY 64
#define SCREEN_H 144
#define SCREEN_W 160

static surface_ptr
create_surface(short w, short h, bool alpha)
{
    SDL_Surface * s = SDL_CreateRGBSurface(SDL_SWSURFACE, w, h, 32,
        0x000000ff, 0x0000ff00, 0x00ff0000, alpha ? 0xff000000 : 0x00000000);
    if (s == NULL)
        throw VideoException();
    return surface_ptr(s);
}

static void
convert_palette(unsigned *palette, byte_t byte, bool obj)
{
    palette[0] = obj ? ColorZ : global_palette[(byte & 0x03)];
    palette[1] = global_palette[(byte & 0x0C) >> 2];
    palette[2] = global_palette[(byte & 0x30) >> 4];
    palette[3] = global_palette[(byte & 0xC0) >> 6];
}

static void
convert_line(unsigned *pixels, byte_t low, byte_t high, unsigned *palette,
             bool flip)
{
    unsigned j = 8;
    while (j > 0) {
        j--;
        unsigned idx = bit_isset(low, j) | (bit_isset(high, j) << 1);
        *pixels = palette[idx];
        pixels++;
    }
}

static void
flip_horizontal(SDL_Surface *surface)
{
    // XXX: assert locked
    unsigned *p = (unsigned *)surface->pixels;
    for (int y = 0; y < surface->h; y++)
        for (int x = 0; x < surface->w / 2; x++) {
            p[y * surface->w + x] ^= p[y * surface->w + surface->w - x - 1];
            p[y * surface->w + surface->w - x - 1] ^= p[y * surface->w + x];
            p[y * surface->w + x] ^= p[y * surface->w + surface->w - x - 1];
        }
}

static void
flip_vertical(SDL_Surface *surface)
{
    unsigned *p = (unsigned *)surface->pixels;
    for (int y  = 0; y < surface->h / 2; y++) {
        int y0 = y * surface->w;
        int y1 = (surface->h - y - 1) * surface->w;
        for (int x = 0; x < surface->w; x++) {
            p[y0 + x] ^= p[y1 + x];
            p[y1 + x] ^= p[y0 + x];
            p[y0 + x] ^= p[y1 + x];
        }
    }
}

static void
blit_image(SDL_Surface *src, SDL_Rect *rect,
           SDL_Surface *dest, short dX, short dY)
{
    SDL_Rect clip = {};

    clip.x = dX;
    clip.y = dY;

    SDL_BlitSurface(src, rect, dest, &clip);
}

class TileMap {
    private:
        TileMap(const TileMap &rhs) { }
    public:
        TileMap(byte_t palette) {
            convert_palette(_palette, palette, false);
        }
        ~TileMap() {
        }

        void init(const byte_t *map_data, bool base) {
            _base = base;
            _map = create_surface(256, 64, false);

            SDL_LockSurface(_map.get());
            unsigned *pixels = (unsigned *)_map->pixels;
            for (unsigned y = 0; y < 8; y++) {
                for (unsigned x = 0; x < 32; x++) {
                    for (unsigned l = 0; l < 8; l++) {
                    byte_t low = *map_data++;
                    byte_t high = *map_data++;
                    convert_line(&pixels[(y * 8 + l) * 256 + x*8],
                                 low, high, _palette, false);
                    }
                }
            }
            SDL_UnlockSurface(_map.get());
        }

        SDL_Rect *tile(byte_t idx) {
            // XXX: Handle tile offset better
            if (!_base)
                idx = (char)idx + 128;
            _tile_rect.x = (idx % 32) * 8;
            _tile_rect.y = (idx / 32) * 8;
            _tile_rect.h = 8;
            _tile_rect.w = 8;

            return &_tile_rect;
        }

        SDL_Surface *map(void) {
            return _map.get();
        }

        surface_ptr _map;
        SDL_Rect _tile_rect;
        unsigned _palette[4];
        bool _base;
};

class Sprite {
    private:
        Sprite(const Sprite &rhs) { }
    public:
        Sprite(void) { }
        ~Sprite(void) { }
        void init(unsigned idx, bool large, const byte_t *oam, const byte_t *vram) {
            _Y = oam[Oam::OamY];
            _X = oam[Oam::OamX];
            _pattern = oam[Oam::OamPattern];
            _flags = oam[Oam::OamFlags];

            unsigned palette[4];

            if (bit_isset(_flags, OAMFlags::SpritePalette))
                memcpy(palette, obj1_palette, sizeof(palette));
            else
                memcpy(palette, obj0_palette, sizeof(palette));

            // XXX: Handle transparency

            if (large) {
                _pattern &= 0xFE;
                _sprite = create_surface(8, 16, true);
            } else
                _sprite = create_surface(8, 8, true);

            SDL_LockSurface(_sprite.get());
            unsigned *pixels = (unsigned *)_sprite->pixels;

            // Handle first half of the sprite
            const byte_t *data = &vram[_pattern * 16];
            for (unsigned i = 0; i < 8; i++) {
                convert_line(&pixels[i * _sprite->w], *data++, *data++,
                    palette, false);
            }

            // Handle second block
            if (large) {
                for (unsigned i = 0; i < 8; i++)
                    convert_line(&pixels[(i + 8) * _sprite->w],
                        *data++, *data++, palette, false);
            }
            if (bit_isset(_flags, OAMFlags::SpriteFlipX))
                flip_horizontal(_sprite.get());
            if (bit_isset(_flags, OAMFlags::SpriteFlipY))
                flip_vertical(_sprite.get());
            SDL_UnlockSurface(_sprite.get());
        }

        void blit(SDL_Surface *screen)
        {
            if (!visible())
                return;

            if (bit_isset(_flags, OAMFlags::SpritePriority)) {
                // Reverse the priority. We accomplish this by
                // bliting the background over the sprite, then
                // blitting it to the background.
                // XXX: This still causes some glitches in Wario Land
                SDL_Rect rect;
                rect.x = _X - 7;
                rect.y = _Y - 16;
                rect.w = _sprite->w;
                rect.h = _sprite->h;
                SDL_SetColorKey(screen, SDL_SRCCOLORKEY,
                    global_palette[0]);
                blit_image(screen, &rect, _sprite.get(), 0, 0);
                SDL_SetColorKey(screen, 0, 0);
            }
            blit_image(_sprite.get(), NULL, screen, _X - 7, _Y - 16);
        }

        bool visible(void) {
            if (_X == 0 || _X >= 168)
                return false;
            if (_Y == 0 || _Y >= 160)
                return false;
            return true;
        }

    private:

        byte_t _X;
        byte_t _Y;
        byte_t _pattern;
        byte_t _flags;
        surface_ptr _sprite;
};

static surface_ptr
create_map(TileMap *tile_map, const byte_t *map)
{
    // Render the tiles
    surface_ptr tiles = create_surface(256, 256, false);
    SDL_LockSurface(tiles.get());
    memset(tiles->pixels, 0x00, tiles->w*tiles->h*4);
    SDL_UnlockSurface(tiles.get());

    for (unsigned y = 0; y < 32; y++) {
        for (unsigned x = 0; x < 32; x++) {
            SDL_Rect rect = {};
            rect.y = y * 8;
            rect.x = x * 8;
            SDL_BlitSurface(
                tile_map->map(),
                tile_map->tile(map[y * 32 + x]),
                tiles.get(),
                &rect);
        }
    }

    return std::move(tiles);
}

static void
blit_bg(SDL_Surface *screen, SDL_Surface *bg, short scx, short scy)
{
    // Blit the tiles on to the window
    SDL_Rect rect = { .x = scx, .y = scy, .h = SCREEN_H, .w = SCREEN_W };
    short y_overscan = (rect.y + rect.h) - bg->h;
    short x_overscan = (rect.x + rect.w) - bg->w;

    if (y_overscan > 0)
        rect.h -= y_overscan;
    else
        y_overscan = 0;
    if (x_overscan > 0)
        rect.w -= x_overscan;
    else
        x_overscan = 0;

    blit_image(bg, &rect, screen, 0, 0);

    if (x_overscan > 0) {
        rect.x = 0;
        rect.y = scy;
        rect.h = SCREEN_H;
        rect.w = x_overscan;

        blit_image(bg, &rect, screen, SCREEN_W - x_overscan, 0);
    }

    if (y_overscan > 0) {
        rect.x = scx;
        rect.y = 0;
        rect.h = y_overscan;
        rect.w = SCREEN_W;

        blit_image(bg, &rect, screen, 0, SCREEN_H - y_overscan);
    }

    if (x_overscan > 0 && y_overscan > 0) {
        rect.x = 0;
        rect.y = 0;
        rect.h = y_overscan;
        rect.w = x_overscan;

        blit_image(bg, &rect, screen, SCREEN_W - x_overscan,
            SCREEN_H - y_overscan);
    }
}

static void
blit_window(SDL_Surface *screen, SDL_Surface *win, short wx, short wy)
{
    wx -= 7;
    if (wx < SCREEN_W && wy < SCREEN_H)
        blit_image(win, NULL, screen, wx, wy);
}

SDLDisplay::SDLDisplay(MemoryBus *bus): _bus(bus), _fcycles(0)
{
    _vram.resize(0x2000);
    _oam.resize(0x0100);
    _reg.resize(0x0010);

    reset();

    _window = surface_ptr(SDL_SetVideoMode(
        292, 479, 32, SDL_HWSURFACE | SDL_DOUBLEBUF));
    if (_window == NULL)
        throw VideoException();

    SDL_Surface *gb = SDL_LoadBMP("gameboy.bmp");
    if (gb != NULL) {
        SDL_Rect rect = { .x = 0, .h = 0 };
        SDL_SetColorKey(gb, SDL_SRCCOLORKEY, 0xffff00ff);
        SDL_BlitSurface(gb, NULL, _window.get(), &rect);
        SDL_FreeSurface(gb);
    }

    SDL_Flip(_window.get());
}

SDLDisplay::~SDLDisplay(void)
{
}

void
SDLDisplay::reset(void)
{
    memset(&_vram[0], 0, _vram.size());
    memset(&_oam[0], 0, _oam.size());
    memset(&_reg[0], 0, _reg.size());
    write(VideoReg::LCDC, 0x91);
    write(VideoReg::SCY, 0x00);
    write(VideoReg::SCX, 0x00);
    write(VideoReg::LYC, 0x00);
    write(VideoReg::BGP, 0xFC);
    write(VideoReg::OBP0, 0xFF);
    write(VideoReg::OBP1, 0xFF);
    write(VideoReg::WY, 0x00);
    write(VideoReg::WX, 0x00);
    _fcycles = 0;
}

bool
SDLDisplay::valid(addr_t addr)
{
    return ((addr >= 0x8000 && addr < 0xA000) ||
            (addr >= 0xFF40 && addr <= 0xFF4B) ||
            (addr >= 0xFE00 && addr <= 0xFEFF));
}

void
SDLDisplay::write(addr_t addr, byte_t arg)
{
    if (addr >= 0x8000 && addr < 0xA000) {
        _vram[addr - 0x8000] = arg;
    } else if (addr >= 0xFF40 && addr <= 0xFF4B) {
        switch (addr) {
        case VideoReg::OBP0:
            convert_palette(obj0_palette, arg, true);
            break;
        case VideoReg::OBP1:
            convert_palette(obj1_palette, arg, true);
            break;
        default:
            break;
        }
        rget(addr) = arg;
    } else if (addr >= 0xFE00 && addr <= 0xFEA0) {
        _oam[addr - 0xFE00] = arg;
    }
}

byte_t
SDLDisplay::read(addr_t addr)
{
    if (addr >= 0x8000 && addr < 0xA000) {
        return _vram[addr - 0x8000];
    } else if (addr >= 0xFF40 && addr <= 0xFF4B) {
        return _reg[addr - VideoReg::LCDC];
    } else if (addr >= 0xFE00 && addr <= 0xFEA0) {
        return _oam[addr - 0xFE00];
    }
    return 0;
}

void
SDLDisplay::tick(unsigned cycles)
{
    _fcycles += cycles;

    byte_t &_stat = rget(VideoReg::STAT);
    byte_t &_ly = rget(VideoReg::LY);

    switch (_stat & 0x03) {
    case LCDMode::HBlankMode:
        if (_fcycles > H_BLANK_CYCLES) {
            // Transition to VBlank or OAM
            _fcycles -= H_BLANK_CYCLES;

            _ly = (_ly + 1) % SCANLINES;
            if (_ly == DISPLAY_LINES) {
                // Wait until our frame is finished
                render();
                _bus->trigger(Interrupt::VBlank);
                _stat = (_stat & 0xfc) | LCDMode::VBlankMode;
            } else {
                _stat = (_stat & 0xfc) | LCDMode::OAMMode;
            }
        }
        break;
    case LCDMode::OAMMode:
        if (_fcycles > OAM_CYCLES) {
            // Transition to Active
            _fcycles -= OAM_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::ActiveMode;
        }
        break;
    case LCDMode::ActiveMode:
        if (_fcycles > ACTIVE_CYCLES) {
            // Transition to HBlank
            _fcycles -= ACTIVE_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::HBlankMode;
        }
        break;
    case LCDMode::VBlankMode:
        if (_fcycles > V_BLANK_CYCLES) {
            // Transition to OAM
            _fcycles -= V_BLANK_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::OAMMode;
        }
        break;
    }
}

void
SDLDisplay::render(void)
{
    byte_t lcdc = rget(VideoReg::LCDC);

    if (!bit_isset(lcdc, LCDCBits::LCDEnabled))
        return;

    surface_ptr screen = create_surface(SCREEN_W, SCREEN_H, false);

    TileMap bg_tiles(rget(VideoReg::BGP));
    bg_tiles.init(&_vram[0x0800], false);
    TileMap obj_tiles(rget(VideoReg::BGP));
    obj_tiles.init(&_vram[0x0000], true);

    TileMap *tiles;
    if (bit_isset(lcdc, LCDCBits::BGTileData))
        tiles = &obj_tiles;
    else
        tiles = &bg_tiles;

    if (bit_isset(lcdc, LCDCBits::BGDisplay)) {
        const byte_t *map = bit_isset(lcdc, LCDCBits::BGTileMap) ?
            &_vram[0x1C00] : &_vram[0x1800];
        surface_ptr bg(create_map(tiles, map));
        short scx = rget(VideoReg::SCX);
        short scy = rget(VideoReg::SCY);
        blit_bg(screen.get(), bg.get(), scx, scy);
    }

    if (bit_isset(lcdc, LCDCBits::WindowDisplay)) {
        const byte_t *map = bit_isset(lcdc, LCDCBits::WindowTileMap) ?
            &_vram[0x1C00] : &_vram[0x1800];
        surface_ptr win(create_map(tiles, map));
        short wx = rget(VideoReg::WX);
        short wy = rget(VideoReg::WY);
        blit_window(screen.get(), win.get(), wx, wy);
    }

    const byte_t *oam = &_oam[0];
    Sprite sprites[40];
    for (unsigned i = 0; i < 40; i++) {
        sprites[i].init(i, bit_isset(lcdc, LCDCBits::OBJSize), oam, &_vram[0]);
        oam += 4;
    }

    if (bit_isset(lcdc, LCDCBits::OBJDisplay))
        for (unsigned i = 40; i > 0; i--)
            sprites[i-1].blit(screen.get());

    // Draw the window
    {
        SDL_Rect rect = { .x = SCREEN_POSX, .y = SCREEN_POSY };
        SDL_BlitSurface(screen.get(), NULL, _window.get(), &rect);
    }

    // Show the screen
    SDL_Flip(_window.get());
}

