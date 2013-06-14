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

#define SCREEN_X 64
#define SCREEN_Y 64

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
convert_palette(unsigned *palette, reg_t byte, bool obj)
{
    palette[0] = obj ? ColorZ : global_palette[(byte & 0x03)];
    palette[1] = global_palette[(byte & 0x0C) >> 2];
    palette[2] = global_palette[(byte & 0x30) >> 4];
    palette[3] = global_palette[(byte & 0xC0) >> 6];
}

static void
convert_line(unsigned *pixels, reg_t low, reg_t high, unsigned *palette,
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
    for (int x = 0; x < surface->w; x++)
        for (int y  = 0; y < surface->h / 2; y++) {
            p[y * surface->w + x] ^= p[(surface->h - y - 1) * surface->w - x];
            p[(surface->h - y - 1) * surface->w + x] ^= p[y * surface->w + x];
            p[y * surface->w + x] ^= p[(surface->h - y - 1) * surface->w - x];
        }
}

class TileMap {
    private:
        TileMap(const TileMap &rhs) { }
    public:
        TileMap(reg_t palette) {
            convert_palette(_palette, palette, false);
        }
        ~TileMap() {
        }

        void init(const reg_t *map_data, bool base) {
            _base = base;
            _map = create_surface(256, 64, false);

            SDL_LockSurface(_map.get());
            unsigned *pixels = (unsigned *)_map->pixels;
            for (unsigned y = 0; y < 8; y++) {
                for (unsigned x = 0; x < 32; x++) {
                    for (unsigned l = 0; l < 8; l++) {
                    reg_t low = *map_data++;
                    reg_t high = *map_data++;
                    convert_line(&pixels[(y * 8 + l) * 256 + x*8],
                                 low, high, _palette, false);
                    }
                }
            }
            SDL_UnlockSurface(_map.get());
        }

        SDL_Rect *tile(reg_t idx) {
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
        void init(unsigned idx, const reg_t *ram, const reg_t *oam) {
            reg_t lcdc = ram[CtrlReg::LCDC];
            _Y = oam[Oam::OamY];
            _X = oam[Oam::OamX];
            _pattern = oam[Oam::OamPattern];
            _flags = oam[Oam::OamFlags];

            unsigned palette[4];

            if (bit_isset(_flags, 4))
                convert_palette(palette, ram[CtrlReg::OBP1], true);
            else
                convert_palette(palette, ram[CtrlReg::OBP0], true);

            // XXX: Handle transparency

            _idx = idx;
            _rect.w = 8;
            if (bit_isset(lcdc, LCDCBits::OBJSize)) {
                _pattern &= 0xFE;
                _rect.h = 16;
            } else {
                _rect.h = 8;
            }
            _rect.x = SCREEN_X - 8 + _X;
            _rect.y = SCREEN_Y - 16 + _Y;

            _sprite = create_surface(_rect.w, _rect.h, true);

            SDL_LockSurface(_sprite.get());
            unsigned *pixels = (unsigned *)_sprite->pixels;

            // Handle first half of the sprite
            const reg_t *data = &ram[Mem::ObjTiles];
            data += _pattern * 16;
            for (unsigned i = 0; i < 8; i++) {
                convert_line(&pixels[i * _rect.w], *data++, *data++,
                    palette, false);
            }

            // Handle second block
            if (bit_isset(lcdc, LCDCBits::OBJSize)) {
                for (unsigned i = 0; i < 8; i++)
                    convert_line(&pixels[(i + 8) * _rect.w], *data++, *data++,
                        palette, false);
            }
            if (bit_isset(_flags, 5))
                flip_horizontal(_sprite.get());
            if (bit_isset(_flags, 6))
                flip_vertical(_sprite.get());
            SDL_UnlockSurface(_sprite.get());
        }

        void blit(SDL_Surface *window)
        {
            if (!visible())
                return;

            // XXX: Clip the sprite to the window.
            SDL_BlitSurface(_sprite.get(), NULL, window, &_rect);
        }

        bool visible(void) {
            if (_X == 0 || _X >= 168)
                return false;
            if (_Y == 0 || _Y >= 160)
                return false;
            return true;
        }

    private:

        reg_t _X;
        reg_t _Y;
        reg_t _pattern;
        reg_t _flags;
        surface_ptr _sprite;
        SDL_Rect _rect;
        reg_t _idx;
};

static surface_ptr
create_map(TileMap *tile_map, const reg_t *map)
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
blit_bg(SDL_Surface *window, SDL_Surface *bg, short scx, short scy)
{
    // Blit the tiles on to the window
    SDL_Rect clipped = { .x = scx, .y = scy, .h = 144, .w = 160 };
    short x_overscan = 0;
    short y_overscan = 0;
    if (clipped.x + clipped.w > 256) {
        x_overscan = clipped.x + clipped.w - 256;
        clipped.w -= x_overscan;
    }
    if (clipped.y + clipped.h > 256) {
        y_overscan = clipped.y + clipped.h - 256;
        clipped.h -= y_overscan;
    }

    SDL_Rect window_rect = { .x = SCREEN_X, .y = SCREEN_Y };
    SDL_BlitSurface(bg, &clipped, window, &window_rect);

    if (x_overscan > 0) {
        window_rect.x += clipped.w;
        window_rect.y = 64;
        clipped.x = 0;
        clipped.w = x_overscan;
        SDL_BlitSurface(bg, &clipped, window, &window_rect);

        if (y_overscan > 0) {
            window_rect.y += clipped.y;
            clipped.y = 0;
            clipped.h = y_overscan;
            SDL_BlitSurface(bg, &clipped, window, &window_rect);
        }
    }

    if (y_overscan > 0) {
        window_rect.x = SCREEN_X;
        window_rect.y = SCREEN_Y;
        clipped.x = scx;
        clipped.y = 0;
        clipped.h = y_overscan;
        clipped.w = 160 - x_overscan;
        SDL_BlitSurface(bg, &clipped, window, &window_rect);
    }

}

static void
blit_window(SDL_Surface *window, SDL_Surface *win, short wx, short wy)
{
    if (wx <= 166 && wy <= 144) {
        // XXX: Handle window offset better (wx < 7)
        SDL_Rect clipped = {};
        wx -= 7;
        clipped.h = 144 - wy;
        clipped.w = 160 - wx;
        wx += SCREEN_X;
        wy += SCREEN_Y;
        SDL_Rect window_rect = { .x = wx, .y = wy };
        SDL_BlitSurface(win, &clipped, window, &window_rect);
    }
}

SDLDisplay::SDLDisplay(void)
{
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
SDLDisplay::render(const reg_t *ram)
{
    reg_t lcdc = ram[CtrlReg::LCDC];

    if (!bit_isset(lcdc, LCDCBits::LCDEnabled))
        return;

    TileMap bg_tiles(ram[CtrlReg::BGP]);
    bg_tiles.init(&ram[0x8800], false);
    TileMap obj_tiles(ram[CtrlReg::BGP]);
    obj_tiles.init(&ram[0x8000], true);

    TileMap *tiles;
    if (bit_isset(lcdc, LCDCBits::BGTileData))
        tiles = &obj_tiles;
    else
        tiles = &bg_tiles;

    if (bit_isset(lcdc, LCDCBits::BGDisplay)) {
        const reg_t *map = bit_isset(lcdc, LCDCBits::BGTileMap) ?
            &ram[0x9C00] : &ram[0x9800];
        surface_ptr bg(create_map(tiles, map));
        short scx = ram[CtrlReg::SCX];
        short scy = ram[CtrlReg::SCY];
        blit_bg(_window.get(), bg.get(), scx, scy);
    }

    if (bit_isset(lcdc, LCDCBits::WindowDisplay)) {
        const reg_t *map = bit_isset(lcdc, LCDCBits::WindowTileMap) ?
            &ram[0x9C00] : &ram[0x9800];
        surface_ptr win(create_map(tiles, map));
        short wx = ram[CtrlReg::WX];
        short wy = ram[CtrlReg::WY];
        blit_window(_window.get(), win.get(), wx, wy);
    }

    const reg_t *oam = &ram[0xFE00];
    Sprite sprites[40];
    for (unsigned i = 0; i < 40; i++) {
        sprites[i].init(i, ram, oam);
        oam += 4;
    }

    if (bit_isset(lcdc, LCDCBits::OBJDisplay))
        for (unsigned i = 40; i > 0; i--)
            sprites[i-1].blit(_window.get());

    // Show the screen
    SDL_Flip(_window.get());
}

