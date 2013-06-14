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

#include "SDL/sdl.h"

#include "cpu.h"
#include "graphics.h"

using namespace DMG;
enum Color {
    Color0 = 0xffffffff,
    Color1 = 0xffbbbbbb,
    Color2 = 0xff666666,
    Color3 = 0xff000000,
    ColorZ = 0x00000000,
};

unsigned global_palette[] = { Color0, Color1, Color2, Color3 };

#define SCREEN_X (256 + 64)
#define SCREEN_Y 64

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
        TileMap(reg_t palette): _map(NULL) {
            convert_palette(_palette, palette, false);
        }
        ~TileMap() {
            if (_map) {
                SDL_FreeSurface(_map);
                _map = NULL;
            }
        }

        void init(const reg_t *map_data, bool base) {
            if (_map != NULL)
                SDL_FreeSurface(_map);
            _base = base;
            _map = SDL_CreateRGBSurface(
                SDL_SWSURFACE, 256, 64, 32, 0x000000ff, 0x0000ff00,
                0x00ff0000, 0xff000000);

            SDL_LockSurface(_map);
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
            SDL_UnlockSurface(_map);
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

        SDL_Surface *_map;
        SDL_Rect _tile_rect;
        unsigned _palette[4];
        bool _base;
};

class Sprite {
    private:
        Sprite(const Sprite &rhs) { }
    public:
        Sprite(void): _sprite(NULL) { }
        ~Sprite(void) {
            if (_sprite) {
                SDL_FreeSurface(_sprite);
                _sprite = NULL;
            }
        }
        void init(unsigned idx, const reg_t *ram, const reg_t *oam) {
            reg_t lcdc = ram[CtrlReg::LCDC];
            memcpy(_bytes, oam, sizeof(_bytes));

            unsigned palette[4];

            if (bit_isset(_bytes[Oam::OamFlags], 4))
                convert_palette(palette, ram[CtrlReg::OBP1], true);
            else
                convert_palette(palette, ram[CtrlReg::OBP0], true);

            // XXX: Handle transparency

            _idx = idx;
            _rect.w = 8;
            if (bit_isset(lcdc, LCDCBits::OBJSize)) {
                _bytes[Oam::OamPattern] &= 0xFE;
                _rect.h = 16;
            } else {
                _rect.h = 8;
            }
            _rect.x = SCREEN_X - 8 + _bytes[Oam::OamX];
            _rect.y = SCREEN_Y - 16 + _bytes[Oam::OamY];

            _sprite = SDL_CreateRGBSurface(SDL_SWSURFACE, _rect.w, _rect.h,
                32, 0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);

            SDL_LockSurface(_sprite);
            unsigned *pixels = (unsigned *)_sprite->pixels;

            // Handle first half of the sprite
            const reg_t *data = &ram[Mem::ObjTiles];
            data += _bytes[Oam::OamPattern] * 16;
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
            if (bit_isset(_bytes[Oam::OamFlags], 5))
                flip_horizontal(_sprite);
            if (bit_isset(_bytes[Oam::OamFlags], 6))
                flip_vertical(_sprite);
            SDL_UnlockSurface(_sprite);
        }

        bool visible(void) {
            if (_bytes[Oam::OamX] == 0 || _bytes[Oam::OamX] >= 168)
                return false;
            if (_bytes[Oam::OamY] == 0 || _bytes[Oam::OamY] >= 160)
                return false;
            return true;
        }

        SDL_Rect *rect(void) {
            return &_rect;
        }

        SDL_Surface *surface(void) {
            return _sprite;
        }

    private:

        reg_t _bytes[4];
        SDL_Surface *_sprite;
        SDL_Rect _rect;
        reg_t _idx;
};

static void
blit_tiles(SDL_Surface *_window, TileMap *tile_map, const reg_t *ram, bool bg)
{
    reg_t lcdc = ram[CtrlReg::LCDC];

    const reg_t *map = NULL;
    if (bg)
        map = bit_isset(lcdc, LCDCBits::BGTileMap) ?
            &ram[0x9C00] : &ram[0x9800];
    else
        map = bit_isset(lcdc, LCDCBits::WindowTileMap) ?
            &ram[0x9C00] : &ram[0x9800];

    // Render the tiles
    SDL_Surface *tiles = SDL_CreateRGBSurface(
        SDL_SWSURFACE, 256, 256, 32, 0x000000ff, 0x0000ff00,
        0x00ff0000, 0);
    SDL_LockSurface(tiles);
    memset(tiles->pixels, 0x00, 256*256*4);
    SDL_UnlockSurface(tiles);

    if (bit_isset(lcdc, LCDCBits::LCDEnabled)) {
        for (unsigned y = 0; y < 32; y++) {
            for (unsigned x = 0; x < 32; x++) {
                SDL_Rect rect = {};
                rect.y = y * 8;
                rect.x = x * 8;
                SDL_BlitSurface(
                    tile_map->_map,
                    tile_map->tile(map[y * 32 + x]),
                    tiles,
                    &rect);
            }
        }
    }

    if (bg) {
        reg_t scx = ram[CtrlReg::SCX];
        reg_t scy = ram[CtrlReg::SCY];
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
        SDL_BlitSurface(tiles, &clipped, _window, &window_rect);

        if (x_overscan > 0) {
            window_rect.x += clipped.w;
            window_rect.y = 64;
            clipped.x = 0;
            clipped.w = x_overscan;
            SDL_BlitSurface(tiles, &clipped, _window, &window_rect);

            if (y_overscan > 0) {
                window_rect.y += clipped.y;
                clipped.y = 0;
                clipped.h = y_overscan;
                SDL_BlitSurface(tiles, &clipped, _window, &window_rect);
            }
        }

        if (y_overscan > 0) {
            window_rect.x = SCREEN_X;
            window_rect.y = SCREEN_Y;
            clipped.x = scx;
            clipped.y = 0;
            clipped.h = y_overscan;
            clipped.w = 160 - x_overscan;
            SDL_BlitSurface(tiles, &clipped, _window, &window_rect);
        }
    } else {
        short wx = ram[CtrlReg::WX];
        short wy = ram[CtrlReg::WY];
        if (wx <= 166) {
            // XXX: Handle window offset better (wx < 7)
            SDL_Rect clipped = {};
            wx -= 7;
            clipped.h = 144 - wy;
            clipped.w = 160 - wx;
            wx += SCREEN_X;
            wy += SCREEN_Y;
            SDL_Rect window_rect = { .x = wx, .y = wy };
            SDL_BlitSurface(tiles, &clipped, _window, &window_rect);
        }
    }

    SDL_FreeSurface(tiles);
}

RenderCallable::RenderCallable(SDL_Surface *window):_window(window)
{
}

void
RenderCallable::operator ()(const reg_t *ram)
{
    reg_t lcdc = ram[CtrlReg::LCDC];

    if (!bit_isset(lcdc, LCDCBits::LCDEnabled))
        return;

    TileMap bg_tiles(ram[CtrlReg::BGP]);
    bg_tiles.init(&ram[0x8800], false);
    TileMap obj_tiles(ram[CtrlReg::BGP]);
    obj_tiles.init(&ram[0x8000], true);

    // Show the tile maps (for debugging)
    if (true) {
        SDL_Rect rect = { .x = 0, .y = 0 };
        SDL_BlitSurface(obj_tiles._map, NULL, _window, &rect);
        rect.y += 64;
        SDL_Rect bg = { .y = 32, .h = 32, .w = 256 };
        SDL_BlitSurface(bg_tiles._map, &bg, _window, &rect);
    }

    TileMap *tiles;
    if (bit_isset(lcdc, LCDCBits::BGTileData))
        tiles = &obj_tiles;
    else
        tiles = &bg_tiles;

    if (bit_isset(lcdc, LCDCBits::BGDisplay))
        blit_tiles(_window, tiles, ram, true);

    if (bit_isset(lcdc, LCDCBits::WindowDisplay))
        blit_tiles(_window, tiles, ram, false);

    const reg_t *oam = &ram[0xFE00];
    Sprite sprites[40];
    for (unsigned i = 0; i < 40; i++) {
        sprites[i].init(i, ram, oam);
        oam += 4;
    }

    unsigned i = 40;
    while (i > 0 && bit_isset(lcdc, LCDCBits::OBJDisplay)) {
        if (sprites[i].visible())
            SDL_BlitSurface(sprites[i].surface(), NULL,
                            _window, sprites[i].rect());
        i--;
    }

    // Show the screen
    SDL_Flip(_window);
}

