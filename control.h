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

#pragma once

#include "SDL/sdl.h"

#include "cpu.h"

namespace DMG {

enum KeysReg {
    ButtonsSelect = 5,
    ArrowSelect   = 4,
    DownOrSelect  = 3,
    UpOrSelect    = 2,
    LeftOrB       = 1,
    RightOrA      = 0,
};

enum class GBKey {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Right = 4,
    Left = 5,
    Up = 6,
    Down = 7,
    Size = 8,
};
static inline byte_t key_value(GBKey key) {
    return static_cast<std::underlying_type<GBKey>::type>(key);
}

class SDLController: public Device  {
public:
    SDLController(void);
    virtual ~SDLController(void);

    void handle_key(const SDL_Event *event);

    virtual void reset() {
        _value = 0x00;
    }
    virtual bool valid(addr_t addr) {
        return (addr == CtrlReg::KEYS);
    }
    virtual byte_t *direct(addr_t addr) {
        return NULL;
    }
    virtual void write(addr_t addr, byte_t arg) {
        // XXX: We need to OR the keys if both are set.
        if (bit_isset(arg, ButtonsSelect))
            arg = (arg & 0xF0) | ((_keys & 0xF0) >> 4);
        if (bit_isset(arg, ArrowSelect))
            arg = (arg & 0xF0) | (_keys & 0x0F);
        _value = arg;
    }
    virtual byte_t read(addr_t addr) {
        return _value;
    }

private:

    SDLKey _key_map[8];
    byte_t _keys;
    byte_t _value;
};

};
