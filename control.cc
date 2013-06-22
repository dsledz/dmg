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

#include "control.h"

using namespace DMG;

SDLController::SDLController(MemoryBus *bus): _bus(bus), _keys(0xFF)
{
    _key_map[key_value(GBKey::A)] =      SDLK_a;
    _key_map[key_value(GBKey::B)] =      SDLK_s;
    _key_map[key_value(GBKey::Select)] = SDLK_z;
    _key_map[key_value(GBKey::Start)] =  SDLK_x;
    _key_map[key_value(GBKey::Right)] =  SDLK_RIGHT;
    _key_map[key_value(GBKey::Left)] =   SDLK_LEFT;
    _key_map[key_value(GBKey::Up)] =     SDLK_UP;
    _key_map[key_value(GBKey::Down)] =   SDLK_DOWN;
    _bus->add_device(this);
}

SDLController::~SDLController(void)
{
    _bus->remove_device(this);
}

void
SDLController::handle_key(const SDL_Event *event)
{
    // See if the key press was one of our buttons.
    // XXX: This is slow.
    for (unsigned i = 0; i < key_value(GBKey::Size); i++) {
        if (event->key.keysym.sym == _key_map[i]) {
            bit_set(_keys, i, event->type == SDL_KEYUP);
            break;
        }
    }
    _bus->irq(Interrupt::Joypad);
}

