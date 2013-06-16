/*
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
 * Memory bank Controller
 */
#pragma once

#include "cpu.h"
#include <string>
#include <algorithm>

namespace DMG {

enum Cartridge {
    RomOnly = 0x00,
    MBC1O   = 0x01,
    MBC1R   = 0x02,
    MBC1RB  = 0x03,
};

class MBC1: public MBC {
    public:
        MBC1(void): _rom_bank(1) {
            _rom.resize(0);
        }
        virtual ~MBC1(void) { }

        virtual void load(const std::string &name);

        virtual void reset(void);
        virtual bool valid(addr_t addr);
        virtual void write(addr_t addr, byte_t value);
        virtual byte_t read(addr_t addr);
        virtual byte_t *direct(addr_t addr);

    private:
        std::string _name;
        Cartridge   _type;
        unsigned    _rom_bank;
        unsigned    _rom_size;
        unsigned    _rom_high;
        unsigned    _rom_low;
        bvec        _rom;
        unsigned    _ram_bank;
        unsigned    _ram_size;
        unsigned    _ram_high;
        unsigned    _ram_low;
        bvec        _ram;
};

};
