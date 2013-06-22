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
#pragma once

#include "types.h"

namespace DMG {

class Device {
public:
    virtual ~Device(void) { };
    virtual void tick(unsigned cycles) = 0;
    virtual void reset(void) = 0;
    virtual bool valid(addr_t addr) = 0;
    virtual void write(addr_t addr, byte_t value) = 0;
    virtual byte_t read(addr_t addr) = 0;
};

class MemoryBus {
public:
    MemoryBus(void) {}
    ~MemoryBus(void) {}

    void reset(void) {
        for_each(_devs.begin(), _devs.end(), [](Device *map){map->reset();});
    }

    void add_device(Device *map){
        _devs.push_front(map);
    }

    void remove_device(Device *map) {
        _devs.remove(map);
    }

    void step(void) {
        for_each(_devs.begin(), _devs.end(),
            [&](Device *map){map->tick(_cycles);});
    }

    inline void irq(Interrupt i) {
        switch (i) {
            case Interrupt::Timeout:
            case Interrupt::VBlank: {
                byte_t ifreg = read(CtrlReg::IF);
                bit_set(ifreg, i, true);
                write(CtrlReg::IF, ifreg);
                break;
            }
            default:
                std::cout << "Ignoring interrupt: " << i << std::endl;
                break;
        }
    };

    inline void set_ticks(unsigned cycles) {
        _cycles = cycles;
    }

    inline void write(addr_t addr, byte_t value) {
        switch (addr) {
        case CtrlReg::DMG_RESET: {
            _devs.remove(find(0));
            break;
        }
        case CtrlReg::DMA: {
            addr_t dest_addr = 0xFE00;
            auto dest = find(0xFE00);
            addr_t src_addr = (addr_t)value << 8;
            auto src = find(src_addr);
            for (unsigned i = 0; i < 160; i++)
                dest->write(dest_addr + i, src->read(src_addr + i));
            break;
        }
        default:
            find(addr)->write(addr, value);
            break;
        }
    }

    inline byte_t read(addr_t addr) {
        return find(addr)->read(addr);
    }

private:

    unsigned _cycles;

    inline Device *find(addr_t addr) {
        auto it = std::find_if(_devs.begin(), _devs.end(),
            [=](Device *map) -> bool { return map->valid(addr); });
        if (it == _devs.end())
            throw MemException(addr);
        return *it;
    }

    std::list<Device *> _devs;
};

};

