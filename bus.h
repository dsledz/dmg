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

/**
 * Drive all devices from a common crystal.  This code is OS X specific
 */
#include <mach/mach.h>
#include <mach/mach_time.h>

class Crystal {
    public:
        Crystal(int hz): _hz(hz) {
            _clock = mach_absolute_time();
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);
            _hz_per_nano = ((double)(_hz) * timebase.numer) /
                (1000000000 * timebase.denom);
        }
        ~Crystal(void) { }

        void reset(void) {
            _clock = mach_absolute_time();
        }

        /* Return the number of ticks since last call */
        unsigned get_ticks(void) {

            uint64_t now = mach_absolute_time();
            uint64_t delta = (now - _clock);

            _clock = now;
            return delta * _hz_per_nano;
        }

    private:
        unsigned _hz;
        double _hz_per_nano;
        uint64_t _clock;
};

class MemoryBus {
public:
    MemoryBus(void): _clock(4194304) {}
    ~MemoryBus(void) {}

    void reset(void) {
        for_each(_devs.begin(), _devs.end(), [](Device *map){map->reset();});
        _clock.reset();
    }

    void add_device(Device *map){
        _devs.push_front(map);
    }

    void remove_device(Device *map) {
        _devs.remove(map);
    }

    void step(void) {
        int avail;
        while ((avail += _clock.get_ticks()) < 10000) {
            struct timespec t = { .tv_sec = 0, .tv_nsec = 100000000 };
            nanosleep(&t, NULL);
        }
        _cycles = 0;
        while (avail > 0) {
            for_each(_devs.begin(), _devs.end(),
                     [&](Device *map){map->tick(_cycles);});
            avail -= _cycles;
        }
    }

    inline void irq(Interrupt i) {
        byte_t ifreg = read(CtrlReg::IF);
        bit_set(ifreg, i, true);
        write(CtrlReg::IF, ifreg);
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

    Crystal _clock;
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

