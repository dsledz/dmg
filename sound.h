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
/**
 * SDL Sound driver
 */

#pragma once

#include "SDL/sdl.h"

#include "types.h"
#include "bus.h"

namespace DMG {

enum SoundReg {
    NR10 = 0xFF10,
    NR11 = 0xFF11,
    NR12 = 0xFF12,
    NR13 = 0xFF13,
    NR14 = 0xFF14,
    NR21 = 0xFF16,
    NR22 = 0xFF17,
    NR23 = 0xFF18,
    NR24 = 0xFF19,
    NR30 = 0xFF1A,
    NR31 = 0xFF1B,
    NR32 = 0xFF1C,
    NR33 = 0xFF1D,
    NR34 = 0xFF1E,
    NR41 = 0xFF20,
    NR42 = 0xFF21,
    NR43 = 0xFF22,
    NR44 = 0xFF23,
    NR50 = 0xFF24,
    NR51 = 0xFF25,
    NR52 = 0xFF26,
    NRLast = 0xFF3F,
};

enum class NR51Bits {
    S4SO2 = 7,
    S3SO2 = 6,
    S2SO2 = 5,
    S1SO2 = 4,
    S4SO1 = 3,
    S3SO1 = 2,
    S2SO1 = 1,
    S1SO1 = 0,
};

enum class NR52Bits {
    AllOn     = 7,
    Sound4On  = 3,
    Sound3On  = 2,
    Sound2On  = 1,
    Sound1On  = 0,
};

struct Envelope {
    int value;
    int direction;
    unsigned len;
    unsigned count;

    inline void shift(void) {
        if (len > 0) {
            count++;
            if (count > len) {
                count = 0;
                value += direction;
                if (value < 0)
                    value = 0;
                else if (value > 15)
                    value = 15;
            }
        }
    }
};

struct Sweep {
    int direction;
    int shift;
    unsigned len;
    unsigned count;
};

struct Channel {
    // Internal State
    bool      on;      // Enabled/Disabled
    byte_t    channel; // Channel ID
    Envelope  env;     // Envelope calculation
    Sweep     sweep;   // Sweep calculation
    int       level;   // Level for sample
    bool      loop;
    sbyte_t   signal;
    int       offset;
    int       freq;   // Freq (In GB Hz)
    byte_t    duty;
    unsigned  dutycount;
    unsigned  period; // Period (in SAMPLES)
    unsigned  edge;   // edge trigger
    unsigned  pos;    // Position within the period
    unsigned  count;  // Current number of sample
    unsigned  len;    // Total number of samples
    unsigned  value;  // polynominal value
    bool      step;
    float     ratio;
    unsigned  shift;
};

class SDLAudio: public Device {
    public:
        SDLAudio(MemoryBus *bus);
        ~SDLAudio(void);

        virtual void tick(unsigned cycles) { }
        virtual void reset(void);

        virtual void write(addr_t addr, byte_t value) {
            set(addr, value);
        };
        virtual byte_t read(addr_t addr);

        void set_volume(int volume) {
            if (volume > 10)
                _volume = 10;
            else if (volume < 0)
                _volume = 0;
            else
                _volume = volume;
        }
        int get_volume(void) {
            return _volume;
        }

        void mix(Uint8 *stream, int len);

    private:
        inline byte_t & rget(addr_t addr) {
            return _mem[addr - SoundReg::NR10];
        }

        inline sbyte_t get_sample(int offset) {
            byte_t s = _mem[0x20 + offset / 2];
            if ((offset & 0x01) == 0)
                s >>= 4;
            return (s & 0xF);
        }

        void set(addr_t addr, byte_t arg);
        void start_sound1(Channel &channel);
        void start_sound2(Channel &channel);
        void start_sound3(Channel &channel);
        void start_sound4(Channel &channel);
        sbyte_t sound1(Channel &channel);
        sbyte_t sound2(Channel &channel);
        sbyte_t sound3(Channel &channel);
        sbyte_t sound4(Channel &channel);

        void sound4(bvec &data, int len);

        MemoryBus *_bus;

        Channel _Snd1;
        Channel _Snd2;
        Channel _Snd3;
        Channel _Snd4;

        bvec _mem;

        unsigned _volume;
};

};
