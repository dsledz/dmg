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

#include "cpu.h"

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
    NR33 = 0xFF1E,
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
    S4S02 = 7,
    S3S02 = 6,
    S2S02 = 5,
    S1S02 = 4,
    S4S01 = 3,
    S3S01 = 2,
    S2S01 = 1,
    S1S01 = 0,
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
};

struct Sweep {
    int direction;
    int shift;
    unsigned len;
    unsigned count;
};

struct Channel {
    // Internal State
    bool     on;      // Enabled/Disabled
    byte_t    channel; // Channel ID
    Envelope env;     // Envelope calculation
    Sweep    sweep;   // Sweep calculation
    bool     loop;
    sbyte_t   signal;
    int      freq;   // Freq (In GB Hz)
    unsigned period; // Period (in SAMPLES)
    unsigned pos;    // Position within the period
    unsigned count;  // Current number of sample
    unsigned len;    // Total number of samples
};

class SDLAudio: public Device {
    public:
        SDLAudio(void);
        ~SDLAudio(void);

        virtual void reset(void);

        virtual bool valid(addr_t addr) {
            return (addr >= SoundReg::NR10 && addr <= SoundReg::NRLast);
        };
        virtual void write(addr_t addr, byte_t value) {
            set(addr, value);
        };
        virtual byte_t read(addr_t addr) {
            return _mem[addr - SoundReg::NR10];
        }

        void mix(Uint8 *stream, int len);
    private:

        inline byte_t & rget(enum SoundReg reg) {
            return _mem[static_cast<addr_t>(reg - SoundReg::NR10)];
        }
        inline byte_t & rget(addr_t addr) {
            return _mem[addr - SoundReg::NR10];
        }

        void set(addr_t addr, byte_t arg);
        void generate(Channel &channel, bvec &data, int len);
        void start_sound(Channel &channel);
        sbyte_t sample_sound(Channel &channel);

        void sound4(bvec &data, int len);

        Channel _Snd1;
        Channel _Snd2;
        Channel _Snd3;
        Channel _Snd4;

        bvec _mem;
};

};
