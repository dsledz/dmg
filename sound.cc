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

#include "sound.h"

using namespace DMG;

//#define SAMPLES_PER_SEC 11025
#define SAMPLES_PER_SEC 44100

static void
mixaudio(void *obj, Uint8 *stream, int len)
{
    SDLAudio *sound = static_cast<SDLAudio *>(obj);
    sound->mix(stream, len);
}

struct AudioLock {
    AudioLock() { SDL_LockAudio(); };
    ~AudioLock() { SDL_UnlockAudio(); };
};

SDLAudio::SDLAudio(MemoryBus *bus): _bus(bus)
{
    _mem.resize(0x30);

    _volume = 1;
    SDL_AudioSpec fmt = { };

    fmt.freq = SAMPLES_PER_SEC;
    fmt.format = AUDIO_S8;
    fmt.channels = 2;
    fmt.samples = 512;
    fmt.callback = mixaudio;
    fmt.userdata = static_cast<void *>(this);

    if (SDL_OpenAudio(&fmt, NULL) < 0) {
        std::cout << "Failed to open audio" << std::endl;
    }

    SDL_PauseAudio(0);
    _bus->add_device(this);
    _bus->add_port(0xFF10, 12, this);
    _bus->add_port(0xFF20, 11, this);
}

SDLAudio::~SDLAudio(void)
{
    _bus->remove_device(this);
    AudioLock lock;
    SDL_PauseAudio(1);
}

void
SDLAudio::reset(void)
{
    memset(&_mem[0], 0, _mem.size());
    set(SoundReg::NR10, 0x80);
    set(SoundReg::NR11, 0xBF);
    set(SoundReg::NR12, 0xF3);
    set(SoundReg::NR14, 0xBF);
    set(SoundReg::NR21, 0x3F);
    set(SoundReg::NR22, 0x00);
    set(SoundReg::NR24, 0xBF);
    set(SoundReg::NR30, 0x7F);
    set(SoundReg::NR31, 0xFF);
    set(SoundReg::NR32, 0x9F);
    set(SoundReg::NR33, 0xBF);
    set(SoundReg::NR41, 0xFF);
    set(SoundReg::NR42, 0x00);
    set(SoundReg::NR43, 0x00);
    set(SoundReg::NR44, 0xBF);
    set(SoundReg::NR50, 0x77);
    set(SoundReg::NR51, 0xF3);
    set(SoundReg::NR52, 0xF1);
    _Snd1.on = false;
    _Snd2.on = false;
    _Snd3.on = false;
    _Snd4.on = false;
}

byte_t
SDLAudio::read(addr_t addr)
{
    AudioLock lock;
    byte_t value = rget(addr);
    switch (addr) {
    case SoundReg::NR52:
        bit_set(value, NR52Bits::Sound1On, _Snd1.on);
        bit_set(value, NR52Bits::Sound2On, _Snd2.on);
        bit_set(value, NR52Bits::Sound3On, _Snd3.on);
        bit_set(value, NR52Bits::Sound4On, _Snd4.on);
        break;
    }
    return value;
}

void
SDLAudio::set(addr_t addr, byte_t arg)
{
    AudioLock lock;
    switch (addr) {
    case SoundReg::NR10:
        _Snd1.sweep.direction = (arg & 0x08) != 0;
        _Snd1.sweep.shift = arg & 0x07;
        _Snd1.sweep.len = ((arg & 0x70) * SAMPLES_PER_SEC) / 128;
        break;
    case SoundReg::NR11:
        _Snd1.len = ((64 - (arg & 0x3F)) * SAMPLES_PER_SEC) / 256;
        _Snd1.duty = (arg & 0xC0) >> 6;
        break;
    case SoundReg::NR12:
        _Snd1.env.direction = (arg & 0x08) != 0;
        _Snd1.env.value = (arg & 0xF0) >> 4;
        _Snd1.env.len = ((arg & 0x07) * SAMPLES_PER_SEC) / 64;
        break;
    case SoundReg::NR13:
        _Snd1.freq = (_Snd1.freq & 0x0700) | arg;
        break;
    case SoundReg::NR14:
        _Snd1.freq = (_Snd1.freq & 0xff) | ((arg & 0x7) << 8);
        _Snd1.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound1(_Snd1);
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound1On, 1);
        }
        break;
    case SoundReg::NR21:
        _Snd2.len = ((64 - (arg & 0x3F)) * SAMPLES_PER_SEC) / 256;
        _Snd2.duty = (arg & 0xC0) >> 6;
        break;
    case SoundReg::NR22:
        _Snd2.env.direction = (arg & 0x08) != 0;
        _Snd2.env.value = (arg & 0xF0) >> 4;
        _Snd2.env.len = ((arg & 0x07) * SAMPLES_PER_SEC) / 64;
        break;
    case SoundReg::NR23:
        _Snd2.freq = (_Snd2.freq & 0x0700) | arg;
        break;
    case SoundReg::NR24:
        _Snd2.freq = (_Snd2.freq & 0xff) | ((word_t)(arg & 0x7) << 8);
        _Snd2.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound2(_Snd2);
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound2On, 1);
        }
        break;
    case SoundReg::NR30:
        // XXX: Start sampling
        _Snd3.on = bit_isset(arg, 8);
        break;
    case SoundReg::NR31:
        _Snd3.len = ((256 - arg) * SAMPLES_PER_SEC) / 256;
        break;
    case SoundReg::NR32:
        _Snd3.level = (arg & 0x60) >> 5;
        break;
    case SoundReg::NR33:
        _Snd3.freq = (_Snd3.freq & 0x0700) | arg;
        break;
    case SoundReg::NR34:
        _Snd3.freq = (_Snd3.freq & 0xff) | ((word_t)(arg & 0x07) << 8);
        _Snd3.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound3(_Snd3);
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound3On, 1);
        }
        break;
    case SoundReg::NR41:
        _Snd3.len = ((64 - (arg & 0x3F)) * SAMPLES_PER_SEC) / 256;
        break;
    case SoundReg::NR42:
        _Snd4.env.direction = (arg & 0x08) != 0;
        _Snd4.env.value = (arg & 0xF0) >> 4;
        _Snd4.env.len = ((arg & 0x07) * SAMPLES_PER_SEC) / 64;
        break;
    case SoundReg::NR43: {
        // XXX: Polynominal
        _Snd4.step = bit_isset(arg, 3);
        _Snd4.ratio = (arg & 0x07) ? (arg & 0x07) : 0.5f;
        _Snd4.shift = 1 << ((arg & 0xF0) >> 4);
        break;
    }
    case SoundReg::NR44:
        _Snd4.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound4(_Snd4);
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound4On, 1);
        }
    case SoundReg::NR50:
        break;
    };
    rget(addr) = arg;
}

void
SDLAudio::mix(Uint8 *stream, int len)
{
    bvec buf;
    short left = 0;
    short right = 0;

    if (!bit_isset(rget(SoundReg::NR52), NR52Bits::AllOn))
        return;

    char *samples = (char *)stream;
    for (int i = 0; i < len; i += 2) {
        left = 0;
        right = 0;
        if (_Snd1.on) {
            sbyte_t sample = sound1(_Snd1);
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S1SO1))
                left += sample;
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S1SO2))
                right += sample;
        }
        if (_Snd2.on) {
            sbyte_t sample = sound2(_Snd2);
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S2SO1))
                left += sample;
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S2SO2))
                right += sample;
        }
        if (_Snd3.on) {
            sbyte_t sample = sound3(_Snd3);
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S3SO1))
                left += sample;
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S3SO2))
                right += sample;
        }
        if (_Snd4.on) {
            sbyte_t sample = sound4(_Snd4);
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S4SO1))
                left += sample;
            if (bit_isset(rget(SoundReg::NR51), NR51Bits::S4SO2))
                right += sample;
        }
        left  *= ((rget(SoundReg::NR50) & 0x07) >> 0);
        right *= ((rget(SoundReg::NR50) & 0x70) >> 4);
        *samples++ = left;
        *samples++ = right;
    }

}
static unsigned
edge(Channel &channel)
{
    switch (channel.duty) {
    case 0: return channel.period / 8;
    case 1: return channel.period / 4;
    case 2: return channel.period / 2;
    case 3: return (channel.period * 3) / 4;
    }
    return channel.period;
}

void
SDLAudio::start_sound1(Channel &channel)
{
    channel.period = 1.0f / (131071.0f / (2048 - channel.freq)) * SAMPLES_PER_SEC;
    channel.edge = edge(channel);
    channel.pos = 0;
    channel.count = 0;
    channel.signal = 1;
    channel.offset = 0;
    channel.on = true;
}

void
SDLAudio::start_sound2(Channel &channel)
{
    channel.period = 1.0f / (131071.0f / (2048 - channel.freq)) * SAMPLES_PER_SEC;
    channel.edge = edge(channel);
    channel.pos = 0;
    channel.count = 0;
    channel.signal = 1;
    channel.offset = 0;
    channel.on = true;
}

void
SDLAudio::start_sound3(Channel &channel)
{
    channel.period = 1.0f / (65536.0f/ (2048 - channel.freq)) * SAMPLES_PER_SEC;
    channel.pos = 0;
    channel.count = 0;
    channel.signal = 1;
    channel.offset = 0;
    channel.duty = 1;
    channel.dutycount = 0;
    channel.on = true;
}

void
SDLAudio::start_sound4(Channel &channel)
{
    channel.period =  1.0f / (524288.0f / channel.ratio / channel.shift) *
        SAMPLES_PER_SEC;
    channel.pos = 0;
    channel.count = 0;
    channel.signal = rand();
    channel.offset = 0;
    channel.value = 0x7fff;
    channel.on = true;
}


sbyte_t
SDLAudio::sound1(Channel &channel)
{
    sbyte_t sample = channel.signal * channel.env.value;

    // Cycle the waveform according to the duty
    channel.pos++;
    if (channel.pos == channel.edge) {
        channel.signal = -channel.signal;
    } else if (channel.pos > channel.period) {
        channel.pos = 0;
        channel.signal = -channel.signal;
    }

    if (channel.len > 0 && !channel.loop) {
        channel.count++;
        if (channel.count >= channel.len) {
            channel.on = false;
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound1On, 0);
        }
    }

    channel.env.shift();

    if (channel.sweep.len > 0) {
        channel.sweep.count++;
        if (channel.sweep.count > channel.sweep.len) {
            channel.sweep.count = 0;
            if (channel.sweep.direction > 0) {
                channel.freq -= channel.freq / (1 << channel.sweep.shift);
                if (channel.freq < 0) {
                    channel.on = false;
                }
            } else {
                channel.freq += channel.freq / (1 << channel.sweep.shift);
                if (channel.freq > 2048)
                    channel.freq = 2047;
            }
            channel.period = 1.0f / (131071 / (2048 - channel.freq)) *
                SAMPLES_PER_SEC;
            channel.edge = edge(channel);
        }
    }

    return sample;
}

sbyte_t
SDLAudio::sound2(Channel &channel)
{
    sbyte_t sample = channel.signal * channel.env.value;
    // Separate the period into 8 pieces.
    channel.pos++;
    if (channel.pos == channel.edge) {
        channel.signal = -channel.signal;
    } else if (channel.pos > channel.period) {
        channel.pos = 0;
        channel.signal = -channel.signal;
    }

    if (channel.len > 0 && !channel.loop) {
        channel.count++;
        if (channel.count >= channel.len) {
            channel.on = false;
            // bit_set(_NR52, NR52Bits::Sound1On, 0);
        }
    }

    channel.env.shift();

    return sample;
}

sbyte_t
SDLAudio::sound3(Channel &channel)
{

    // Read the current sample
    // Scale the sample based on the level
    // See if we need to shift the sample.
    // The waveform is 32 samples long played over the period.
    sbyte_t sample = get_sample(channel.offset) - 8;

    switch (channel.level) {
    case 0:
        sample = 0;
        break;
    case 2:
        sample >>= 1;
        break;
    case 3:
        sample >>= 2;
        break;
    }

    channel.pos++;
    if (channel.pos >= (channel.period / 32) + channel.duty) {
        channel.pos = 0;
        if (channel.dutycount == channel.period % 32) {
            channel.duty--;
        }
        channel.dutycount++;
        channel.offset++;
        if (channel.offset > 31) {
            channel.offset = 0;
            channel.duty = 1;
            channel.dutycount = 0;
        }
    }

    if (channel.len > 0 && !channel.loop) {
        channel.count++;
        if (channel.count >= channel.len) {
            channel.on = false;
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound3On, 0);
        }
    }

    return sample;
}

sbyte_t
SDLAudio::sound4(Channel &channel)
{
    sbyte_t sample = channel.signal & channel.env.value;

    channel.pos++;
    if ((channel.pos == channel.period / 2) ||
        (channel.pos >= channel.period)) {
        word_t mask = (((channel.value & 0x02) >> 1) ^ (channel.value & 0x01));
        mask <<= (channel.step ? 6 : 14);
        channel.value >>= 1;
        channel.value |= mask;
        channel.value &= (channel.step ? 0x7f : 0xffff);
        channel.signal = (sbyte_t)channel.value;
    }
    if (channel.pos >= channel.period)
        channel.pos = 0;

    if (channel.len > 0 && !channel.loop) {
        channel.count++;
        if (channel.count >= channel.len) {
            channel.on = false;
            bit_set(rget(SoundReg::NR52), NR52Bits::Sound4On, 0);
        }
    }

    channel.env.shift();

    return sample;
}

