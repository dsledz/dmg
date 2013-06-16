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

SDLAudio::SDLAudio(void)
{
    SDL_AudioSpec fmt = { };

    fmt.freq = SAMPLES_PER_SEC;
    fmt.format = AUDIO_S8;
    fmt.channels = 1;
    fmt.samples = 512;        /* A good value for games */
    fmt.callback = mixaudio;
    fmt.userdata = static_cast<void *>(this);

    if (SDL_OpenAudio(&fmt, NULL) < 0) {
        std::cout << "Failed to open audio" << std::endl;
    }
    SDL_PauseAudio(0);
}

SDLAudio::~SDLAudio(void)
{
    AudioLock lock;
    SDL_PauseAudio(1);
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
        // XXX: duty
        break;
    case SoundReg::NR12:
        _Snd1.env.direction = (arg & 0x08) != 0;
        _Snd1.env.value = (arg & 0x80) >> 4;
        _Snd1.env.len = ((arg & 0x07) * SAMPLES_PER_SEC) / 64;
        break;
    case SoundReg::NR13:
        _Snd1.freq = (_Snd1.freq & 0x0700) | arg;
        break;
    case SoundReg::NR14:
        _Snd1.freq = (_Snd1.freq & 0xff) | ((arg & 0x7) << 8);
        _Snd1.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound(_Snd1);
            bit_set(_NR52, NR52Bits::Sound1On, 1);
        }
        break;
    case SoundReg::NR21:
        _Snd2.len = ((64 - (arg & 0x3F)) * SAMPLES_PER_SEC) / 256;
        // XXX: duty
        break;
    case SoundReg::NR22:
        _Snd2.env.direction = (arg & 0x08) != 0;
        _Snd2.env.value = (arg & 0x80) >> 4;
        _Snd2.env.len = ((arg & 0x07) * SAMPLES_PER_SEC) / 64;
        break;
    case SoundReg::NR23:
        _Snd2.freq = (_Snd2.freq & 0x0700) | arg;
        break;
    case SoundReg::NR24:
        _Snd2.freq = (_Snd2.freq & 0xff) | ((arg & 0x7) << 8);
        _Snd2.loop = (arg & 0x40) == 0;
        if (arg & 0x80) {
            start_sound(_Snd2);
            bit_set(_NR52, NR52Bits::Sound2On, 1);
        }
        break;
    case SoundReg::NR51:
        _NR51 = arg;
        break;
    case SoundReg::NR52:
        _NR52 = arg;
        break;
    };
}

void
SDLAudio::sound(const byte_t *ram)
{
    for (addr_t addr = SoundRegStart; addr < SoundRegEnd; addr++)
        set(addr, ram[addr]);
}

void
SDLAudio::mix(Uint8 *stream, int len)
{
    bvec buf;

    if (!bit_isset(_NR52, NR52Bits::AllOn))
        return;

    if (_Snd1.on) {
        generate(_Snd1, buf, len);
        SDL_MixAudio(stream, &buf[0], buf.size(), SDL_MIX_MAXVOLUME);
    }
    if (_Snd2.on) {
        generate(_Snd2, buf, len);
        SDL_MixAudio(stream, &buf[0], buf.size(), SDL_MIX_MAXVOLUME);
    }
    if (_Snd3.on) {
        //SDL_MixAudio(stream, &buf[0], buf.size(), SDL_MIX_MAXVOLUME);
    }
    if (_Snd4.on) {
        //SDL_MixAudio(stream, &buf[0], buf.size(), SDL_MIX_MAXVOLUME);
    }
}

void
SDLAudio::start_sound(Channel &channel)
{
    channel.period = 1.0f / (131071 / (2048 - channel.freq)) * SAMPLES_PER_SEC;
    channel.pos = 0;
    channel.count = 0;
    channel.signal = 1;
    channel.on = true;
}

static void
envelope(struct Envelope *env)
{
    if (env->len > 0) {
        env->count++;
        if (env->count > env->len) {
            env->count = 0;
            env->value += env->direction;
            if (env->value < 0)
                env->value = 0;
            else if (env->value > 15)
                env->value = 15;
        }
    }
}

static void
sweep(Channel &channel)
{
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
        }
    }
}

sbyte_t
SDLAudio::sample_sound(Channel &channel)
{
    sbyte_t sample = channel.signal * channel.env.value;

    channel.pos++;
    if (channel.pos == channel.period / 2) {
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

    envelope(&channel.env);

    sweep(channel);

    return sample;
}

void
SDLAudio::generate(Channel &channel, bvec &data, int len)
{
    data.resize(len);
    // Handle the sweep register
    for (unsigned i = 0; i < len; i++)
        data[i] = sample_sound(channel);
}

void
SDLAudio::sound4(bvec &data, int len)
{
    data.resize(len);
    for (unsigned i = 0; i < data.size(); i++)
        data[i] = rand();
}
