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
/**
 * Serialization library
 */
#pragma once

#include "types.h"

namespace DMG {

class SaveState {
public:
    SaveState(bvec &data): data(data), pos(0) {
        data.resize(0);
    }

    ~SaveState(void) { }

    bvec &data;
    unsigned pos;
};

class LoadState {
public:
    LoadState(const bvec &data): data(data), pos(0) { }

    const bvec &data;
    unsigned pos;
};

template<typename T> static inline
SaveState & operator << (SaveState & state, const T & d)
{
    state.data.resize(state.pos + sizeof(d));
    memcpy(&state.data[state.pos], &d, sizeof(d));
    state.pos += sizeof(d);
    return state;
}

static inline
SaveState & operator <<(SaveState &state, const bvec & d)
{
    unsigned size = d.size();
    state << size;
    state.data.resize(state.pos + d.size());
    memcpy(&state.data[state.pos], d.data(), d.size());
    state.pos += d.size();
    return state;
}

template<typename T> static inline
LoadState & operator >>(LoadState & state, T & d)
{
    memcpy(&d, &state.data[state.pos], sizeof(d));
    state.pos += sizeof(d);
    return state;
}

static inline
LoadState & operator >>(LoadState & state, bvec & d)
{
    unsigned size;
    state >> size;
    d.resize(size);
    memcpy(d.data(), &state.data[state.pos], size);
    state.pos += size;
    return state;
}


};
