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
 * Timer
 */

#include "timer.h"

using namespace DMG;

Timer::Timer(MemoryBus *bus): _bus(bus)
{
    _bus->add_device(this);
    _bus->add_port(CtrlReg::TIMA, this);
    _bus->add_port(CtrlReg::TMA, this);
    _bus->add_port(CtrlReg::TAC, this);
    _bus->add_port(CtrlReg::DIV, this);
}

Timer::~Timer(void)
{
    _bus->remove_device(this);
}

void
Timer::save(SaveState &state)
{
    state << _cycles;
    state << _dcycles;
    state << _tcycles;
    state << _div;
    state << _tima;
    state << _tma;
    state << _tac;
}

void
Timer::load(LoadState &state)
{
    state >> _cycles;
    state >> _dcycles;
    state >> _tcycles;
    state >> _div;
    state >> _tima;
    state >> _tma;
    state >> _tac;
}

void
Timer::reset(void)
{
    _cycles = 0;
    _dcycles = 0;
    _tcycles = 0;
    _tima = 0;
    _tma = 0;
    _tac = 0;
}

void
Timer::write(addr_t addr, byte_t value)
{
    switch (addr) {
    case CtrlReg::DIV:
        _div = value;
        break;
    case CtrlReg::TIMA:
        _tima = value;
        break;
    case CtrlReg::TMA:
        _tma = value;
        break;
    case CtrlReg::TAC:
        _tac = value;
        break;
    }
}

byte_t
Timer::read(addr_t addr)
{
    switch (addr) {
    case CtrlReg::DIV:
        return _div;
    case CtrlReg::TIMA:
        return _tima;
    case CtrlReg::TMA:
        return _tma;
    case CtrlReg::TAC:
        return _tac;
    }
    return 0;
}

void
Timer::tick(unsigned cycles)
{
    _cycles += cycles;
    _dcycles += cycles;
    _tcycles += cycles;

    if (_dcycles > 256) {
        _dcycles -= 256;
        _div++;
        // Divider register triggered
    }
    if (_tac & 0x04) {
        unsigned limit = 1024;
        switch (_tac & 0x3) {
        case 0: limit = 1024; break;
        case 1: limit = 16; break;
        case 2: limit = 64; break;
        case 3: limit = 256; break;
        }
        if (_tcycles > limit) {
            _tcycles -= limit;
            if (_tima == 0xff) {
                _bus->irq(Interrupt::Timeout);
                // Reset the overflow
                _tima = _tma;
            } else
                _tima++;
        }
    }
}

