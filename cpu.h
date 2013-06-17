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

#include <future>
#include <iostream>
#include <iomanip>

#include "types.h"
#include "bus.h"

namespace DMG {

/*  _   _      _
 * | | | | ___| |_ __   ___ _ __ ___
 * | |_| |/ _ \ | '_ \ / _ \ '__/ __|
 * |  _  |  __/ | |_) |  __/ |  \__ \
 * |_| |_|\___|_| .__/ \___|_|  |___/
 *              |_|
 */
void read_rom(const std::string &name, bvec &rom);

enum class IME {
    Disabled = 0,
    Shadow = 1,
    Enabled = 2
};

enum class State {
    Running = 0,
    Halted = 1,
    Stopped = 2,
    Fault = 3,
};

enum class Register {
    A = 0x07, B = 0x00, C = 0x01, D = 0x02,
    E = 0x03, H = 0x04, L = 0x05, HL = 0x06,
    F = 0x08,
    AF = 0x09, BC = 0x0A, DE = 0x0B, SP = 0x0C, PC = 0x0D
};

class SerialIO: public Device {
public:
    SerialIO(MemoryBus *bus): _bus(bus) { }
    virtual ~SerialIO(void) { }

    virtual void tick(unsigned cycles) { };
    virtual void reset(void) { };
    virtual bool valid(addr_t addr) {
        return ((addr == CtrlReg::SB) || (addr == CtrlReg::SC));
    }
    virtual void write(addr_t addr, byte_t value) {
    }
    virtual byte_t read(addr_t addr) {
        return 0;
    }
private:
    MemoryBus *_bus;
};

class Clock: public Device {
public:
    Clock(MemoryBus *bus);
    virtual ~Clock(void);

    virtual void tick(unsigned cycles);
    virtual void reset(void);
    virtual bool valid(addr_t addr);
    virtual void write(addr_t addr, byte_t value);
    virtual byte_t read(addr_t addr);

private:
    MemoryBus *_bus;

    unsigned _cycles;
    unsigned _dcycles;
    unsigned _tcycles;
    byte_t _div;
    byte_t _tima;
    byte_t _tma;
    byte_t _tac;
};

class RamDevice: public Device {
public:
    ~RamDevice(void) { }
    RamDevice(MemoryBus *bus): _bus(bus) {
        _ram.resize(0x4000);
    }
    virtual void tick(unsigned cycles) { }
    virtual void reset(void) {
        memset(&_ram[0], 0, _ram.size());
    }
    virtual bool valid(addr_t addr) {
        return (addr >= 0xC000 && addr < 0xFE00);
    }
    virtual void write(addr_t addr, byte_t value) {
        addr -= 0xC000;
        _ram[addr] = value;
        _ram[addr ^ 0x2000] = value;
    }
    virtual byte_t read(addr_t addr) {
        return _ram[addr - 0xC000];
    }
private:
    MemoryBus *_bus;

    bvec _ram;
};

class Cpu: public Device {
public:
    Cpu(MemoryBus *bus);
    ~Cpu(void);
    bool operator ==(const Cpu &rhs) const;

    // XXX: Do we need to do something here?
    virtual void tick(unsigned cycles);
    virtual void reset(void);
    virtual bool valid(addr_t addr);
    virtual void write(addr_t addr, byte_t value);
    virtual byte_t read(addr_t addr);

    inline unsigned cycles(void) {
        unsigned tmp = _icycles;
        _icycles = 0;
        return tmp;
    };

    void toggle_debug(void) { _debug = !_debug; };
    void dump(void);

    // Exposed for testing only

    // Test functions
    void test_step(unsigned steps);
    void set(Register reg, word_t value);
    void set(addr_t addr, byte_t value);
    word_t get(Register reg);
    byte_t get(addr_t addr);

protected:

    /* Top level stages */
    void dispatch(void);
    void prefix_cb(void);
    void interrupt(void);

    /* Addition */
    void _add(byte_t &orig, byte_t value);
    void _adc(byte_t &orig, byte_t value);
    void _addw(word_t &worig, word_t value);
    void _inc(byte_t &orig);
    void _inci(addr_t addr);
    void _incw(word_t &worig);

    /* Subtraction */
    void _sub(byte_t &orig, byte_t value);
    void _subw(word_t &worig, word_t value);
    void _sbc(byte_t &orig, byte_t value);
    void _dec(byte_t &orig);
    void _deci(addr_t addr);
    void _decw(word_t &worig);

    /* Load */
    void _ld(byte_t &orig, byte_t value);
    void _ldw(word_t &worig, word_t value);
    void _ldi(addr_t addr, byte_t value);
    void _ldwi(addr_t addr, word_t value);

    /* Bitwise ops */
    void _and(byte_t &orig, byte_t value);
    void _xor(byte_t &orig, byte_t value);
    void _or(byte_t &orig, byte_t value);
    void _bit(byte_t orig, int bit);
    void _reset(byte_t &orig, int bit);
    void _set(byte_t &orig, int bit);
    void _swap(byte_t &orig);
    void _rl(byte_t &orig);
    void _rla(void);
    void _rlc(byte_t &orig);
    void _rlca(void);
    void _rr(byte_t &orig);
    void _rra(void);
    void _rrc(byte_t &orig);
    void _rrca(void);
    void _sla(byte_t &orig);
    void _sra(byte_t &orig);
    void _srl(byte_t &orig);

    void _cp(byte_t lhs, byte_t rhs);
    void _daa(void);
    void _cpl(void);
    void _ccf(void);
    void _scf(void);
    void _ldhlsp(sbyte_t value);
    void _addsp(sbyte_t value);

    void _stop(void);
    void _halt(void);
    void _rst(byte_t value);
    void _jr(bool jump, sbyte_t value);
    void _jp(bool jump, word_t value);
    void _call(bool jump, word_t value);
    void _ret(bool jump);
    void _push(byte_t high, byte_t low);
    void _pop(byte_t &high, byte_t &low);
    void _push(word_t arg);
    void _pop(word_t &arg);

    /* Store/Load */
    byte_t _fetch(Register reg);
    void _store(Register reg, byte_t value);
    word_t _fetchw(Register reg);

    void _write(addr_t addr, byte_t value);
    byte_t _read(addr_t addr);

    inline void _tick(unsigned cycles) {
        _icycles += cycles;
        _cycles += cycles;
    }
    inline void _set_hflag(word_t orig, word_t arg, word_t result) {
        _flags.H = bit_isset(orig ^ arg ^ result, 4);
    }
    inline void _set_cflag(word_t orig, word_t arg, word_t result) {
        _flags.C = bit_isset(orig ^ arg ^ result, 8);
    }
    inline void _set_zflag(word_t result) {
        _flags.Z = (result & 0xff) == 0;
    }
    inline void _set_nflag(bool neg) {
        _flags.N = neg;
    }

    /* decode accessors */
    inline word_t _d16(void) {
        word_t tmp = _read(_rPC) | (_read(_rPC+1) << 8);
        if (_debug)
            std::cout << " d(" << Print(tmp) << ")";
        _rPC+=2;
        return tmp;
    }
    inline byte_t _d8(void) {
        byte_t tmp = _read(_rPC);
        if (_debug)
            std::cout << " d(" << Print(tmp) << ")";
        _rPC++;
        return tmp;
    }
    inline byte_t _r8(void) {
        byte_t tmp = _read(_rPC);
        if (_debug)
            std::cout << " r(" << Print(tmp) << ")";
        _rPC++;
        return tmp;
    }
    inline word_t _a8(void) {
        word_t tmp = _read(_rPC) + 0xff00;
        if (_debug)
            std::cout << " a(" << Print(tmp) << ")";
        _rPC++;
        return tmp;
    }

    void _dump_reg(std::ostream &os) const;

private:
    // Exploit the endianness of the fields so we can access a single
    // byte of 16 bit value
    union {
        struct {
            union {
                struct {
                    byte_t reserved:4;
                    byte_t C:1;
                    byte_t H:1;
                    byte_t N:1;
                    byte_t Z:1;
                } _flags;
                byte_t _rF;
            };
            byte_t _rA;
        };
        word_t _rAF;
    };
    union {
        struct {
            byte_t _rC;
            byte_t _rB;
        };
        word_t _rBC;
    };
    union {
        struct {
            byte_t _rE;
            byte_t _rD;
        };
        word_t _rDE;
    };
    union {
        struct {
            byte_t _rL;
            byte_t _rH;
        };
        word_t _rHL;
    };
    union {
        struct {
            byte_t _rSPl;
            byte_t _rSPh;
        };
        word_t _rSP;
    };
    union {
        struct {
            byte_t _rPCl;
            byte_t _rPCh;
        };
        word_t _rPC;
    };

    IME _ime;
    State _state;
    byte_t _IE;
    byte_t _IF;
    byte_t _ram[256];

    unsigned _icycles; // Cycles of last instruction(s)
    unsigned _cycles;  // Total number of cycles (XXX: debugging only?)

    MemoryBus *_bus;

    // XXX: debug
    bool _debug;
};

enum LCDCBits {
    BGDisplay = 0,
    OBJDisplay = 1,
    OBJSize = 2,
    BGTileMap = 3,
    BGTileData = 4,
    WindowDisplay = 5,
    WindowTileMap = 6,
    LCDEnabled = 7,
};

enum OAMFlags {
    SpritePalette = 4,
    SpriteFlipX = 5,
    SpriteFlipY = 6,
    SpritePriority = 7,
};

enum LCDMode {
    HBlankMode  = 0,
    VBlankMode  = 1,
    OAMMode     = 2,
    ActiveMode  = 3,
};

enum STATBits {
    LYCInterrupt = 6,
    Mode10Int    = 5,
    Mode01Int    = 4,
    Mode00Int    = 3,
    Coincidence  = 2,
    LCDModeBit1  = 1,
    LCDModeBit0  = 0
};

};
