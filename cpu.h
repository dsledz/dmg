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
#include <type_traits>
#include <vector>

typedef unsigned char reg_t;
typedef char sreg_t;
typedef unsigned short addr_t;
typedef unsigned short wreg_t;
typedef unsigned char byte_t;
typedef std::vector<byte_t> bvec;

namespace DMG {

class Print {
public:
    Print(bool reg): v(reg), w(2) {}
    Print(reg_t reg): v(reg), w(2) {}
    Print(sreg_t reg): v(reg), w(2) {}
    Print(wreg_t reg): v(reg), w(4) {}
    Print(unsigned reg): v(reg), w(4) {}
    Print(int reg): v(reg), w(2) {}
    unsigned v;
    unsigned w;
};

static inline std::ostream& operator << (std::ostream &os,
                                         const Print& obj)
{
    os << std::hex << "0x" << std::setw(obj.w) << std::setfill('0')
        << obj.v;
    return os;
}

static inline void bit_set(reg_t &reg, unsigned n, bool val)
{
    reg &= ~(1 << n);
    reg |= (val ? (1 << n) : 0);
}

static inline bool bit_isset(reg_t reg, unsigned n)
{
    return (reg & (1 << n));
}

/*  _____                    _   _
 * | ____|_  _____ ___ _ __ | |_(_) ___  _ __  ___
 * |  _| \ \/ / __/ _ \ '_ \| __| |/ _ \| '_ \/ __|
 * | |___ >  < (_|  __/ |_) | |_| | (_) | | | \__ \
 * |_____/_/\_\___\___| .__/ \__|_|\___/|_| |_|___/
 *                    |_|
 */
class EmuException {
public:
    EmuException() {};
};

class CpuException: protected EmuException {
public:
    CpuException() {};
};

class OpcodeException: protected EmuException {
public:
    OpcodeException(reg_t op): _op(op) { };
private:
    reg_t _op;
};

class RomException: protected EmuException {
public:
    RomException(const std::string &name): rom(name) {};
    std::string rom;
};

enum class IME {
    Disabled = 0,
    Shadow = 1,
    Enabled = 2
};

enum class State {
    Running = 0,
    Halted = 1,
    Stopped = 2,
};

enum class Register {
    A = 0x07, B = 0x00, C = 0x01, D = 0x02,
    E = 0x03, H = 0x04, L = 0x05, HL = 0x06,
    F = 0x08,
    AF = 0x09, BC = 0x0A, DE = 0x0B, SP = 0x0C, PC = 0x0D
};

enum Interrupt {
    VBlank = 0,
    LCDStat = 1,
    Timer = 2,
    Serial = 3,
    Joypad = 4,
};

enum Mem {
    ObjTiles = 0x8000,
    BGTiles  = 0x8800,
    TileMap0 = 0x9800,
    TileMap1 = 0x9C00,
    OAMTable = 0xFE00,
};

enum Oam {
    OamY = 0,
    OamX = 1,
    OamPattern = 2,
    OamFlags = 3,
};

enum CtrlReg {
    KEYS = 0xFF00,
    DIV  = 0xFF04,
    TIMA = 0xFF05,
    TMA  = 0xFF06,
    TAC  = 0xFF07,
    NR10 = 0xFF10,
    NR11 = 0xFF11,
    NR12 = 0xFF12,
    NR13 = 0xFF13,
    NR14 = 0xFF14,
    NR21 = 0xFF16,
    NR22 = 0xFF17,
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
    LCDC = 0xFF40,
    STAT = 0xFF41,
    SCY  = 0xFF42,
    SCX  = 0xFF43,
    LY   = 0xFF44,
    LYC  = 0xFF45,
    DMA  = 0xFF46,
    BGP  = 0xFF47,
    OBP0 = 0xFF48,
    OBP1 = 0xFF49,
    WY   = 0xFF4A,
    WX   = 0xFF4B,
    DMG_RESET = 0xFF50,
    IE   = 0xFFFF,
    IF   = 0xFF0F,
};

enum Cartridge {
    RomOnly = 0x00,
    MBC1    = 0x01,
    MBC1R   = 0x02,
    MBC1RB  = 0x03,
};

enum RomSize {
    Banks0    = 0x00,
    Banks4    = 0x01,
    Banks8    = 0x02,
    Banks16   = 0x03,
    Banks32   = 0x04,
    Banks64   = 0x05,
    Banks128  = 0x06,
    Banks256  = 0x07,
    Banks72   = 0x52,
    Banks80   = 0x53,
    Banks96   = 0x54,
};

enum RamSize {
    Ram0     = 0x00,
    Ram1     = 0x01,
    Ram2     = 0x02,
    Ram3     = 0x03,
};

enum class GBKey {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Right = 4,
    Left = 5,
    Up = 6,
    Down = 7,
    None = 8,
    Size = 8,
};
static inline reg_t key_value(GBKey key) {
    return static_cast<std::underlying_type<GBKey>::type>(key);
}


class Video {
public:
    Video() { };

    virtual void render(const reg_t *ram) { };
};

class Controller {
public:
    Controller() { };

    virtual reg_t get_buttons(void) const { return 0x0F; };
    virtual reg_t get_arrows(void) const { return 0x0F; };
};

class Cpu {
public:
    Cpu(void);
    ~Cpu(void);
    bool operator ==(const Cpu &rhs) const;

    // DMG execution
    void reset(void);
    void step(void);

    void load_rom(const std::string &name);
    void set_video(Video *video) { _video = video; };
    void set_control(Controller *control) { _control = control; };
    void set_key(GBKey key, bool set);
    void toggle_debug(void) { _debug = !_debug; };
    void dump(void);

    // Exposed for testing only

    // Test functions
    reg_t load(reg_t op);
    reg_t load(reg_t op, byte_t arg);
    reg_t load(reg_t op, byte_t arg1, byte_t arg2);
    void test_step(unsigned steps);
    void set(Register reg, wreg_t value);
    void set(addr_t addr, reg_t value);
    wreg_t get(Register reg);
    reg_t get(addr_t addr);
    unsigned cycles(void) { return _cycles; };

protected:

    /* Top level stages */
    void dispatch(void);
    void prefix_cb(void);
    void interrupt(void);
    void video();
    void timer();

    /* Addition */
    void _add(Register reg, reg_t value);
    void _addw(Register reg, wreg_t value);
    void _adc(Register reg, reg_t value);
    void _inc(Register reg);
    void _incw(Register reg);

    /* Subtraction */
    void _sub(Register reg, reg_t value);
    void _subw(Register reg, wreg_t value);
    void _sbc(Register reg, reg_t value);
    void _dec(Register reg);
    void _decw(Register reg);

    /* Load */
    void _ld(Register reg, reg_t value);
    void _ldw(Register reg, wreg_t value);
    void _ldi(addr_t addr, Register reg);
    void _ldi(addr_t addr, reg_t value);
    void _ldwi(addr_t addr, Register reg);

    /* Bitwise ops */
    void _and(Register reg, reg_t value);
    void _xor(Register reg, reg_t value);
    void _or(Register reg, reg_t value);
    void _bit(Register reg, int bit);
    void _res(Register reg, int bit);
    void _set(Register reg, int bit);
    void _swap(Register reg);
    void _rl(Register reg);
    void _rla(void);
    void _rlc(Register reg);
    void _rlca(void);
    void _rr(Register reg);
    void _rra(void);
    void _rrc(Register reg);
    void _rrca(void);
    void _sla(Register reg);
    void _sra(Register reg);
    void _srl(Register reg);

    void _cp(reg_t lhs, reg_t rhs);
    void _daa(void);
    void _cpl(void);
    void _ccf(void);
    void _scf(void);

    void _stop(void);
    void _halt(void);
    void _rst(reg_t value);
    void _jr(bool jump, sreg_t value);
    void _jp(bool jump, wreg_t value);
    void _call(wreg_t value);
    void _call(bool jump, wreg_t value);
    void _ret(bool jump);
    void _push(reg_t high, reg_t low);
    void _pop(reg_t &high, reg_t &low);
    void _push(wreg_t reg);
    void _pop(wreg_t &reg);

    /* Store/Load */
    reg_t _store(Register reg, reg_t value);
    wreg_t _storew(Register reg, wreg_t value);
    reg_t _fetch(Register reg);
    wreg_t _fetchw(Register reg);
    void _write(addr_t addr, reg_t value);
    inline void _tick(unsigned cycles) {
        _cycles += cycles;
        _fcycles += cycles;
        _dcycles += cycles;
        _tcycles += cycles;
    }

    /* decode accessors */
    inline wreg_t _d16(void) {
        wreg_t tmp = _mem[_PC] | (_mem[_PC+1] << 8);
        if (_debug)
            std::cout << " d(" << Print(tmp) << ")";
        _PC+=2;
        return tmp;
    }
    inline reg_t _d8(void) {
        reg_t tmp = _mem[_PC];
        if (_debug)
            std::cout << " d(" << Print(tmp) << ")";
        _PC++;
        return tmp;
    }
    inline sreg_t _r8(void) {
        reg_t tmp = _mem[_PC];
        if (_debug)
            std::cout << " r(" << Print(tmp) << ")";
        _PC++;
        return static_cast<sreg_t>(tmp);
    }
    inline wreg_t _a8(void) {
        wreg_t tmp = _mem[_PC] + 0xff00;
        if (_debug)
            std::cout << " a(" << Print(tmp) << ")";
        _PC++;
        return tmp;
    }

    void _read_rom(const std::string &name, bvec &rom);
    void _dump_reg(std::ostream &os) const;

private:
    // Exploit the endianness of the fields so we can access a single
    // byte of 16 bit value
    union {
        struct {
            union {
                struct {
                    reg_t reserved:4;
                    reg_t C:1;
                    reg_t H:1;
                    reg_t N:1;
                    reg_t Z:1;
                } _flags;
                reg_t _F;
            };
            reg_t _A;
        };
        wreg_t _AF;
    };
    union {
        struct {
            reg_t _C;
            reg_t _B;
        };
        wreg_t _BC;
    };
    union {
        struct {
            reg_t _E;
            reg_t _D;
        };
        wreg_t _DE;
    };
    union {
        struct {
            reg_t _L;
            reg_t _H;
        };
        wreg_t _HL;
    };
    union {
        struct {
            reg_t _SPl;
            reg_t _SPh;
        };
        wreg_t _SP;
    };
    union {
        struct {
            reg_t _PCl;
            reg_t _PCh;
        };
        wreg_t _PC;
    };

    IME _ime;
    State _state;

    unsigned _cycles; // Total number of cycles (XXX: debugging only?)
    unsigned _fcycles; // Current number of cycles since start of hblank
    unsigned _dcycles; // Number of cycles since divider timer
    unsigned _tcycles; // Number of cycles since timer tick

    bvec _rom;

    Video _null_video;
    Video *_video;

    Controller _null_controller;
    Controller *_control;
    reg_t _keys;

    // XXX: debug
    bool _debug;

    Cartridge _type;
    RomSize _rom_size;
    RamSize _ram_size;
    std::string _name;

    bvec _mem;
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

};
