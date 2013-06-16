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

typedef unsigned char byte_t;
typedef char sbyte_t;
typedef unsigned short addr_t;
typedef unsigned short word_t;
typedef std::vector<byte_t> bvec;

namespace DMG {

class Print {
public:
    Print(bool arg): v(arg), w(2) {}
    Print(byte_t arg): v(arg), w(2) {}
    Print(sbyte_t arg): v(arg), w(2) {}
    Print(word_t arg): v(arg), w(4) {}
    Print(unsigned arg): v(arg), w(4) {}
    Print(int arg): v(arg), w(2) {}
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

template<typename T>
static inline void bit_set(byte_t &arg, T bit, bool val)
{
    auto n = static_cast<typename std::underlying_type<T>::type>(bit);
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline void bit_set(byte_t &arg, int n, bool val)
{
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline void bit_set(byte_t &arg, unsigned n, bool val)
{
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline bool bit_isset(word_t arg, unsigned n)
{
    return (arg & (1 << n));
}

static inline bool bit_isset(word_t arg, int n)
{
    return (arg & (1 << n));
}

template<typename T>
static inline bool bit_isset(word_t arg, T bit)
{
    auto n = static_cast<typename std::underlying_type<T>::type>(bit);
    return (arg & (1 << n));
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
    OpcodeException(byte_t op): _op(op) { };
private:
    byte_t _op;
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
    Fault = 3,
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

const addr_t SoundRegStart = 0xFF00;
const addr_t SoundRegEnd = 0xFF3F;

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
};

enum CtrlReg {
    KEYS = 0xFF00,
    DIV  = 0xFF04,
    TIMA = 0xFF05,
    TMA  = 0xFF06,
    TAC  = 0xFF07,
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
static inline byte_t key_value(GBKey key) {
    return static_cast<std::underlying_type<GBKey>::type>(key);
}

class Audio {
public:
    Audio() { };

    virtual void set(addr_t addr, byte_t arg) { };
    virtual void sound(const byte_t *ram) { };

private:
};

class Video {
public:
    Video() { };

    virtual void render(const byte_t *ram) { };
};

class Controller {
public:
    Controller() { };

    virtual byte_t get_buttons(void) const { return 0x0F; };
    virtual byte_t get_arrows(void) const { return 0x0F; };
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
    void set_audio(Audio *audio) { _audio = audio; };
    void set_key(GBKey key, bool set);
    void toggle_debug(void) { _debug = !_debug; };
    void dump(void);

    // Exposed for testing only

    // Test functions
    byte_t load(byte_t op);
    byte_t load(byte_t op, byte_t arg);
    byte_t load(byte_t op, byte_t arg1, byte_t arg2);
    void test_step(unsigned steps);
    void set(Register reg, word_t value);
    void set(addr_t addr, byte_t value);
    word_t get(Register reg);
    byte_t get(addr_t addr);
    unsigned cycles(void) { return _cycles; };

protected:

    /* Top level stages */
    void dispatch(void);
    void prefix_cb(void);
    void interrupt(void);
    void video();
    void timer();
    void audio();

    inline void _trigger(Interrupt i) {
        byte_t ifreg = _read(CtrlReg::IF);
        bit_set(ifreg, i, true);
        _write(CtrlReg::IF, ifreg);
    };

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
    void _call(word_t value);
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
    byte_t _read(addr_t addr) const;
    inline void _tick(unsigned cycles) {
        _cycles += cycles;
        _fcycles += cycles;
        _dcycles += cycles;
        _tcycles += cycles;
        _scycles += cycles;
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

    void _read_rom(const std::string &name, bvec &rom);
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

    unsigned _cycles; // Total number of cycles (XXX: debugging only?)
    unsigned _fcycles; // Current number of cycles since start of hblank
    unsigned _dcycles; // Number of cycles since divider timer
    unsigned _tcycles; // Number of cycles since timer tick
    unsigned _scycles; // Number of cycles since sound handling

    bvec _rom;

    Video _null_video;
    Video *_video;

    Controller _null_controller;
    Controller *_control;

    Audio _null_audio;
    Audio *_audio;

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
