// DMG

#pragma once

#include <future>
#include <iostream>
#include <iomanip>
#include <type_traits>

typedef unsigned char reg_t;
typedef char sreg_t;
typedef unsigned short addr_t;
typedef unsigned short wreg_t;
typedef unsigned char byte_t;

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

enum class Opcode {
    NOP = 0x00,
    LD_HL_SP_r8 = 0xF8,
    LD_BC_d16 = 0x01,
    LD_BC_A = 0x02,
    INC_BC = 0x03,
    INC_B = 0x04,
    DEC_B = 0x05,
    LD_B_d8 = 0x06,
    RLCA = 0x07,
    LD_a16_SP = 0x08,
    ADD_HL_BC = 0x09,
    LD_A_BC = 0x0A,
    DEC_BC = 0x0B,
    INC_C = 0x0C,
    DEC_C = 0x0D,
    LD_C_d8 = 0x0E,
    RRCA = 0x0F,
    STOP = 0x10,
    LD_DE_d16 = 0x11,
    LD_DC_A = 0x12,
    INC_DE = 0x13,
    INC_D = 0x14,
    DEC_D = 0x15,
    LD_D_d8 = 0x16,
    RLS = 0x17,
    JR_r8 = 0x18,
    ADD_HL_DE = 0x19,
    LD_A_DE = 0x1A,
    DEC_DE = 0x1B,
    INC_E = 0x1C,
    DEC_E = 0x1D,
    LD_E_d8 = 0x1E,
    RRA = 0x1F,
    JR_NZ_R8 = 0x20,
    LD_HL_d16 = 0x21,
    LDI_HL_A = 0x22,
    DAA = 0x27,
    CPL = 0x2F,
    LD_SP_d16 = 0x31,
    LDD_HL_A = 0x32,
    INC_A = 0x3C,
    LD_B_B = 0x40,
    ADD_A_B = 0x80,
    AND_B = 0xA0,
    XOR_A = 0xAF,
    POP_BC = 0xC1,
    CB = 0xCB,
    PUSH_DE = 0xD5,
    POP_AF = 0xF1,
    PUSH_AF = 0xF5,
    PUSH_BC = 0xC5,
    POP_DE = 0xD1,
    LD_A_C = 0x79,
    AND_D8 = 0xE6,
    CP_E = 0xBB,
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
    OpcodeException(Opcode op): _op(op) { };
private:
    Opcode _op;
};

class RomException: protected EmuException {
public:
    RomException(const std::string &name): _name(name) {};
    std::string _name;
};

class Address {
public:
    Address(reg_t &reg, addr_t addr): _reg(reg), _addr(addr) { }

    Address & operator =(const reg_t &rhs);

private:
    reg_t &_reg;
    addr_t _addr;
};

class Callable {
    public:
        Callable() { }

        virtual void operator ()(const reg_t *ram) = 0;
};

struct RegisterSet {
    reg_t A;
    reg_t B;
    reg_t C;
    reg_t D;
    reg_t E;
    wreg_t HL;
    wreg_t SP;
    wreg_t PC;
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
};
typedef std::underlying_type<GBKey>::type key_type;

class Cpu {
public:
    Cpu(void);
    ~Cpu(void);
//    Cpu(const Cpu &rhs);
//    Cpu & operator =(const Cpu &rhs);
    bool operator ==(const Cpu &rhs) const;

    void to_string(std::ostream &os) const;

    // Cycle debug logging
    void debug(bool debug) { _debug = debug; };
    void toggle_debug(void) { _debug = !_debug; };

    // DMG execution
    void reset(bool logo=true);
    void step(void);

    // Exposed for testing only
    reg_t load(Opcode op);
    reg_t load(Opcode op, byte_t arg);
    reg_t load(Opcode op, byte_t arg1, byte_t arg2);
    void test_setup(const RegisterSet &registers, const reg_t *code, addr_t len);
    void test_step(unsigned steps);

    // Test functions
    void set(const RegisterSet &registers);
    void set(Register reg, wreg_t value);
    void set(addr_t addr, reg_t value);
    wreg_t get(Register reg);
    reg_t get(addr_t addr);
    unsigned cycles(void) { return _cycles; };

    void load_rom(const std::string &name);
    void load_rom(const reg_t *data, unsigned len);
    void set_render(Callable *render) { _render = render; };
    void set_key(GBKey key, bool set);
    void dump(void);

protected:

    void _op(void);

    void _add(Register reg, reg_t value);
    void _adc(Register reg, reg_t value);
    void _sub(Register reg, reg_t value);
    void _sbc(Register reg, reg_t value);
    void _and(Register reg, reg_t value);
    void _xor(Register reg, reg_t value);
    void _or(Register reg, reg_t value);
    void _ld(Register reg, reg_t value);
    void _cb(reg_t op);
    void _bit(Register reg, int bit);
    void _res(Register reg, int bit);
    void _set(Register reg, int bit);
    void _inc(Register reg);
    void _dec(Register reg);
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
    void _daa(void);
    void _cpl(void);
    void _ccf(void);
    void _scf(void);

    void _cp(reg_t lhs, reg_t rhs);

    reg_t _store(Register reg, reg_t value);
    reg_t _fetch(Register reg);

    wreg_t _storew(Register reg, wreg_t value);
    wreg_t _fetchw(Register reg);
    void _addw(Register reg, wreg_t value);
    void _incw(Register reg);
    void _subw(Register reg, wreg_t value);
    void _ldw(Register reg, wreg_t value);
    void _decw(Register reg);

    void _ldi(addr_t addr, Register reg);
    void _ldi(addr_t addr, reg_t value);
    void _ldwi(addr_t addr, Register reg);

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

    reg_t to_reg(Register reg);

    // Video processing XXX: bad name
    void video();
    void timer();

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

    void _dma(reg_t value);
    void _write(addr_t addr, reg_t value);

private:
    // XXX: remaining flags
    // Exploit the endianness of the fields so we can access a single
    // byte of 16 bit value
    union {
        struct {
            union {
                struct {
                    reg_t U0:1;
                    reg_t U1:1;
                    reg_t U2:1;
                    reg_t U3:1;
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

    void _interrupt(Interrupt interrupt);
    void _handle_interrupts(void);
    IME _ime;
    State _state;

    unsigned _cycles; // Total number of cycles (XXX: debugging only?)
    unsigned _fcycles; // Current number of cycles since start of hblank
    unsigned _dcycles; // Number of cycles since divider timer
    unsigned _tcycles; // Number of cycles since timer tick
    inline void _tick(unsigned cycles) {
        _cycles += cycles;
        _fcycles += cycles;
        _dcycles += cycles;
        _tcycles += cycles;
    }

    reg_t *_rom;
    unsigned _rom_len;

    Callable *_render;
    reg_t _keys;

    // XXX: debug
    bool _debug;

    Cartridge _type;
    RomSize _rom_size;
    RamSize _ram_size;
    std::string _name;

    unsigned char _mem[64*1024];

    // Async rendering
    void async_render(void);
    //std::future<void> _frame;
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
