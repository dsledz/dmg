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
 * Gameboy CPU. Similar, to a stripped down Z80
 */

#include <sys/stat.h>
#include "cpu.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <type_traits>

#include "roms/cpu_instrs.h"
#include "roms/dmg.h"

#define CURRENT_ROM cpu_instrs_rom

using namespace DMG;

Cpu::Cpu(void): _A(0), _F(0), _B(0), _C(0), _D(0), _E(0), _H(0), _L(0),
     _SP(0), _PC(0), _rom(NULL), _debug(false)
{
    memset(_mem, 0, sizeof(_mem));
}

Cpu::~Cpu(void)
{
    if (_rom != NULL)
        delete[] _rom;
    _rom = NULL;
}

#ifdef EQUAL_FAST
bool Cpu::operator ==(const Cpu &rhs) const
{
    return _A == rhs._A && _F == rhs._F &&
        _B == rhs._B && _C == rhs._C &&
        _D == rhs._D && _E == rhs._E &&
        _H == rhs._H && _L == rhs._L &&
        _SP == rhs._SP && _PC == rhs._PC &&
        _flags.Z == rhs._flags.Z && _flags.C == rhs._flags.C &&
        _mem == rhs._mem;
}
#else
#define EQ(a, b) \
    if (a != b) { \
        std::cout << std::hex << "Mismatch '" #a "' Expected: " << Print(a) \
                  << " Actual: " << Print(b) << std::endl; \
        return false; \
    }

bool Cpu::operator == (const Cpu &rhs) const
{
    EQ(_A, rhs._A);
    EQ(_F, rhs._F);
    EQ(_B, rhs._B);
    EQ(_C, rhs._C);
    EQ(_D, rhs._D);
    EQ(_E, rhs._E);
    EQ(_H, rhs._H);
    EQ(_L, rhs._L);
    EQ(_SP, rhs._SP);
    EQ(_PC, rhs._PC);
    EQ(_flags.Z, rhs._flags.Z);
    EQ(_flags.C, rhs._flags.C);
    for (unsigned i = 0; i < sizeof(_mem); i++)
        if (_mem[i] != rhs._mem[i]) {
            std::cout << "Memory differences at: " << Print(i) << std::endl;
            EQ(_mem[i], rhs._mem[i]);
        }
    return true;
}
#endif

/*
 *  ___                       _   _
 * / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___
 *| | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __|
 *| |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \
 * \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/
 *      |_|
 */

reg_t Cpu::_store(Register reg, reg_t value)
{
    if (_debug)
        std::cout << "  <- " << Print(value);

    switch (reg) {
    case Register::A: _A = value; break;
    case Register::B: _B = value; break;
    case Register::C: _C = value; break;
    case Register::D: _D = value; break;
    case Register::E: _E = value; break;
    case Register::H: _H = value; break;
    case Register::L: _L = value; break;
    case Register::HL: _write(_HL, value); break;
    case Register::SP: _write(_SP, value); break;
    default: throw CpuException();
    }
    return value;
}

reg_t Cpu::_fetch(Register reg)
{
    switch (reg) {
        case Register::A: return _A;
        case Register::B: return _B;
        case Register::C: return _C;
        case Register::D: return _D;
        case Register::E: return _E;
        case Register::H: return _H;
        case Register::L: return _L;
        case Register::BC: return _mem[_BC];
        case Register::DE: return _mem[_DE];
        case Register::HL: return _mem[_HL];
        case Register::SP: return _mem[_SP];
        default: throw CpuException();
    }
    throw CpuException();
}

void Cpu::_add(Register reg, reg_t value)
{
    reg_t lhs = _fetch(reg);
    wreg_t res = lhs + value;
    _flags.C = (res > 255) ? true : false;
    _flags.H = ((lhs & 0x0f) + (value & 0x0f) > 0x0f) ? true : false;
    _flags.N = false;
    _flags.Z = (res & 0xff) == 0;

    _store(reg, res);
}

void Cpu::_inc(Register reg)
{
    reg_t value = _fetch(reg) + 1;
    _flags.Z = (value == 0);
    _flags.H = (value & 0xf) == 0x00;
    _flags.N = false;

    _store(reg, value);
}

void Cpu::_adc(Register reg, reg_t value)
{
    reg_t lhs = _fetch(reg);
    wreg_t res = lhs + value + _flags.C;
    _flags.H = ((lhs & 0x0f) + (value & 0x0f) + _flags.C > 0x0f) ? true : false;
    _flags.C = (res > 255) ? true : false;
    _flags.N = false;
    _flags.Z = (res & 0xff) == 0;

    _store(reg, res);
}

void Cpu::_sub(Register reg, reg_t value)
{
    _flags.C = (value > _fetch(reg)) ? true : false;
    _flags.H = ((value & 0x0f) > (_fetch(reg) & 0x0f)) ? true : false;
    value = _fetch(reg) - value;
    _flags.Z = (value == 0);

    _store(reg, value);
    _flags.N = true;
}

void Cpu::_dec(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.H = (value & 0x0f) == 0;
    _flags.Z = (value == 1);

    value -= 1;
    _store(reg, value);
    _flags.N = true;
}

void Cpu::_sbc(Register reg, reg_t value)
{
    reg_t carry = _flags.C;
    _flags.C = (value > _fetch(reg) - carry) ? true : false;
    _flags.H = ((value & 0x0f) + carry > (_fetch(reg) & 0x0f)) ? true : false;
    value = _fetch(reg) - value - carry;
    _flags.Z = (value == 0);

    _store(reg, value);
    _flags.N = true;

}

void Cpu::_cp(reg_t lhs, reg_t rhs)
{
    if (_debug)
        std::cout << "   " << Print(lhs) << "<->" << Print(rhs);

    _flags.Z = (lhs == rhs);
    _flags.C = (rhs > lhs);
    _flags.H = ((rhs & 0x0f) > (lhs & 0x0f)) ? true : false;
    _flags.N = true;
}

void Cpu::_and(Register reg, reg_t value)
{
    value = _fetch(reg) & value;
    _flags.Z = (value == 0);
    _flags.C = 0;
    _flags.H = 1;
    _flags.N = 0;

    _store(reg, value);
}

void Cpu::_xor(Register reg, reg_t value)
{
    value = _fetch(reg) ^ value;
    _flags.Z = (value == 0);
    _flags.C = 0;
    _flags.H = 0;
    _flags.N = 0;

    _store(reg, value);
}

void Cpu::_or(Register reg, reg_t value)
{
    value = _fetch(reg) | value;
    _flags.Z = (value == 0);
    _flags.C = 0;
    _flags.H = 0;
    _flags.N = 0;

    _store(reg, value);
}

void Cpu::_ld(Register reg, reg_t value)
{
    _store(reg, value);
}

void Cpu::_bit(Register reg, int bit)
{
    _flags.Z = (_fetch(reg) & (1 << bit)) == 0;
    _flags.N = 0;
    _flags.H = 1;
}

void Cpu::_res(Register reg, int bit)
{
    _store(reg, _fetch(reg) & ~(1 << bit));
}

void Cpu::_set(Register reg, int bit)
{
    _store(reg, _fetch(reg) | (1 << bit));
}

void Cpu::_rl(Register reg)
{
    reg_t value = _fetch(reg);
    reg_t carry = _flags.C;
    _flags.C = (value & 0x80) != 0;
    value = (value << 1) | carry;
    _flags.Z = (value == 0);
    _flags.N = 0;
    _flags.H = 0;

    _store(reg, value);
}

void Cpu::_rla(void)
{
    _rl(Register::A);
    _flags.Z = 0;
}

void Cpu::_rlc(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.C = (value & 0x80) != 0;
    value = (value << 1) | _flags.C;
    _flags.Z = (value == 0);
    _flags.N = 0;
    _flags.H = 0;

    _store(reg, value);
}

void Cpu::_rlca(void)
{
    _rlc(Register::A);
    _flags.Z = 0;
}

void Cpu::_rr(Register reg)
{
    reg_t value = _fetch(reg);
    reg_t carry = (_flags.C ? 0x80 : 0x00);
    _flags.C = (value & 0x01);
    value = (value >> 1) | carry;
    _flags.Z = (value == 0);
    _flags.N = 0;
    _flags.H = 0;

    _store(reg, value);
}

void Cpu::_rra(void)
{
    _rr(Register::A);
    _flags.Z = 0;
}

void Cpu::_rrc(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.C = (value & 0x01);
    reg_t carry = (_flags.C ? 0x80 : 0x00);
    value = (value >> 1) | carry;
    _flags.Z = (value == 0);
    _flags.N = false;
    _flags.H = false;

    _store(reg, value);
}

void Cpu::_rrca(void)
{
    _rrc(Register::A);
    _flags.Z = 0;
}

void Cpu::_sla(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.C = (value & 0x80) != 0;
    value = (value << 1);
    _flags.Z = (value == 0);
    _flags.N = false;
    _flags.H = false;

    _store(reg, value);
}

void Cpu::_sra(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.C = (value & 0x01);
    reg_t carry = value & 0x80;
    value = (value >> 1) | carry;
    _flags.Z = (value == 0);
    _flags.N = false;
    _flags.H = false;

    _store(reg, value);
}

void Cpu::_srl(Register reg)
{
    reg_t value = _fetch(reg);
    _flags.C = (value & 0x01);
    value = (value >> 1);
    _flags.Z = (value == 0);
    _flags.N = false;
    _flags.H = false;

    _store(reg, value);
}

void Cpu::_swap(Register reg)
{
    reg_t value = _fetch(reg);
    value = (value >> 4) | (value << 4);

    _flags.Z = (value == 0);
    _flags.C = 0;
    _flags.N = false;
    _flags.H = false;

    _store(reg, value);
}

void Cpu::_cb(reg_t op)
{
    int bit = (op & 0x38) >> 3;
    Register reg = Register(op & 0x7);
    if ((op & 0xF8) == 0x00) {
        _rlc(reg);
    } else if ((op & 0xF8) == 0x08) {
        _rrc(reg);
    } else if ((op & 0xF8) == 0x10) {
        _rl(reg);
    } else if ((op & 0xF8) == 0x18) {
        _rr(reg);
    } else if ((op & 0xF8) == 0x20) {
        _sla(reg);
    } else if ((op & 0xF8) == 0x28) {
        _sra(reg);
    } else if ((op & 0xF8) == 0x30) {
        _swap(reg);
    } else if ((op & 0xF8) == 0x38) {
        _srl(reg);
    } else if ((op & 0xC0) == 0x40) {
        _bit(reg, bit);
    } else if ((op & 0xC0) == 0x80) {
        _res(reg, bit);
    } else if ((op & 0xC0) == 0xC0) {
        _set(reg, bit);
    } else {
        std::cout << "Unknown CB opcode: " << Print(op) << std::endl;
        throw OpcodeException(Opcode(op));
    }
}

void Cpu::_rst(reg_t value)
{
    _push(_PCh, _PCl);
    _PC = value;
}

void Cpu::_jr(bool jump, sreg_t value)
{
    if (jump) {
        _PC += (char)value;
        _tick(4);
    }
}

void Cpu::_jp(bool jump, wreg_t value)
{
    if (jump) {
        if (_debug)
            std::cout << "  <- " << Print(value);
        _PC = value;
        _tick(4);
    }
}

void Cpu::_call(bool jump, wreg_t addr)
{
    if (jump) {
        _call(addr);
        _tick(12);
    }
}

void Cpu::_call(wreg_t addr)
{
    _push(_PCh, _PCl);

    if (_debug) {
        std::cout << "  <- " << Print(addr);
    }
    _PC = addr;
}

void Cpu::_ret(bool jump)
{
    if (jump) {
        _pop(_PCh, _PCl);
        _tick(16);
    }
}

void Cpu::_push(reg_t high, reg_t low)
{
    if (_debug) {
        wreg_t value = high;
        value = value << 8 | low;
        std::cout << " <- " << Print(value);
    }

    _write(--_SP, high);
    _write(--_SP, low);
}

void Cpu::_pop(reg_t &high, reg_t &low)
{
    low = _mem[_SP++];
    high = _mem[_SP++];
    if (_debug) {
        wreg_t value = high;
        value = value << 8 | low;
        std::cout << " -> " << Print(value);
    }
}

void Cpu::_halt()
{
    // Cpu is halted until the next interrupt
    _state = State::Halted;
}

void Cpu::_stop()
{
    _d8();
    std::cout << "CPU stopped" << std::endl;
    to_string(std::cout);
    _state = State::Stopped;
}

void Cpu::dump(void)
{
    to_string(std::cout);
}

wreg_t Cpu::_storew(Register reg, wreg_t value)
{
    if (_debug)
        std::cout << "  <- " << Print(value);

    switch (reg) {
    case Register::BC: _BC = value; break;
    case Register::DE: _DE = value; break;
    case Register::HL: _HL = value; break;
    case Register::PC: _PC = value; break;
    case Register::SP: _SP = value; break;
    default: throw CpuException();
    }
    return value;
}

wreg_t Cpu::_fetchw(Register reg)
{
    switch (reg) {
        case Register::BC: return _BC;
        case Register::DE: return _DE;
        case Register::HL: return _HL;
        case Register::PC: return _PC;
        case Register::SP: return _SP;
        default: throw CpuException();
    }
    throw CpuException();
}

void Cpu::_addw(Register reg, wreg_t value)
{
    wreg_t res = _fetchw(reg);

    res += value;
    _flags.C = (res < value) ? true : false;
    _flags.H = ((res & 0xfff) < (value & 0xfff)) ? true : false;
    _flags.N = 0;
    _storew(reg, res);
}

void Cpu::_incw(Register reg)
{
    wreg_t value = _fetchw(reg) + 1;
    _storew(reg, value);
}

void Cpu::_subw(Register reg, wreg_t value)
{
    value = _fetchw(reg) - value;
    _storew(reg, value);
}

void Cpu::_decw(Register reg)
{
    wreg_t value = _fetchw(reg) - 1;
    _storew(reg, value);
}

void Cpu::_ldw(Register reg, wreg_t value)
{
    _storew(reg, value);
}

void Cpu::_ldi(addr_t addr, reg_t value)
{
    if (_debug)
        std::cout << " -> " << Print(addr);
    _write(addr, value);
}

void Cpu::_ldi(addr_t addr, Register reg)
{
    if (_debug)
        std::cout << " -> " << Print(addr);
    switch (reg) {
    case Register::SP:
        _write(addr, _SPl);
        _write(addr+1, _SPh);
    default:
        _write(addr, _fetch(reg));
    }
}

void Cpu::_cpl(void)
{
    reg_t value = _fetch(Register::A);
    value = ~value;
    _flags.N = true;
    _flags.H = true;
    _store(Register::A, value);
}

void Cpu::_ccf(void)
{
    _flags.C = !_flags.C;
    _flags.N = 0;
    _flags.H = 0;
}

void Cpu::_scf(void)
{
    _flags.N = false;
    _flags.H = false;
    _flags.C = true;
}

void Cpu::_daa(void)
{
    reg_t value = _fetch(Register::A);
    reg_t l = value & 0xf;
    reg_t h = (value & 0xf0) >> 4;
    reg_t add = 0;
    if (!_flags.N) {
        if (!_flags.C && h <= 9 && !_flags.H && l <= 9)
            add = 0x00;
        else if (!_flags.C && h <=8 && !_flags.H && l > 9)
            add = 0x06;
        else if (!_flags.C && h <=9 && _flags.H && l <= 3)
            add = 0x06;
        else if (!_flags.C && h > 9 && !_flags.H && l <= 9)
            add = 0x60;
        else if (!_flags.C && h >=9 && !_flags.H && l > 9)
            add = 0x66;
        else if (!_flags.C && h > 9 && _flags.H && l <= 3)
            add = 0x66;
        else if (_flags.C && h < 3 && !_flags.H && l <= 9)
            add = 0x60;
        else if (_flags.C && h < 3 && !_flags.H && l > 9)
            add = 0x66;
        else if (_flags.C && h <=3 && _flags.H && l <= 3)
            add = 0x66;
        _flags.C = (add >= 0x60);
    } else {
        if (!_flags.C && h <= 9 && !_flags.H && l <= 9) {
            add = 0x00;
            _flags.C = 0;
        } else if (!_flags.C && h < 9 && _flags.H && l >= 6) {
            add = 0xFA;
            _flags.C = 0;
        } else if (_flags.C && h >=7 && !_flags.H && l <= 9) {
            add = 0xA0;
            _flags.C = 1;
        } else if (_flags.C && (h == 6 || h == 7) && _flags.H && l >=6) {
            add = 0x9A;
            _flags.C = 1;
        }
    }
    _add(Register::A, add);
}

#define OPCODE(op, cycles, bytes, name, func) \
    case op: { \
        if (_debug) \
            std::cout << Print(_cycles) << ": " \
                      << Print(pc) << ": " << Print(op) << ": " \
                      << name; \
        func; \
        _tick(cycles); \
        if (_debug) \
            std::cout << std::endl; \
        break; \
    }

void Cpu::_op(void)
{
    wreg_t pc = _PC;
    reg_t op = _mem[_PC++];
    //Register src = Register(op & 0x7);
    //Register dest = Register((op & 0x38) >> 3);

    switch (op) {
        OPCODE(0x00, 4, 1, "NOP", );
        OPCODE(0x01, 10, 3, "LD BC,d16", _ldw(Register::BC, _d16()));
        OPCODE(0x02, 8, 1, "LD (BC),A", _ldi(_BC, _A));
        OPCODE(0x03, 8, 1, "INC BC", _incw(Register::BC));
        OPCODE(0x04, 4, 1, "INC B", _inc(Register::B));
        OPCODE(0x05, 4, 1, "DEC B", _dec(Register::B));
        OPCODE(0x06, 8, 2, "LD B,d8", _ld(Register::B, _d8()));
        OPCODE(0x07, 4, 1, "RLCA", _rlca());
        OPCODE(0x08, 20, 3, "LD (a16), SP", _ldi(_d16(), Register::SP));
        OPCODE(0x09, 8, 1, "ADD HL,BC", _addw(Register::HL, _BC));
        OPCODE(0x0A, 8, 1, "LD A,(BC)", _ld(Register::A, _fetch(Register::BC)));
        OPCODE(0x0B, 8, 1, "DEC BC", _decw(Register::BC));
        OPCODE(0x0C, 4, 1, "INC C", _inc(Register::C));
        OPCODE(0x0D, 4, 1, "DEC C", _dec(Register::C));
        OPCODE(0x0E, 8, 2, "LD C,d8", _ld(Register::C, _d8()));
        OPCODE(0x0F, 4, 1, "RRCA", _rrca());
        OPCODE(0x10, 4, 2, "STOP", _stop());
        OPCODE(0x11, 10, 3, "LD DE,d16", _ldw(Register::DE, _d16()));
        OPCODE(0x12, 8, 1, "LD (DE),A", _ldi(_DE, Register::A));
        OPCODE(0x13, 8, 1, "INC DE", _incw(Register::DE));
        OPCODE(0x14, 4, 1, "INC D", _inc(Register::D));
        OPCODE(0x15, 4, 1, "DEC D", _dec(Register::D));
        OPCODE(0x16, 8, 2, "LD D,d8", _ld(Register::D, _d8()));
        OPCODE(0x17, 4, 1, "RLA", _rla());
        OPCODE(0x18, 12, 2, "JR r8", _jr(true, _r8()));
        OPCODE(0x19, 8, 1, "ADD HL,DE", _addw(Register::HL, _DE));
        OPCODE(0x1A, 8, 1, "LD A,(DE)", _ld(Register::A, _fetch(Register::DE)));
        OPCODE(0x1B, 8, 1, "DEC DE", _decw(Register::DE));
        OPCODE(0x1C, 4, 1, "INC E", _inc(Register::E));
        OPCODE(0x1D, 4, 1, "DEC E", _dec(Register::E));
        OPCODE(0x1E, 8, 2, "LD E,d8", _ld(Register::E, _d8()));
        OPCODE(0x1F, 4, 1, "RRA", _rra());
        OPCODE(0x20, 8, 2, "JR NZ,r8", _jr(!_flags.Z, _r8()));
        OPCODE(0x21, 10, 3, "LD HL,d16", _ldw(Register::HL, _d16()));
        OPCODE(0x22, 8, 1, "LDI (HL), A", _ldi(_HL++, Register::A));
        OPCODE(0x23, 8, 1, "INC HL", _incw(Register::HL));
        OPCODE(0x24, 4, 1, "INC H", _inc(Register::H));
        OPCODE(0x25, 4, 1, "DEC H", _dec(Register::H));
        OPCODE(0x26, 8, 2, "LD H,d8", _ld(Register::H, _d8()));
        OPCODE(0x27, 4, 1, "DAA", _daa());
        OPCODE(0x28, 8, 2, "JR Z,r8", _jr(_flags.Z, _r8()));
        OPCODE(0x29, 8, 1, "ADD HL,HL", _addw(Register::HL, _HL));
        OPCODE(0x2A, 8, 1, "LDI a, (HL)", _ld(Register::A, _mem[_HL++]));
        OPCODE(0x2B, 8, 1, "DEC HL", _decw(Register::HL));
        OPCODE(0x2C, 4, 1, "INC L", _inc(Register::L));
        OPCODE(0x2D, 4, 1, "DEC L", _dec(Register::L));
        OPCODE(0x2E, 8, 2, "LD L,d8", _ld(Register::L, _d8()));
        OPCODE(0x2F, 8, 2, "CPL", _cpl());
        OPCODE(0x30, 8, 2, "JR NC,r8", _jr(!_flags.C, _r8()));
        OPCODE(0x31, 12, 3, "LD SP,d16", _ldw(Register::SP, _d16()));
        OPCODE(0x32, 8, 1, "LDD (HL), A", _ldi(_HL--, Register::A));
        OPCODE(0x33, 8, 1, "INC SP", _incw(Register::SP));
        OPCODE(0x34, 12, 1, "INC (HL)", _inc(Register::HL));
        OPCODE(0x35, 12, 1, "DEC (HL)", _dec(Register::HL));
        OPCODE(0x36, 8, 2, "LD (HL),d8", _ldi(_HL, _d8()));
        OPCODE(0x37, 4, 1, "SCF", _scf());
        OPCODE(0x38, 8 ,2, "JR C,r8", _jr(_flags.C, _r8()));
        OPCODE(0x39, 8, 1, "ADD HL,SP", _addw(Register::HL, _SP));
        OPCODE(0x3A, 8, 1, "LD A, (HL-)", _ld(Register::A, _mem[_HL--]));
        OPCODE(0x3B, 8, 1, "DEC SP", _decw(Register::SP));
        OPCODE(0x3C, 4, 1, "INC A", _inc(Register::A));
        OPCODE(0x3D, 4, 1, "DEC A", _dec(Register::A));
        OPCODE(0x3E, 8, 2, "LD A,d8", _ld(Register::A, _d8()));
        OPCODE(0x3F, 4, 1, "CCF", _ccf());
        OPCODE(0x40, 4, 1, "LD B,B", _ld(Register::B, _B));
        OPCODE(0x41, 4, 1, "LD B,C", _ld(Register::B, _C));
        OPCODE(0x42, 4, 1, "LD B,D", _ld(Register::B, _D));
        OPCODE(0x43, 4, 1, "LD B,E", _ld(Register::B, _E));
        OPCODE(0x44, 4, 1, "LD B,H", _ld(Register::B, _H));
        OPCODE(0x45, 4, 1, "LD B,L", _ld(Register::B, _L));
        OPCODE(0x46, 4, 1, "LD B,(HL)", _ld(Register::B, _fetch(Register::HL)));
        OPCODE(0x47, 4, 1, "LD B,A", _ld(Register::B, _A));
        OPCODE(0x48, 4, 1, "LD C,B", _ld(Register::C, _B));
        OPCODE(0x49, 4, 1, "LD C,C", _ld(Register::C, _C));
        OPCODE(0x4A, 4, 1, "LD C,D", _ld(Register::C, _D));
        OPCODE(0x4B, 4, 1, "LD C,E", _ld(Register::C, _E));
        OPCODE(0x4C, 4, 1, "LD C,H", _ld(Register::C, _H));
        OPCODE(0x4D, 4, 1, "LD C,L", _ld(Register::C, _L));
        OPCODE(0x4E, 4, 1, "LD C,(HL)", _ld(Register::C, _fetch(Register::HL)));
        OPCODE(0x4F, 4, 1, "LD C,A", _ld(Register::C, _A));
        OPCODE(0x50, 4, 1, "LD D,B", _ld(Register::D, _B));
        OPCODE(0x51, 4, 1, "LD D,C", _ld(Register::D, _C));
        OPCODE(0x52, 4, 1, "LD D,D", _ld(Register::D, _D));
        OPCODE(0x53, 4, 1, "LD D,E", _ld(Register::D, _E));
        OPCODE(0x54, 4, 1, "LD D,H", _ld(Register::D, _H));
        OPCODE(0x55, 4, 1, "LD D,L", _ld(Register::D, _L));
        OPCODE(0x56, 4, 1, "LD D,(HL)", _ld(Register::D, _fetch(Register::HL)));
        OPCODE(0x57, 4, 1, "LD D,A", _ld(Register::D, _A));
        OPCODE(0x58, 4, 1, "LD E,B", _ld(Register::E, _B));
        OPCODE(0x59, 4, 1, "LD E,C", _ld(Register::E, _C));
        OPCODE(0x5A, 4, 1, "LD E,D", _ld(Register::E, _D));
        OPCODE(0x5B, 4, 1, "LD E,E", _ld(Register::E, _E));
        OPCODE(0x5C, 4, 1, "LD E,H", _ld(Register::E, _H));
        OPCODE(0x5D, 4, 1, "LD E,L", _ld(Register::E, _L));
        OPCODE(0x5E, 4, 1, "LD E,(HL)", _ld(Register::E, _fetch(Register::HL)));
        OPCODE(0x5F, 4, 1, "LD E,A", _ld(Register::E, _A));
        OPCODE(0x60, 4, 1, "LD H,B", _ld(Register::H, _B));
        OPCODE(0x61, 4, 1, "LD H,C", _ld(Register::H, _C));
        OPCODE(0x62, 4, 1, "LD H,D", _ld(Register::H, _D));
        OPCODE(0x63, 4, 1, "LD H,E", _ld(Register::H, _E));
        OPCODE(0x64, 4, 1, "LD H,H", _ld(Register::H, _H));
        OPCODE(0x65, 4, 1, "LD H,L", _ld(Register::H, _L));
        OPCODE(0x66, 4, 1, "LD H,(HL)", _ld(Register::H, _fetch(Register::HL)));
        OPCODE(0x67, 4, 1, "LD H,A", _ld(Register::H, _A));
        OPCODE(0x68, 4, 1, "LD L,B", _ld(Register::L, _B));
        OPCODE(0x69, 4, 1, "LD L,C", _ld(Register::L, _C));
        OPCODE(0x6A, 4, 1, "LD L,D", _ld(Register::L, _D));
        OPCODE(0x6B, 4, 1, "LD L,E", _ld(Register::L, _E));
        OPCODE(0x6C, 4, 1, "LD L,H", _ld(Register::L, _H));
        OPCODE(0x6D, 4, 1, "LD L,L", _ld(Register::L, _L));
        OPCODE(0x6E, 4, 1, "LD L,(HL)", _ld(Register::L, _fetch(Register::HL)));
        OPCODE(0x6F, 4, 1, "LD L,A", _ld(Register::L, _A));
        OPCODE(0x70, 4, 1, "LD (HL),B", _ldi(_HL, Register::B));
        OPCODE(0x71, 4, 1, "LD (HL),C", _ldi(_HL, Register::C));
        OPCODE(0x72, 4, 1, "LD (HL),D", _ldi(_HL, Register::D));
        OPCODE(0x73, 4, 1, "LD (HL),E", _ldi(_HL, Register::E));
        OPCODE(0x74, 4, 1, "LD (HL),H", _ldi(_HL, Register::H));
        OPCODE(0x75, 4, 1, "LD (HL),L", _ldi(_HL, Register::L));
        OPCODE(0x76, 4, 1, "HALT", _halt());
        OPCODE(0x77, 4, 1, "LD (HL),A", _ldi(_HL, Register::A));
        OPCODE(0x78, 4, 1, "LD A,B", _ld(Register::A, _B));
        OPCODE(0x79, 4, 1, "LD A,C", _ld(Register::A, _C));
        OPCODE(0x7A, 4, 1, "LD A,D", _ld(Register::A, _D));
        OPCODE(0x7B, 4, 1, "LD A,E", _ld(Register::A, _E));
        OPCODE(0x7C, 4, 1, "LD A,H", _ld(Register::A, _H));
        OPCODE(0x7D, 4, 1, "LD A,L", _ld(Register::A, _L));
        OPCODE(0x7E, 4, 1, "LD A,(HL)", _ld(Register::A, _fetch(Register::HL)));
        OPCODE(0x7F, 4, 1, "LD A,A", _ld(Register::A, _A));
        OPCODE(0x80, 4, 1, "ADD A,B", _add(Register::A, _B));
        OPCODE(0x81, 4, 1, "ADD A,C", _add(Register::A, _C));
        OPCODE(0x82, 4, 1, "ADD A,D", _add(Register::A, _D));
        OPCODE(0x83, 4, 1, "ADD A,E", _add(Register::A, _E));
        OPCODE(0x84, 4, 1, "ADD A,H", _add(Register::A, _H));
        OPCODE(0x85, 4, 1, "ADD A,L", _add(Register::A, _L));
        OPCODE(0x86, 4, 1, "ADD A,(HL)", _add(Register::A, _fetch(Register::HL)));
        OPCODE(0x87, 4, 1, "ADD A,A", _add(Register::A, _A));
        OPCODE(0x88, 8, 1, "ADC A,B", _adc(Register::A, _B));
        OPCODE(0x89, 8, 1, "ADC A,C", _adc(Register::A, _C));
        OPCODE(0x8A, 8, 1, "ADC A,D", _adc(Register::A, _D));
        OPCODE(0x8B, 8, 1, "ADC A,E", _adc(Register::A, _E));
        OPCODE(0x8C, 8, 1, "ADC A,H", _adc(Register::A, _H));
        OPCODE(0x8D, 8, 1, "ADC A,L", _adc(Register::A, _L));
        OPCODE(0x8E, 8, 1, "ADC A,(HL)", _adc(Register::A, _fetch(Register::HL)));
        OPCODE(0x8F, 8, 1, "ADC A,A", _adc(Register::A, _A));
        OPCODE(0x90, 4, 1, "SUB B", _sub(Register::A, _B));
        OPCODE(0x91, 4, 1, "SUB C", _sub(Register::A, _C));
        OPCODE(0x92, 4, 1, "SUB D", _sub(Register::A, _D));
        OPCODE(0x93, 4, 1, "SUB E", _sub(Register::A, _E));
        OPCODE(0x94, 4, 1, "SUB H", _sub(Register::A, _H));
        OPCODE(0x95, 4, 1, "SUB L", _sub(Register::A, _L));
        OPCODE(0x96, 4, 1, "SUB (HL)", _sub(Register::A, _fetch(Register::HL)));
        OPCODE(0x97, 4, 1, "SUB A", _sub(Register::A, _A));
        OPCODE(0x98, 8, 1, "SBC A,B", _sbc(Register::A, _B));
        OPCODE(0x99, 8, 1, "SBC A,C", _sbc(Register::A, _C));
        OPCODE(0x9A, 8, 1, "SBC A,D", _sbc(Register::A, _D));
        OPCODE(0x9B, 8, 1, "SBC A,E", _sbc(Register::A, _E));
        OPCODE(0x9C, 8, 1, "SBC A,H", _sbc(Register::A, _H));
        OPCODE(0x9D, 8, 1, "SBC A,L", _sbc(Register::A, _L));
        OPCODE(0x9E, 8, 1, "SBC A,(HL)", _sbc(Register::A, _fetch(Register::HL)));
        OPCODE(0x9F, 8, 1, "SBC A,A", _sbc(Register::A, _A));
        OPCODE(0xA0, 4, 1, "AND B", _and(Register::A, _B));
        OPCODE(0xA1, 4, 1, "AND C", _and(Register::A, _C));
        OPCODE(0xA2, 4, 1, "AND D", _and(Register::A, _D));
        OPCODE(0xA3, 4, 1, "AND E", _and(Register::A, _E));
        OPCODE(0xA4, 4, 1, "AND H", _and(Register::A, _H));
        OPCODE(0xA5, 4, 1, "AND L", _and(Register::A, _L));
        OPCODE(0xA6, 4, 1, "AND (HL)", _and(Register::A, _fetch(Register::HL)));
        OPCODE(0xA7, 4, 1, "AND A", _and(Register::A, _A));
        OPCODE(0xA8, 4, 1, "XOR B", _xor(Register::A, _B));
        OPCODE(0xA9, 4, 1, "XOR C", _xor(Register::A, _C));
        OPCODE(0xAA, 4, 1, "XOR D", _xor(Register::A, _D));
        OPCODE(0xAB, 4, 1, "XOR E", _xor(Register::A, _E));
        OPCODE(0xAC, 4, 1, "XOR H", _xor(Register::A, _H));
        OPCODE(0xAD, 4, 1, "XOR L", _xor(Register::A, _L));
        OPCODE(0xAE, 7, 2, "XOR (HL)", _xor(Register::A, _fetch(Register::HL)));
        OPCODE(0xAF, 4, 1, "XOR A", _xor(Register::A, _A));
        OPCODE(0xB0, 4, 1, "OR B", _or(Register::A, _B));
        OPCODE(0xB1, 4, 1, "OR C", _or(Register::A, _C));
        OPCODE(0xB2, 4, 1, "OR D", _or(Register::A, _D));
        OPCODE(0xB3, 4, 1, "OR E", _or(Register::A, _E));
        OPCODE(0xB4, 4, 1, "OR H", _or(Register::A, _H));
        OPCODE(0xB5, 4, 1, "OR L", _or(Register::A, _L));
        OPCODE(0xB6, 4, 1, "OR (HL)", _or(Register::A, _fetch(Register::HL)));
        OPCODE(0xB7, 4, 1, "OR A", _or(Register::A, _A));
        OPCODE(0xB8, 4, 1, "CP B", _cp(_A, _B));
        OPCODE(0xB9, 4, 1, "CP C", _cp(_A, _C));
        OPCODE(0xBA, 4, 1, "CP D", _cp(_A, _D));
        OPCODE(0xBB, 4, 1, "CP E", _cp(_A, _E));
        OPCODE(0xBC, 4, 1, "CP H", _cp(_A, _H));
        OPCODE(0xBD, 4, 1, "CP L", _cp(_A, _L));
        OPCODE(0xBE, 4, 1, "CP (HL)", _cp(_A, _fetch(Register::HL)));
        OPCODE(0xBF, 4, 1, "CP A", _cp(_A, _A));
        OPCODE(0xC0, 8, 1, "RET NZ", _ret(!_flags.Z));
        OPCODE(0xC1, 12, 1, "POP BC", _pop(_B, _C));
        OPCODE(0xC2, 12, 0, "JP NZ", _jp(!_flags.Z, _d16()));
        OPCODE(0xC3, 12, 0, "JP", _jp(true, _d16()));
        OPCODE(0xC4, 12, 3, "CALL NZ,a16", _call(!_flags.Z, _d16()));
        OPCODE(0xC5, 16, 1, "PUSH BC", _push(_B, _C));
        OPCODE(0xC6, 8, 2, "ADD a,d8", _add(Register::A, _d8()));
        OPCODE(0xC7, 16, 1, "RST 00H", _rst(0x00));
        OPCODE(0xC8, 8, 1, "RET Z", _ret(_flags.Z));
        OPCODE(0xC9, 4, 1, "RET", _ret(true));
        OPCODE(0xCA, 12, 3, "JP Z,a16", _jp(_flags.Z, _d16()));
        OPCODE(0xCB, 16, 1, "PREFIX CB", _cb(_d8()));
        OPCODE(0xCC, 12, 3, "CALL Z,a16", _call(_flags.Z, _d16()));
        OPCODE(0xCD, 24, 3, "CALL a16", _call(true, _d16()));
        OPCODE(0xCE, 8, 2, "ADC a,d8", _adc(Register::A, _d8()));
        OPCODE(0xCF, 16, 1, "RST 08H", _rst(0x08));
        OPCODE(0xD0, 8, 1, "RET NC", _ret(!_flags.C));
        OPCODE(0xD1, 12, 1, "POP DE", _pop(_D, _E));
        OPCODE(0xD2, 12, 0, "JP NC", _jp(!_flags.C, _d16()));
        OPCODE(0xD4, 12, 3, "CALL NC,a16", _call(!_flags.C, _d16()));
        OPCODE(0xD5, 16, 1, "PUSH DE", _push(_D, _E));
        OPCODE(0xD6, 8, 2, "SUB a,d8", _sub(Register::A, _d8()));
        OPCODE(0xD7, 16, 1, "RST 10H", _rst(0x10));
        OPCODE(0xD8, 8, 1, "RET C", _ret(_flags.C));
        OPCODE(0xD9, 16, 1, "RETI", _ime = IME::Shadow; _ret(true));
        OPCODE(0xDA, 12, 3, "JP C,a16", _jp(_flags.C, _d16()));
        OPCODE(0xDC, 12, 3, "CALL C, a16", _call(_flags.C, _d16()));
        OPCODE(0xDE, 8, 2, "SBC a,d8", _sbc(Register::A, _d8()));
        OPCODE(0xDF, 16, 1, "RST 18H", _rst(0x18));
        OPCODE(0xE0, 12, 2, "LDH (a8),A", _ldi(_a8(), Register::A));
        OPCODE(0xE1, 12, 1, "POP HL", _pop(_H, _L));
        OPCODE(0xE2, 8, 2, "LD (C), A", _ldi(0xff00 + _C, Register::A));
        OPCODE(0xE5, 16, 1, "PUSH HL", _push(_H, _L));
        OPCODE(0xE6, 8, 2, "AND d8", _and(Register::A, _d8()));
        OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
        OPCODE(0xE8, 16, 2, "ADD SP, r8", _ldw(Register::SP, _SP + _r8()));
        OPCODE(0xE9, 4, 1, "JP (HL)", _jp(true, _HL));
        OPCODE(0xEA, 16, 3, "LD (a16),A", _ldi(_d16(), Register::A));
        OPCODE(0xEE, 16, 1, "XOR d8", _xor(Register::A, _d8()));
        OPCODE(0xEF, 16, 1, "RST 28H", _rst(0x28));
        OPCODE(0xF0, 12, 2, "LDH A,(a8)", _ld(Register::A, _mem[_a8()]));
        OPCODE(0xF1, 12, 1, "POP AF", _pop(_A, _F); _F &= 0xf0;);
        OPCODE(0xF2, 8, 2, "LD A, (C)", _ld(Register::A, _mem[0xff00 + _C]));
        OPCODE(0xF3, 4, 1, "DI", _ime = IME::Disabled;);
        OPCODE(0xF5, 16, 1, "PUSH AF", _push(_A, _F));
        OPCODE(0xF6, 8, 2, "OR d8", _or(Register::A, _d8()));
        OPCODE(0xF7, 16, 1, "RST 30H", _rst(0x30));
        OPCODE(0xF8, 12, 2, "LD HL,SP+r8", _ldw(Register::HL, _SP + _r8()));
        OPCODE(0xF9, 8, 1, "LD SP,HL", _ldw(Register::SP, _HL));
        OPCODE(0xFa, 16, 3, "LD A,(a16)", _ld(Register::A, _mem[_d16()]));
        OPCODE(0xFB, 4, 1, "EI", _ime = IME::Shadow;);
        OPCODE(0xFE, 8, 2, "CP d8", _cp(_A, _d8()));
        OPCODE(0xFF, 16, 1, "RST 38H", _rst(0x38));
    default:
        std::cout << "Unknown opcode: " << Print(op) << std::endl;
        throw OpcodeException(Opcode(op));
    }
}

void Cpu::step(void)
{
    _handle_interrupts();

    switch (_state) {
    case State::Running:
        _op();
        break;
    case State::Halted:
        _tick(4);
        break;
    case State::Stopped:
        // XXX: Should we disable the screen?
        break;
    }

    video();
    timer();

}

void Cpu::set(Register r, wreg_t value)
{
    switch (r) {
    case Register::A: _A = value; break;
    case Register::F: _F = value; break;
    case Register::B: _B = value; break;
    case Register::C: _C = value; break;
    case Register::D: _D = value; break;
    case Register::E: _E = value; break;
    case Register::H: _H = value; break;
    case Register::L: _L = value; break;
    case Register::SP: _SP = value; break;
    case Register::PC: _PC = value; break;
    case Register::AF: _AF = value; break;
    case Register::BC: _BC = value; break;
    case Register::DE: _DE = value; break;
    case Register::HL: _HL = value; break;
    }
}

wreg_t Cpu::get(Register r)
{
    switch (r) {
    case Register::A: return _A; break;
    case Register::F: return _F; break;
    case Register::B: return _B; break;
    case Register::C: return _C; break;
    case Register::D: return _D; break;
    case Register::E: return _E; break;
    case Register::H: return _H; break;
    case Register::L: return _L; break;
    case Register::SP: return _SP; break;
    case Register::PC: return _PC; break;
    case Register::AF: return _AF; break;
    case Register::BC: return _BC; break;
    case Register::DE: return _DE; break;
    case Register::HL: return _HL; break;
    }
}

reg_t Cpu::get(addr_t addr)
{
    return _mem[addr];
}

void Cpu::set(addr_t addr, reg_t value)
{
    _mem[addr] = value;
}

static inline reg_t op_value(const Opcode &obj)
{
    return static_cast<std::underlying_type<Opcode>::type>(obj);
}

static inline std::ostream& operator << (std::ostream& os, const Opcode& obj)
{
    os << std::hex << "0x" << std::setw(2) << std::setfill('0')
       << int(op_value(obj));
    return os;
}

void Cpu::to_string(std::ostream &os) const
{
    os << "A: " << Print(_A) << " F: " << Print(_F)
       << " B: " << Print(_B) << " C: " << Print(_C)
       << " D: " << Print(_D) << " E: " << Print(_E) << std::endl
       << "HL: " << Print(_HL)
       << " PC: " << Print(_PC)
       << " SP: " << Print(_SP) << std::endl;
    switch (_ime) {
    case IME::Disabled:
        os << "IME: Disabled";
        break;
    case IME::Shadow:
        os << "IME: Shadow";
        break;
    case IME::Enabled:
        os << "IME: Enabled";
        break;
    }
    // XXX: state
    os << std::endl;
}

reg_t Cpu::load(Opcode op)
{
    _mem[_PC++] = op_value(op);
    return 1;
}

reg_t Cpu::load(Opcode op, reg_t arg)
{
    _mem[_PC++] = op_value(op);
    _mem[_PC++] = arg;
    return 1;
}

reg_t Cpu::load(Opcode op, reg_t arg1, reg_t arg2)
{
    _mem[_PC++] = op_value(op);
    _mem[_PC++] = arg1;
    _mem[_PC++] = arg2;
    return 1;
}

void Cpu::set(const RegisterSet &registers)
{
    _A = registers.A;
    _B = registers.B;
    _C = registers.C;
    _D = registers.D;
    _E = registers.E;
    _HL = registers.HL;
    _SP = registers.SP;
}

void Cpu::test_setup(const RegisterSet &registers,
                     const reg_t *code, addr_t len)
{
    set(registers);

    memcpy(&_mem[0], code, len);
    _PC = len;
}

void Cpu::test_step(unsigned steps)
{
    _PC = 0;
    for (unsigned i = 0; i < steps; i++)
        step();
}

void Cpu::_dma(reg_t value)
{
    addr_t dest = Mem::OAMTable;
    addr_t src = (addr_t)value << 8;
    memcpy(&_mem[dest], &_mem[src], 40*4);
}

void Cpu::_write(addr_t addr, reg_t value)
{
    if (addr == 0xFFFF) {
        // Interrupt
    } else if (addr >= 0xFF80) {
        // Internal RAM
    } else if (addr >= 0xFF00) {
        switch (addr) {
        case CtrlReg::KEYS:
            // copy current keys
            if ((value & 0x10) == 0) {
                value &= 0xF0;
                value |= ~(_keys >> 4) & 0x0F;
            } else if ((value & 0x20) == 0) {
                value &= 0xF0;
                value |= ~_keys & 0x0F;
            }
            break;
        case CtrlReg::DMG_RESET:
            // Nuke the DMG ROM
            memcpy(&_mem[0], _rom, 256);
            break;
        case CtrlReg::DMA:
            // Start the DMA transfer
            // XXX: Should we actually delay it?
            _dma(value);
            break;
        }
    } else if (addr >= 0xFF00) {
        // I/O
    } else if (addr >= 0xFEA0) {
        // Empty I/O
    } else if (addr >= 0xFE00) {
        // Sprite Attrib Memory
    } else if (addr >= 0xE000) {
        // Echo of 8kb Internal RAM
    } else if (addr >= 0xC000) {
        // Internal RAM
        // XXX: his should be 0xDE00
        if (addr <= 0xCE00)
            _mem[addr + 0x2000] = value;
    } else if (addr >= 0xA000) {
        // switchable RAM bank
    } else if (addr >= 0x8000) {
        // Video RAM
    } else if (addr >= 0x6000) {
        return;
    } else if (addr >= 0x4000) {
        return;
    } else if (addr >= 0x2000) {
        value &= 0x1f;
        if (value == 0)
            value = 1;
        unsigned rom_addr = value * 0x4000;
        if (_debug)
            std::cout << " loading " << Print(rom_addr);
        if (rom_addr < _rom_len)
            memcpy(&_mem[0x4000], &_rom[rom_addr], 0x4000);
        return;
    } else {
        return;
    }
    _mem[addr] = value;
}

/*
 * ____
 *|  _ \ _ __ ___   __ _ _ __ __ _ _ __ ___
 *| |_) | '__/ _ \ / _` | '__/ _` | '_ ` _ \
 *|  __/| | | (_) | (_| | | | (_| | | | | | |
 *|_|   |_|  \___/ \__, |_|  \__,_|_| |_| |_|
 *                 |___/
 */
reg_t default_stack[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0xBF, 0xF3, 0x00, 0xBF, 0x00, 0x3F, 0x00, 0x00, 0xBF, 0x7F, 0xFF, 0x9F, 0x00, 0xBF, 0x00,
    0xFF, 0x00, 0x00, 0x00, 0x77, 0xF3, 0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* rest is zero */
};
void Cpu::reset(bool logo)
{
    _A = 0x01;
    _F = 0xB0;
    _B = 0x00;
    _C = 0x13;
    _D = 0x00;
    _E = 0xD8;
    _H = 0x01;
    _L = 0x4D;
    _SP = 0xFFFE;
    _PC = 0x0000;

    memset(_mem, 0, sizeof(_mem));

    memcpy(&_mem[0xFF00], default_stack, sizeof(default_stack));

    if (_rom == NULL)
        load_rom(CURRENT_ROM, sizeof(CURRENT_ROM));
    memcpy(&_mem[0x0000], _rom, 32*1024);

    if (logo)
        memcpy(&_mem[0x0000], dmg_rom, sizeof(dmg_rom));
    else
        _PC = 0x0100;

    // Cartridge Header
    _name.clear();
    for (unsigned i = 0x0134; i < 0x0142; i++)
        _name += _mem[i];

    if ((_mem[0x0143] & 0xC0) == 0xC0)
        std::cout << "Color Gameboy Only" << std::endl;

    _type = static_cast<Cartridge>(_mem[0x0147]);
    _rom_size = static_cast<RomSize>(_mem[0x0148]);
    _ram_size = static_cast<RamSize>(_mem[0x0149]);

    std::cout << "Cartridge Header" << std::endl;
    std::cout << "Name: " << _name << std::endl;
    std::cout << "MBC: " << Print(_type) << std::endl;
    std::cout << "Rom Size: " << Print(_rom_size) << std::endl;
}

void Cpu::load_rom(const reg_t *rom, unsigned len)
{
    if (_rom != NULL)
        delete[] _rom;
    _rom = new reg_t[len];
    _rom_len = len;
    memcpy(_rom, rom, _rom_len);
}

void Cpu::load_rom(const std::string &name)
{
    std::cout << "Loading: " << name << std::endl;
    struct stat sb;
    if (stat(name.c_str(), &sb) == -1)
        throw RomException(name);
    _rom_len = sb.st_size;
    if (_rom != NULL)
        delete[] _rom;
    _rom = new reg_t[_rom_len];
    try {
        std::ifstream file(name, std::ios::in | std::ios::binary);
        file.read((char *)_rom, _rom_len);
    } catch (std::ifstream::failure e) {
        throw RomException(name);
    }
}

/*
 * __     ___     _
 * \ \   / (_) __| | ___  ___
 *  \ \ / /| |/ _` |/ _ \/ _ \
 *   \ V / | | (_| |  __/ (_) |
 *    \_/  |_|\__,_|\___|\___/
 */

// 201 - 207
#define H_BLANK_CYCLES 201
// 77 - 83
#define OAM_CYCLES (77 + H_BLANK_CYCLES)
// 169 - 175
#define ACTIVE_CYCLES (169 + OAM_CYCLES)
// 4560
#define V_BLANK_CYCLES 4560

#define SCANLINE_CYCLES 456

#define SCANLINES 156
#define DISPLAY_LINES 144

addr_t InterruptVector[] = {
    0x40, /* Interrupt::VBlank */
    0x48, /* Interrupt::LCDStat */
    0x50, /* Interrupt::Timer */
    0x58, /* Interrupt::Serial */
    0x60, /* Interrupt::Joypad */
};

void Cpu::_handle_interrupts(void)
{
    reg_t flags = _mem[CtrlReg::IF];
    if (_ime == IME::Disabled) {
        if (_state == State::Halted)
            for (unsigned i = 0; i < 5; i++)
                if (bit_isset(flags, i))
                    _state = State::Running;
        return;
    }
    if (_ime == IME::Shadow) {
        _ime = IME::Enabled;
        return;
    }
    reg_t enabled = _mem[CtrlReg::IE];
    for (unsigned i = 0; i < 5; i++) {
        if (bit_isset(flags, i) && bit_isset(enabled, i)) {
            _push(_PCh, _PCl);
            _PC = InterruptVector[i];
            if (_debug)
                std::cout << " Interrupt Triggered: " << i << std::endl;
            bit_set(_mem[CtrlReg::IF], i, false);
            _tick(20);
            _ime = IME::Disabled;
            _state = State::Running;
            return;
        }
    }
}

void Cpu::async_render(void)
{
    if (_render == NULL)
        return;

    (*_render)(_mem);
}

static inline void empty()
{
}

#include "time.h"

void Cpu::video(void)
{
    reg_t &stat = _mem[CtrlReg::STAT];
    reg_t ly = _mem[CtrlReg::LY];
    static unsigned start = 0;

    // See if we've moved scanlines
    if (_fcycles > SCANLINE_CYCLES) {
        _fcycles = 0;
        ly = (ly + 1) % SCANLINES;
        _write(CtrlReg::LY, ly);

        switch (ly) {
        case 0: {
            unsigned end = clock();
            start = end;
            break;
        }
        case DISPLAY_LINES:
            // Wait until our frame is finished
            async_render();
            bit_set(_mem[CtrlReg::IF], Interrupt::VBlank, true);
            break;
        }
    }

    // Handle the LCD Status register
    if (ly < DISPLAY_LINES) {
        if (_fcycles < H_BLANK_CYCLES) {
            stat = (stat & 0xfc) | 0x00;
        } else if (_fcycles < OAM_CYCLES) {
            stat = (stat & 0xfc) | 0x10;
        } else if (_fcycles < ACTIVE_CYCLES) {
            stat = (stat & 0xfc) | 0x11;
        }
    } else {
        // We're in vblank
        stat = (stat & 0xfc) | 0x01;
    }
}

void Cpu::set_key(GBKey key, bool set)
{
    bit_set(_keys, static_cast<key_type>(key), set);
    if (_state == State::Stopped)
        _state = State::Running;
}

void Cpu::timer(void)
{
    if (_dcycles > 256) {
        _dcycles -= 256;
        _mem[CtrlReg::DIV]++;
        // Divider register triggerd
    }
    reg_t &tac = _mem[CtrlReg::TAC];
    if (tac & 0x04) {
        unsigned limit = 1024;
        switch (tac & 0x3) {
        case 0: limit = 1024; break;
        case 1: limit = 16; break;
        case 2: limit = 64; break;
        case 3: limit = 256; break;
        }
        if (_tcycles > limit) {
            reg_t &tima = _mem[CtrlReg::TIMA];
            _tcycles -= limit;
            if (tima == 0xff) {
                // Trigger the interrupt
                bit_set(_mem[CtrlReg::IF], Interrupt::Timer, true);
                // Reset the overflow
                tima = _mem[CtrlReg::TMA];
            } else
                tima++;
        }
    }
}
