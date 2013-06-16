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
#include <vector>
#include <algorithm>

#include "mbc.h"

using namespace DMG;

#define DEFAULT_ROM (32*1024)
#define MEM_SIZE (64*1024)

Cpu::Cpu(void): _rAF(0), _rBC(0), _rDE(0), _rHL(0), _rSP(0), _rPC(0),
    _ime(IME::Disabled), _state(State::Running),
    _icycles(0), _cycles(0),
    _video(NULL),
    _boot_rom(NULL),
    _bus(),
    _clock(&_bus),
    _debug(false)
{
    _bus.add_map(this);
    _bus.add_map(&_clock);
}

Cpu::~Cpu(void)
{
    if (_boot_rom != NULL) {
        delete _boot_rom;
        _boot_rom = NULL;
    }
}

#define EQ(a, b) \
    if (a != b) { \
        std::cout << std::hex << "Mismatch '" #a "' Expected: " << Print(a) \
                  << " Actual: " << Print(b) << std::endl; \
        return false; \
    }

bool Cpu::operator == (const Cpu &rhs) const
{
    EQ(_rA, rhs._rA);
    EQ(_rF, rhs._rF);
    EQ(_rB, rhs._rB);
    EQ(_rC, rhs._rC);
    EQ(_rD, rhs._rD);
    EQ(_rE, rhs._rE);
    EQ(_rH, rhs._rH);
    EQ(_rL, rhs._rL);
    EQ(_rSP, rhs._rSP);
    EQ(_rPC, rhs._rPC);
#if 0
    for (unsigned i = 0; i < _mem.size(); i++)
        if (_read(i) != rhs._read(i)) {
            std::cout << "Memory differences at: " << Print(i) << std::endl;
            EQ(_read(i), rhs._read(i));
        }
#endif
    return true;
}

bool
Cpu::valid(addr_t addr)
{
    switch (addr) {
    case CtrlReg::IF:
    case CtrlReg::IE:
        return true;
    }
    return false;
}

void
Cpu::write(addr_t addr, byte_t value)
{
    switch (addr) {
    case CtrlReg::IF:
        // XXX: Handle interrupts
        _IF = value;
        break;
    case CtrlReg::IE:
        _IE = value;
        break;
    }
}

byte_t
Cpu::read(addr_t addr)
{
    switch (addr) {
    case CtrlReg::IF:
        return _IF;
    case CtrlReg::IE:
        return _IE;
    }
    return 0;
}

void Cpu::reset(void)
{
    _rA = 0x01;
    _rF = 0xB0;
    _rB = 0x00;
    _rC = 0x13;
    _rD = 0x00;
    _rE = 0xD8;
    _rH = 0x01;
    _rL = 0x4D;
    _rSP = 0xFFFE;
    _rPC = 0x0000;
    _ime = IME::Disabled;
    _state = State::Running;
    _IF = 0x00;
    _IE = 0x00;
}


/*
 *  ___                       _   _
 * / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___
 *| | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __|
 *| |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \
 * \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/
 *      |_|
 */

byte_t Cpu::_fetch(Register reg)
{
    switch (reg) {
        case Register::A: return _rA;
        case Register::B: return _rB;
        case Register::C: return _rC;
        case Register::D: return _rD;
        case Register::E: return _rE;
        case Register::H: return _rH;
        case Register::L: return _rL;
        case Register::HL: return _read(_rHL);
        default: throw CpuException();
    }
    throw CpuException();
}

void Cpu::_store(Register reg, byte_t value)
{
    switch (reg) {
        case Register::A: _rA = value; break;
        case Register::B: _rB = value; break;
        case Register::C: _rC = value; break;
        case Register::D: _rD = value; break;
        case Register::E: _rE = value; break;
        case Register::H: _rH = value; break;
        case Register::L: _rL = value; break;
        case Register::HL: _write(_rHL, value); break;
        default: throw CpuException();
    }
}

void Cpu::_add(byte_t &dest, byte_t arg)
{
    word_t result = dest + arg;

    _set_hflag(dest, arg, result);
    _set_cflag(dest, arg, result);
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_inc(byte_t &dest)
{
    word_t result = dest + 1;

    _set_hflag(dest, 1, result);
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_inci(addr_t addr)
{
    byte_t dest = _read(addr);

    _inc(dest);

    _write(addr, dest);
}

void Cpu::_adc(byte_t &dest, byte_t arg)
{
    word_t result = dest + arg + _flags.C;

    _set_hflag(dest, arg, result);
    _set_cflag(dest, arg, result);
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_sub(byte_t &dest, byte_t arg)
{
    word_t result = dest - arg;

    _set_hflag(dest, arg, result);
    _set_cflag(dest, arg, result);
    _set_zflag(result);
    _set_nflag(true);

    dest = result;
}

void Cpu::_dec(byte_t &dest)
{
    byte_t result = dest - 1;

    _set_hflag(dest, 1, result);
    _set_zflag(result);
    _set_nflag(true);

    dest = result;
}

void Cpu::_deci(addr_t addr)
{
    byte_t dest = _read(addr);

    _dec(dest);

    _write(addr, dest);
}

void Cpu::_sbc(byte_t &dest, byte_t arg)
{
    byte_t carry = _flags.C;
    byte_t result = dest - arg - carry;

    _flags.C = (arg > dest - carry) ? true : false;
    _flags.H = ((arg & 0x0f) + carry > (dest & 0x0f)) ? true : false;
    _set_zflag(result);
    _set_nflag(true);

    dest = result;
}

void Cpu::_cp(byte_t dest, byte_t arg)
{
    if (_debug)
        std::cout << "   " << Print(dest) << "<->" << Print(arg);

    _flags.Z = (dest == arg);
    _flags.C = (arg > dest);
    _flags.H = ((arg & 0x0f) > (dest & 0x0f)) ? true : false;
    _flags.N = 1;
}

void Cpu::_and(byte_t &dest, byte_t arg)
{
    byte_t result = dest & arg;

    _flags.C = 0;
    _flags.H = 1;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_xor(byte_t &dest, byte_t arg)
{
    byte_t result = dest ^ arg;

    _flags.C = 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_or(byte_t &dest, byte_t arg)
{
    byte_t result = dest | arg;

    _flags.C = 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_ld(byte_t &dest, byte_t arg)
{
    byte_t result = arg;

    dest = result;
}

void Cpu::_bit(byte_t dest, int bit)
{
    byte_t result = dest & (1 << bit);

    _flags.H = 1;
    _set_zflag(result);
    _set_nflag(false);
}

void Cpu::_reset(byte_t &dest, int bit)
{
    byte_t result = dest & ~(1 << bit);

    dest = result;
}

void Cpu::_set(byte_t &dest, int bit)
{
    byte_t result = dest | (1 << bit);

    dest = result;
}

void Cpu::_rl(byte_t &dest)
{
    byte_t result = (dest << 1) | _flags.C;

    _flags.C = (dest & 0x80) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_rla(void)
{
    _rl(_rA);
    _flags.Z = 0;
}

void Cpu::_rlc(byte_t &dest)
{
    byte_t result = (dest << 1) | ((dest & 0x80) >> 7);

    _flags.C = (dest & 0x80) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_rlca(void)
{
    _rlc(_rA);
    _flags.Z = 0;
}

void Cpu::_rr(byte_t &dest)
{
    byte_t result = (dest >> 1) | (_flags.C ? 0x80 : 0x00);

    _flags.C = (dest & 0x01) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_rra(void)
{
    _rr(_rA);
    _flags.Z = 0;
}

void Cpu::_rrc(byte_t &dest)
{
    byte_t result = (dest >> 1) | (dest & 0x01 ? 0x80 : 0x00);

    _flags.C = (dest & 0x01) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_rrca(void)
{
    _rrc(_rA);
    _flags.Z = 0;
}

void Cpu::_sla(byte_t &dest)
{
    byte_t result = (dest << 1);

    _flags.C = (dest & 0x80) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_sra(byte_t &dest)
{
    byte_t result = (dest >> 1) | (dest & 0x80);

    _flags.C = (dest & 0x01) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_srl(byte_t &dest)
{
    byte_t result = (dest >> 1);

    _flags.C = (dest & 0x01) != 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::_swap(byte_t &dest)
{
    byte_t result = (dest >> 4) | (dest << 4);

    _flags.C = 0;
    _flags.H = 0;
    _set_zflag(result);
    _set_nflag(false);

    dest = result;
}

void Cpu::prefix_cb(void)
{
    byte_t op = _d8();
    int bit = (op & 0x38) >> 3;
    byte_t dest = _fetch(Register(op & 0x07));
    if ((op & 0xC0) == 0x00) {
        if ((op & 0xF8) == 0x00) {
            _rlc(dest);
        } else if ((op & 0xF8) == 0x08) {
            _rrc(dest);
        } else if ((op & 0xF8) == 0x10) {
            _rl(dest);
        } else if ((op & 0xF8) == 0x18) {
            _rr(dest);
        } else if ((op & 0xF8) == 0x20) {
            _sla(dest);
        } else if ((op & 0xF8) == 0x28) {
            _sra(dest);
        } else if ((op & 0xF8) == 0x30) {
            _swap(dest);
        } else if ((op & 0xF8) == 0x38) {
            _srl(dest);
        }
        _store(Register(op & 0x07), dest);
    } else if ((op & 0xC0) == 0x40) {
        _bit(dest, bit);
    } else if ((op & 0xC0) == 0x80) {
        _reset(dest, bit);
        _store(Register(op & 0x07), dest);
    } else if ((op & 0xC0) == 0xC0) {
        _set(dest, bit);
        _store(Register(op & 0x07), dest);
    }
}

void Cpu::_rst(byte_t arg)
{
    _push(_rPCh, _rPCl);
    _rPC = arg;
}

void Cpu::_jr(bool jump, sbyte_t arg)
{
    if (jump) {
        _rPC += (char)arg;
        _tick(4);
    }
}

void Cpu::_jp(bool jump, word_t arg)
{
    if (jump) {
        if (_debug)
            std::cout << "  <- " << Print(arg);
        _rPC = arg;
        _tick(4);
    }
}

void Cpu::_call(bool jump, word_t addr)
{
    if (jump) {
        _push(_rPCh, _rPCl);

        if (_debug) {
            std::cout << "  <- " << Print(addr);
        }
        _rPC = addr;
        _tick(12);
    }
}

void Cpu::_ret(bool jump)
{
    if (jump) {
        _pop(_rPCh, _rPCl);
        _tick(16);
    }
}

void Cpu::_push(byte_t high, byte_t low)
{
    if (_debug) {
        word_t arg = high;
        arg = arg << 8 | low;
        std::cout << " <- " << Print(arg);
    }

    _write(--_rSP, high);
    _write(--_rSP, low);
}

void Cpu::_pop(byte_t &high, byte_t &low)
{
    low = _read(_rSP++);
    high = _read(_rSP++);
    if (_debug) {
        word_t arg = high;
        arg = arg << 8 | low;
        std::cout << " -> " << Print(arg);
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
    _dump_reg(std::cout);
    _state = State::Stopped;
}

void Cpu::dump(void)
{

    _dump_reg(std::cout);
}

word_t Cpu::_fetchw(Register reg)
{
    switch (reg) {
        case Register::BC: return _rBC;
        case Register::DE: return _rDE;
        case Register::HL: return _rHL;
        case Register::PC: return _rPC;
        case Register::SP: return _rSP;
        default: throw CpuException();
    }
    throw CpuException();
}

void Cpu::_addw(word_t &wdest, word_t arg)
{
    word_t result = wdest + arg;

    _flags.C = (result < arg) ? true : false;
    _flags.H = ((result & 0xfff) < (arg & 0xfff)) ? true : false;
    _set_nflag(false);

    wdest = result;
}

void Cpu::_addsp(sbyte_t arg)
{
    word_t &wdest = _rSP;
    word_t result = wdest + arg;

    _set_cflag(_rSP, arg, result);
    _set_hflag(_rSP, arg, result);
    _flags.Z = 0;
    _set_nflag(false);

    wdest = result;
}

void Cpu::_ldhlsp(sbyte_t arg)
{
    word_t &wdest = _rHL;
    word_t result = _rSP + arg;

    _set_cflag(_rSP, arg, result);
    _set_hflag(_rSP, arg, result);
    _flags.Z = 0;
    _set_nflag(false);

    wdest = result;
}

void Cpu::_incw(word_t &wdest)
{
    word_t result = wdest + 1;

    wdest = result;
}

void Cpu::_subw(word_t &wdest, word_t arg)
{
    word_t result = wdest - arg;

    wdest = result;
}

void Cpu::_decw(word_t &wdest)
{
    word_t result = wdest - 1;

    wdest = result;
}

void Cpu::_ldw(word_t &wdest, word_t arg)
{
    wdest = arg;
}

void Cpu::_ldi(addr_t addr, byte_t arg)
{
    if (_debug)
        std::cout << " -> " << Print(addr);

    _write(addr, arg);
}

void Cpu::_ldwi(addr_t addr, word_t arg)
{
    if (_debug)
        std::cout << " -> " << Print(addr) << " <- " << Print(arg);

    _write(addr, arg & 0xff);
    _write(addr+1, arg >> 8);
}

void Cpu::_cpl(void)
{
    byte_t &dest = _rA;
    byte_t result = ~dest;

    _flags.H = 1;
    _flags.N = 1;

    dest = result;
}

void Cpu::_ccf(void)
{
    _flags.C = !_flags.C;
    _flags.H = 0;
    _flags.N = 0;
}

void Cpu::_scf(void)
{
    _flags.H = 0;
    _flags.C = 1;
    _flags.N = 0;
}

void Cpu::_daa(void)
{
    byte_t &dest = _rA;
    word_t arg = 0;
    word_t result = dest;

    if (!_flags.N) {
        if (_flags.H || (dest & 0x0f) > 9) arg += 0x06;
        if (_flags.C || dest > 0x99) arg += 0x60;
        result += arg;
    } else {
        if (_flags.H) arg += 0x6;
        if (_flags.C) arg += 0x60;
        result -= arg;
    }

    _set_cflag(dest, arg, result);
    _flags.H = 0;
    _set_zflag(result);

    dest = result;
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

void Cpu::dispatch(void)
{
    word_t pc = _rPC;
    byte_t op = _read(_rPC++);
    //Register src = Register(op & 0x7);
    //Register dest = Register((op & 0x38) >> 3);

    switch (op) {
        OPCODE(0x00, 4, 1, "NOP", );
        OPCODE(0x01, 10, 3, "LD BC,d16", _ldw(_rBC, _d16()));
        OPCODE(0x02, 8, 1, "LD (BC),A", _ldi(_rBC, _rA));
        OPCODE(0x03, 8, 1, "INC BC", _incw(_rBC));
        OPCODE(0x04, 4, 1, "INC B", _inc(_rB));
        OPCODE(0x05, 4, 1, "DEC B", _dec(_rB));
        OPCODE(0x06, 8, 2, "LD B,d8", _ld(_rB, _d8()));
        OPCODE(0x07, 4, 1, "RLCA", _rlca());
        OPCODE(0x08, 20, 3, "LD (a16), SP", _ldwi(_d16(), _rSP));
        OPCODE(0x09, 8, 1, "ADD HL,BC", _addw(_rHL, _rBC));
        OPCODE(0x0A, 8, 1, "LD A,(BC)", _ld(_rA, _read(_rBC)));
        OPCODE(0x0B, 8, 1, "DEC BC", _decw(_rBC));
        OPCODE(0x0C, 4, 1, "INC C", _inc(_rC));
        OPCODE(0x0D, 4, 1, "DEC C", _dec(_rC));
        OPCODE(0x0E, 8, 2, "LD C,d8", _ld(_rC, _d8()));
        OPCODE(0x0F, 4, 1, "RRCA", _rrca());
        OPCODE(0x10, 4, 2, "STOP", _stop());
        OPCODE(0x11, 10, 3, "LD DE,d16", _ldw(_rDE, _d16()));
        OPCODE(0x12, 8, 1, "LD (DE),A", _ldi(_rDE, _rA));
        OPCODE(0x13, 8, 1, "INC DE", _incw(_rDE));
        OPCODE(0x14, 4, 1, "INC D", _inc(_rD));
        OPCODE(0x15, 4, 1, "DEC D", _dec(_rD));
        OPCODE(0x16, 8, 2, "LD D,d8", _ld(_rD, _d8()));
        OPCODE(0x17, 4, 1, "RLA", _rla());
        OPCODE(0x18, 12, 2, "JR r8", _jr(true, _r8()));
        OPCODE(0x19, 8, 1, "ADD HL,DE", _addw(_rHL, _rDE));
        OPCODE(0x1A, 8, 1, "LD A,(DE)", _ld(_rA, _read(_rDE)));
        OPCODE(0x1B, 8, 1, "DEC DE", _decw(_rDE));
        OPCODE(0x1C, 4, 1, "INC E", _inc(_rE));
        OPCODE(0x1D, 4, 1, "DEC E", _dec(_rE));
        OPCODE(0x1E, 8, 2, "LD E,d8", _ld(_rE, _d8()));
        OPCODE(0x1F, 4, 1, "RRA", _rra());
        OPCODE(0x20, 8, 2, "JR NZ,r8", _jr(!_flags.Z, _r8()));
        OPCODE(0x21, 10, 3, "LD HL,d16", _ldw(_rHL, _d16()));
        OPCODE(0x22, 8, 1, "LDI (HL), A", _ldi(_rHL++, _rA));
        OPCODE(0x23, 8, 1, "INC HL", _incw(_rHL));
        OPCODE(0x24, 4, 1, "INC H", _inc(_rH));
        OPCODE(0x25, 4, 1, "DEC H", _dec(_rH));
        OPCODE(0x26, 8, 2, "LD H,d8", _ld(_rH, _d8()));
        OPCODE(0x27, 4, 1, "DAA", _daa());
        OPCODE(0x28, 8, 2, "JR Z,r8", _jr(_flags.Z, _r8()));
        OPCODE(0x29, 8, 1, "ADD HL,HL", _addw(_rHL, _rHL));
        OPCODE(0x2A, 8, 1, "LDI a, (HL)", _ld(_rA, _read(_rHL++)));
        OPCODE(0x2B, 8, 1, "DEC HL", _decw(_rHL));
        OPCODE(0x2C, 4, 1, "INC L", _inc(_rL));
        OPCODE(0x2D, 4, 1, "DEC L", _dec(_rL));
        OPCODE(0x2E, 8, 2, "LD L,d8", _ld(_rL, _d8()));
        OPCODE(0x2F, 8, 2, "CPL", _cpl());
        OPCODE(0x30, 8, 2, "JR NC,r8", _jr(!_flags.C, _r8()));
        OPCODE(0x31, 12, 3, "LD SP,d16", _ldw(_rSP, _d16()));
        OPCODE(0x32, 8, 1, "LDD (HL), A", _ldi(_rHL--, _rA));
        OPCODE(0x33, 8, 1, "INC SP", _incw(_rSP));
        OPCODE(0x34, 12, 1, "INC (HL)", _inci(_rHL));
        OPCODE(0x35, 12, 1, "DEC (HL)", _deci(_rHL));
        OPCODE(0x36, 8, 2, "LD (HL),d8", _ldi(_rHL, _d8()));
        OPCODE(0x37, 4, 1, "SCF", _scf());
        OPCODE(0x38, 8 ,2, "JR C,r8", _jr(_flags.C, _r8()));
        OPCODE(0x39, 8, 1, "ADD HL,SP", _addw(_rHL, _rSP));
        OPCODE(0x3A, 8, 1, "LD A, (HL-)", _ld(_rA, _read(_rHL--)));
        OPCODE(0x3B, 8, 1, "DEC SP", _decw(_rSP));
        OPCODE(0x3C, 4, 1, "INC A", _inc(_rA));
        OPCODE(0x3D, 4, 1, "DEC A", _dec(_rA));
        OPCODE(0x3E, 8, 2, "LD A,d8", _ld(_rA, _d8()));
        OPCODE(0x3F, 4, 1, "CCF", _ccf());
        OPCODE(0x40, 4, 1, "LD B,B", _ld(_rB, _rB));
        OPCODE(0x41, 4, 1, "LD B,C", _ld(_rB, _rC));
        OPCODE(0x42, 4, 1, "LD B,D", _ld(_rB, _rD));
        OPCODE(0x43, 4, 1, "LD B,E", _ld(_rB, _rE));
        OPCODE(0x44, 4, 1, "LD B,H", _ld(_rB, _rH));
        OPCODE(0x45, 4, 1, "LD B,L", _ld(_rB, _rL));
        OPCODE(0x46, 4, 1, "LD B,(HL)", _ld(_rB, _read(_rHL)));
        OPCODE(0x47, 4, 1, "LD B,A", _ld(_rB, _rA));
        OPCODE(0x48, 4, 1, "LD C,B", _ld(_rC, _rB));
        OPCODE(0x49, 4, 1, "LD C,C", _ld(_rC, _rC));
        OPCODE(0x4A, 4, 1, "LD C,D", _ld(_rC, _rD));
        OPCODE(0x4B, 4, 1, "LD C,E", _ld(_rC, _rE));
        OPCODE(0x4C, 4, 1, "LD C,H", _ld(_rC, _rH));
        OPCODE(0x4D, 4, 1, "LD C,L", _ld(_rC, _rL));
        OPCODE(0x4E, 4, 1, "LD C,(HL)", _ld(_rC, _read(_rHL)));
        OPCODE(0x4F, 4, 1, "LD C,A", _ld(_rC, _rA));
        OPCODE(0x50, 4, 1, "LD D,B", _ld(_rD, _rB));
        OPCODE(0x51, 4, 1, "LD D,C", _ld(_rD, _rC));
        OPCODE(0x52, 4, 1, "LD D,D", _ld(_rD, _rD));
        OPCODE(0x53, 4, 1, "LD D,E", _ld(_rD, _rE));
        OPCODE(0x54, 4, 1, "LD D,H", _ld(_rD, _rH));
        OPCODE(0x55, 4, 1, "LD D,L", _ld(_rD, _rL));
        OPCODE(0x56, 4, 1, "LD D,(HL)", _ld(_rD, _read(_rHL)));
        OPCODE(0x57, 4, 1, "LD D,A", _ld(_rD, _rA));
        OPCODE(0x58, 4, 1, "LD E,B", _ld(_rE, _rB));
        OPCODE(0x59, 4, 1, "LD E,C", _ld(_rE, _rC));
        OPCODE(0x5A, 4, 1, "LD E,D", _ld(_rE, _rD));
        OPCODE(0x5B, 4, 1, "LD E,E", _ld(_rE, _rE));
        OPCODE(0x5C, 4, 1, "LD E,H", _ld(_rE, _rH));
        OPCODE(0x5D, 4, 1, "LD E,L", _ld(_rE, _rL));
        OPCODE(0x5E, 4, 1, "LD E,(HL)", _ld(_rE, _read(_rHL)));
        OPCODE(0x5F, 4, 1, "LD E,A", _ld(_rE, _rA));
        OPCODE(0x60, 4, 1, "LD H,B", _ld(_rH, _rB));
        OPCODE(0x61, 4, 1, "LD H,C", _ld(_rH, _rC));
        OPCODE(0x62, 4, 1, "LD H,D", _ld(_rH, _rD));
        OPCODE(0x63, 4, 1, "LD H,E", _ld(_rH, _rE));
        OPCODE(0x64, 4, 1, "LD H,H", _ld(_rH, _rH));
        OPCODE(0x65, 4, 1, "LD H,L", _ld(_rH, _rL));
        OPCODE(0x66, 4, 1, "LD H,(HL)", _ld(_rH, _read(_rHL)));
        OPCODE(0x67, 4, 1, "LD H,A", _ld(_rH, _rA));
        OPCODE(0x68, 4, 1, "LD L,B", _ld(_rL, _rB));
        OPCODE(0x69, 4, 1, "LD L,C", _ld(_rL, _rC));
        OPCODE(0x6A, 4, 1, "LD L,D", _ld(_rL, _rD));
        OPCODE(0x6B, 4, 1, "LD L,E", _ld(_rL, _rE));
        OPCODE(0x6C, 4, 1, "LD L,H", _ld(_rL, _rH));
        OPCODE(0x6D, 4, 1, "LD L,L", _ld(_rL, _rL));
        OPCODE(0x6E, 4, 1, "LD L,(HL)", _ld(_rL, _read(_rHL)));
        OPCODE(0x6F, 4, 1, "LD L,A", _ld(_rL, _rA));
        OPCODE(0x70, 4, 1, "LD (HL),B", _ldi(_rHL, _rB));
        OPCODE(0x71, 4, 1, "LD (HL),C", _ldi(_rHL, _rC));
        OPCODE(0x72, 4, 1, "LD (HL),D", _ldi(_rHL, _rD));
        OPCODE(0x73, 4, 1, "LD (HL),E", _ldi(_rHL, _rE));
        OPCODE(0x74, 4, 1, "LD (HL),H", _ldi(_rHL, _rH));
        OPCODE(0x75, 4, 1, "LD (HL),L", _ldi(_rHL, _rL));
        OPCODE(0x76, 4, 1, "HALT", _halt());
        OPCODE(0x77, 4, 1, "LD (HL),A", _ldi(_rHL, _rA));
        OPCODE(0x78, 4, 1, "LD A,B", _ld(_rA, _rB));
        OPCODE(0x79, 4, 1, "LD A,C", _ld(_rA, _rC));
        OPCODE(0x7A, 4, 1, "LD A,D", _ld(_rA, _rD));
        OPCODE(0x7B, 4, 1, "LD A,E", _ld(_rA, _rE));
        OPCODE(0x7C, 4, 1, "LD A,H", _ld(_rA, _rH));
        OPCODE(0x7D, 4, 1, "LD A,L", _ld(_rA, _rL));
        OPCODE(0x7E, 4, 1, "LD A,(HL)", _ld(_rA, _read(_rHL)));
        OPCODE(0x7F, 4, 1, "LD A,A", _ld(_rA, _rA));
        OPCODE(0x80, 4, 1, "ADD A,B", _add(_rA, _rB));
        OPCODE(0x81, 4, 1, "ADD A,C", _add(_rA, _rC));
        OPCODE(0x82, 4, 1, "ADD A,D", _add(_rA, _rD));
        OPCODE(0x83, 4, 1, "ADD A,E", _add(_rA, _rE));
        OPCODE(0x84, 4, 1, "ADD A,H", _add(_rA, _rH));
        OPCODE(0x85, 4, 1, "ADD A,L", _add(_rA, _rL));
        OPCODE(0x86, 4, 1, "ADD A,(HL)", _add(_rA, _read(_rHL)));
        OPCODE(0x87, 4, 1, "ADD A,A", _add(_rA, _rA));
        OPCODE(0x88, 8, 1, "ADC A,B", _adc(_rA, _rB));
        OPCODE(0x89, 8, 1, "ADC A,C", _adc(_rA, _rC));
        OPCODE(0x8A, 8, 1, "ADC A,D", _adc(_rA, _rD));
        OPCODE(0x8B, 8, 1, "ADC A,E", _adc(_rA, _rE));
        OPCODE(0x8C, 8, 1, "ADC A,H", _adc(_rA, _rH));
        OPCODE(0x8D, 8, 1, "ADC A,L", _adc(_rA, _rL));
        OPCODE(0x8E, 8, 1, "ADC A,(HL)", _adc(_rA, _read(_rHL)));
        OPCODE(0x8F, 8, 1, "ADC A,A", _adc(_rA, _rA));
        OPCODE(0x90, 4, 1, "SUB B", _sub(_rA, _rB));
        OPCODE(0x91, 4, 1, "SUB C", _sub(_rA, _rC));
        OPCODE(0x92, 4, 1, "SUB D", _sub(_rA, _rD));
        OPCODE(0x93, 4, 1, "SUB E", _sub(_rA, _rE));
        OPCODE(0x94, 4, 1, "SUB H", _sub(_rA, _rH));
        OPCODE(0x95, 4, 1, "SUB L", _sub(_rA, _rL));
        OPCODE(0x96, 4, 1, "SUB (HL)", _sub(_rA, _read(_rHL)));
        OPCODE(0x97, 4, 1, "SUB A", _sub(_rA, _rA));
        OPCODE(0x98, 8, 1, "SBC A,B", _sbc(_rA, _rB));
        OPCODE(0x99, 8, 1, "SBC A,C", _sbc(_rA, _rC));
        OPCODE(0x9A, 8, 1, "SBC A,D", _sbc(_rA, _rD));
        OPCODE(0x9B, 8, 1, "SBC A,E", _sbc(_rA, _rE));
        OPCODE(0x9C, 8, 1, "SBC A,H", _sbc(_rA, _rH));
        OPCODE(0x9D, 8, 1, "SBC A,L", _sbc(_rA, _rL));
        OPCODE(0x9E, 8, 1, "SBC A,(HL)", _sbc(_rA, _read(_rHL)));
        OPCODE(0x9F, 8, 1, "SBC A,A", _sbc(_rA, _rA));
        OPCODE(0xA0, 4, 1, "AND B", _and(_rA, _rB));
        OPCODE(0xA1, 4, 1, "AND C", _and(_rA, _rC));
        OPCODE(0xA2, 4, 1, "AND D", _and(_rA, _rD));
        OPCODE(0xA3, 4, 1, "AND E", _and(_rA, _rE));
        OPCODE(0xA4, 4, 1, "AND H", _and(_rA, _rH));
        OPCODE(0xA5, 4, 1, "AND L", _and(_rA, _rL));
        OPCODE(0xA6, 4, 1, "AND (HL)", _and(_rA, _read(_rHL)));
        OPCODE(0xA7, 4, 1, "AND A", _and(_rA, _rA));
        OPCODE(0xA8, 4, 1, "XOR B", _xor(_rA, _rB));
        OPCODE(0xA9, 4, 1, "XOR C", _xor(_rA, _rC));
        OPCODE(0xAA, 4, 1, "XOR D", _xor(_rA, _rD));
        OPCODE(0xAB, 4, 1, "XOR E", _xor(_rA, _rE));
        OPCODE(0xAC, 4, 1, "XOR H", _xor(_rA, _rH));
        OPCODE(0xAD, 4, 1, "XOR L", _xor(_rA, _rL));
        OPCODE(0xAE, 7, 2, "XOR (HL)", _xor(_rA, _read(_rHL)));
        OPCODE(0xAF, 4, 1, "XOR A", _xor(_rA, _rA));
        OPCODE(0xB0, 4, 1, "OR B", _or(_rA, _rB));
        OPCODE(0xB1, 4, 1, "OR C", _or(_rA, _rC));
        OPCODE(0xB2, 4, 1, "OR D", _or(_rA, _rD));
        OPCODE(0xB3, 4, 1, "OR E", _or(_rA, _rE));
        OPCODE(0xB4, 4, 1, "OR H", _or(_rA, _rH));
        OPCODE(0xB5, 4, 1, "OR L", _or(_rA, _rL));
        OPCODE(0xB6, 4, 1, "OR (HL)", _or(_rA, _read(_rHL)));
        OPCODE(0xB7, 4, 1, "OR A", _or(_rA, _rA));
        OPCODE(0xB8, 4, 1, "CP B", _cp(_rA, _rB));
        OPCODE(0xB9, 4, 1, "CP C", _cp(_rA, _rC));
        OPCODE(0xBA, 4, 1, "CP D", _cp(_rA, _rD));
        OPCODE(0xBB, 4, 1, "CP E", _cp(_rA, _rE));
        OPCODE(0xBC, 4, 1, "CP H", _cp(_rA, _rH));
        OPCODE(0xBD, 4, 1, "CP L", _cp(_rA, _rL));
        OPCODE(0xBE, 4, 1, "CP (HL)", _cp(_rA, _read(_rHL)));
        OPCODE(0xBF, 4, 1, "CP A", _cp(_rA, _rA));
        OPCODE(0xC0, 8, 1, "RET NZ", _ret(!_flags.Z));
        OPCODE(0xC1, 12, 1, "POP BC", _pop(_rB, _rC));
        OPCODE(0xC2, 12, 0, "JP NZ", _jp(!_flags.Z, _d16()));
        OPCODE(0xC3, 12, 0, "JP", _jp(true, _d16()));
        OPCODE(0xC4, 12, 3, "CALL NZ,a16", _call(!_flags.Z, _d16()));
        OPCODE(0xC5, 16, 1, "PUSH BC", _push(_rB, _rC));
        OPCODE(0xC6, 8, 2, "ADD a,d8", _add(_rA, _d8()));
        OPCODE(0xC7, 16, 1, "RST 00H", _rst(0x00));
        OPCODE(0xC8, 8, 1, "RET Z", _ret(_flags.Z));
        OPCODE(0xC9, 4, 1, "RET", _ret(true));
        OPCODE(0xCA, 12, 3, "JP Z,a16", _jp(_flags.Z, _d16()));
        OPCODE(0xCB, 16, 1, "PREFIX CB", prefix_cb());
        OPCODE(0xCC, 12, 3, "CALL Z,a16", _call(_flags.Z, _d16()));
        OPCODE(0xCD, 24, 3, "CALL a16", _call(true, _d16()));
        OPCODE(0xCE, 8, 2, "ADC a,d8", _adc(_rA, _d8()));
        OPCODE(0xCF, 16, 1, "RST 08H", _rst(0x08));
        OPCODE(0xD0, 8, 1, "RET NC", _ret(!_flags.C));
        OPCODE(0xD1, 12, 1, "POP DE", _pop(_rD, _rE));
        OPCODE(0xD2, 12, 0, "JP NC", _jp(!_flags.C, _d16()));
        OPCODE(0xD4, 12, 3, "CALL NC,a16", _call(!_flags.C, _d16()));
        OPCODE(0xD5, 16, 1, "PUSH DE", _push(_rD, _rE));
        OPCODE(0xD6, 8, 2, "SUB a,d8", _sub(_rA, _d8()));
        OPCODE(0xD7, 16, 1, "RST 10H", _rst(0x10));
        OPCODE(0xD8, 8, 1, "RET C", _ret(_flags.C));
        OPCODE(0xD9, 16, 1, "RETI", _ime = IME::Shadow; _ret(true));
        OPCODE(0xDA, 12, 3, "JP C,a16", _jp(_flags.C, _d16()));
        OPCODE(0xDC, 12, 3, "CALL C, a16", _call(_flags.C, _d16()));
        OPCODE(0xDE, 8, 2, "SBC a,d8", _sbc(_rA, _d8()));
        OPCODE(0xDF, 16, 1, "RST 18H", _rst(0x18));
        OPCODE(0xE0, 12, 2, "LDH (a8), A", _ldi(_a8(), _rA));
        OPCODE(0xE1, 12, 1, "POP HL", _pop(_rH, _rL));
        OPCODE(0xE2, 8, 2, "LD (C), A", _ldi(0xff00 + _rC, _rA));
        OPCODE(0xE5, 16, 1, "PUSH HL", _push(_rH, _rL));
        OPCODE(0xE6, 8, 2, "AND d8", _and(_rA, _d8()));
        OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
        OPCODE(0xE8, 16, 2, "ADD SP, r8", _addsp(_r8()));
        OPCODE(0xE9, 4, 1, "JP (HL)", _jp(true, _rHL));
        OPCODE(0xEA, 16, 3, "LD (a16),A", _ldi(_d16(), _rA));
        OPCODE(0xEE, 16, 1, "XOR d8", _xor(_rA, _d8()));
        OPCODE(0xEF, 16, 1, "RST 28H", _rst(0x28));
        OPCODE(0xF0, 12, 2, "LDH A,(a8)", _ld(_rA, _read(_a8())));
        OPCODE(0xF1, 12, 1, "POP AF", _pop(_rA, _rF); _rF &= 0xf0;);
        OPCODE(0xF2, 8, 2, "LD A, (C)", _ld(_rA, _read(0xff00 + _rC)));
        OPCODE(0xF3, 4, 1, "DI", _ime = IME::Disabled;);
        OPCODE(0xF5, 16, 1, "PUSH AF", _push(_rA, _rF));
        OPCODE(0xF6, 8, 2, "OR d8", _or(_rA, _d8()));
        OPCODE(0xF7, 16, 1, "RST 30H", _rst(0x30));
        OPCODE(0xF8, 12, 2, "LD HL,SP+r8", _ldhlsp(_r8()));
        OPCODE(0xF9, 8, 1, "LD SP,HL", _ldw(_rSP, _rHL));
        OPCODE(0xFa, 16, 3, "LD A,(a16)", _ld(_rA, _read(_d16())));
        OPCODE(0xFB, 4, 1, "EI", _ime = IME::Shadow;);
        OPCODE(0xFE, 8, 2, "CP d8", _cp(_rA, _d8()));
        OPCODE(0xFF, 16, 1, "RST 38H", _rst(0x38));
    default:
        _state = State::Fault;
        std::cout << "Unknown opcode: " << Print(op) << std::endl;
        throw OpcodeException(op);
    }
}

void Cpu::step(void)
{
    interrupt();

    unsigned i = 0;
    switch (_state) {
    case State::Running:
        dispatch();
        break;
    case State::Halted:
        _tick(4);
        break;
    case State::Stopped:
        // XXX: Should we disable the screen?
        break;
    case State::Fault:
        break;
    }
    i = cycles();

    _clock.tick(i);

    _video->tick(i);
}

void Cpu::set(Register r, word_t arg)
{
    switch (r) {
    case Register::A: _rA = arg; break;
    case Register::F: _rF = arg; break;
    case Register::B: _rB = arg; break;
    case Register::C: _rC = arg; break;
    case Register::D: _rD = arg; break;
    case Register::E: _rE = arg; break;
    case Register::H: _rH = arg; break;
    case Register::L: _rL = arg; break;
    case Register::SP: _rSP = arg; break;
    case Register::PC: _rPC = arg; break;
    case Register::AF: _rAF = arg; break;
    case Register::BC: _rBC = arg; break;
    case Register::DE: _rDE = arg; break;
    case Register::HL: _rHL = arg; break;
    }
}

word_t Cpu::get(Register r)
{
    switch (r) {
    case Register::A: return _rA; break;
    case Register::F: return _rF; break;
    case Register::B: return _rB; break;
    case Register::C: return _rC; break;
    case Register::D: return _rD; break;
    case Register::E: return _rE; break;
    case Register::H: return _rH; break;
    case Register::L: return _rL; break;
    case Register::SP: return _rSP; break;
    case Register::PC: return _rPC; break;
    case Register::AF: return _rAF; break;
    case Register::BC: return _rBC; break;
    case Register::DE: return _rDE; break;
    case Register::HL: return _rHL; break;
    }
}

byte_t Cpu::get(addr_t addr)
{
    return _read(addr);
}

void Cpu::set(addr_t addr, byte_t arg)
{
    _write(addr, arg);
}

void Cpu::_dump_reg(std::ostream &os) const
{
    os << "A: " << Print(_rA) << " F: " << Print(_rF)
       << " B: " << Print(_rB) << " C: " << Print(_rC)
       << " D: " << Print(_rD) << " E: " << Print(_rE) << std::endl
       << "HL: " << Print(_rHL)
       << " PC: " << Print(_rPC)
       << " SP: " << Print(_rSP) << std::endl;
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

byte_t Cpu::load(byte_t op)
{
    _write(_rPC++, op);
    return 1;
}

byte_t Cpu::load(byte_t op, byte_t arg)
{
    _write(_rPC++, op);
    _write(_rPC++, arg);
    return 1;
}

byte_t Cpu::load(byte_t op, byte_t arg1, byte_t arg2)
{
    _write(_rPC++, op);
    _write(_rPC++, arg1);
    _write(_rPC++, arg2);
    return 1;
}

void Cpu::test_step(unsigned steps)
{
    _rPC = 0xD000;
    for (unsigned i = 0; i < steps; i++)
        step();
}

byte_t Cpu::_read(addr_t addr)
{
    return _bus.read(addr);
}

void Cpu::_write(addr_t addr, byte_t arg)
{
    _bus.write(addr, arg);
}

/*
 * ____
 *|  _ \ _ __ ___   __ _ _ __ __ _ _ __ ___
 *| |_) | '__/ _ \ / _` | '__/ _` | '_ ` _ \
 *|  __/| | | (_) | (_| | | | (_| | | | | | |
 *|_|   |_|  \___/ \__, |_|  \__,_|_| |_| |_|
 *                 |___/
 */
void Cpu::external_reset(void)
{
    _bus.reset();

    // Load the boot rom (if it exists)
    try {
        bvec boot;
        read_rom("boot_dmg.gb", boot);
        _boot_rom = new SimpleMap(0, boot);
        _bus.add_map(_boot_rom);
    } catch (RomException &e) {
        _rPC = 0x0100;
    }

}

void DMG::read_rom(const std::string &name, bvec &rom)
{
    struct stat sb;
    if (stat(name.c_str(), &sb) == -1)
        throw RomException(name);
    rom.resize(sb.st_size);
    try {
        std::ifstream file(name, std::ios::in | std::ios::binary);
        file.read((char *)&rom[0], rom.size());
    } catch (std::ifstream::failure e) {
        throw RomException(name);
    }
}

addr_t InterruptVector[] = {
    0x40, /* Interrupt::VBlank */
    0x48, /* Interrupt::LCDStat */
    0x50, /* Interrupt::Timer */
    0x58, /* Interrupt::Serial */
    0x60, /* Interrupt::Joypad */
};

/**
 * Process interrupts before dispatch
 */
void Cpu::interrupt(void)
{
    if (_ime == IME::Disabled) {
        if (_state == State::Halted)
            for (unsigned i = 0; i < 5; i++)
                if (bit_isset(_IF, i))
                    _state = State::Running;
        return;
    }
    if (_ime == IME::Shadow) {
        _ime = IME::Enabled;
        return;
    }
    for (unsigned i = 0; i < 5; i++) {
        if (bit_isset(_IF, i) && bit_isset(_IE, i)) {
            _push(_rPCh, _rPCl);
            _rPC = InterruptVector[i];
            if (_debug)
                std::cout << " Interrupt Triggered: " << i << std::endl;
            bit_set(_IF, i, false);
            _tick(20);
            _ime = IME::Disabled;
            _state = State::Running;
            return;
        }
    }
}

Clock::Clock(MemoryBus *bus): _bus(bus)
{
}

Clock::~Clock(void)
{
}

void
Clock::reset(void)
{
    _cycles = 0;
    _dcycles = 0;
    _tcycles = 0;
    _tima = 0;
    _tma = 0;
    _tac = 0;
}

bool
Clock::valid(addr_t addr)
{
    return ((addr == CtrlReg::TIMA) || (addr == CtrlReg::TMA) ||
            (addr == CtrlReg::TAC));
}

void
Clock::write(addr_t addr, byte_t value)
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
Clock::read(addr_t addr)
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
Clock::tick(unsigned cycles)
{
    _cycles += cycles;
    _dcycles += cycles;
    _tcycles += cycles;

    if (_dcycles > 256) {
        _dcycles -= 256;
        _div++;
        // Divider register triggerd
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
                // Trigger the interrupt
                _bus->trigger(Interrupt::Timer);
                // Reset the overflow
                _tima = _tma;
            } else
                _tima++;
        }
    }
}

