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

#include "gtest/gtest.h"
#include "cpu.h"

using namespace DMG;

enum Opcode {
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

static inline byte_t op_value(const Opcode &obj)
{
    return static_cast<std::underlying_type<Opcode>::type>(obj);
}

TEST(DMGOpcodes, Constructor) {
    Cpu cpu;
}

TEST(BitOps, set)
{
    byte_t val;
    bit_set(val, 2, true);
    EXPECT_EQ(val, 0x4);
}

TEST(DMGOpcodes, Nop)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::NOP);

    expect = cpu;

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, IncA)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::INC_A);

    expect = cpu;
    expect.set(Register::A, 1);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, AddAB)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::B, 6);
    cpu.load(Opcode::ADD_A_B);

    expect = cpu;
    expect.set(Register::A, 6);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, LdBB)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::B, 8);
    cpu.load(Opcode::LD_B_B);

    expect = cpu;

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, IncBC)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::INC_BC);

    expect = cpu;
    expect.set(Register::C, 1);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, Mem)
{
    Cpu cpu, expect;
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, LdBC_A)
{
    Cpu cpu, expect;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 10);
    cpu.set(Register::BC, 0xC234);
    cpu.load(Opcode::LD_BC_A);

    expect = cpu;
    expect.set(0xC234, 10);
    expect.set(0xE234, 10);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, AndB)
{
    Cpu cpu, expect;

    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 0x1);
    cpu.set(Register::B, 0x0);
    cpu.load(Opcode::AND_B);
    expect = cpu;

    expect.set(Register::A, 0x0);
    expect.set(Register::F, 0xA0);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, LDSP_d16)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::LD_SP_d16, 0xfe, 0xff);

    Cpu expect = cpu;

    expect.set(Register::SP, 0xfffe);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, ldd)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::LD_HL_d16, 0xff, 0x9f);
    cpu.load(Opcode::LDD_HL_A);
    cpu.load(Opcode::LDD_HL_A);
    cpu.load(Opcode::LDD_HL_A);

    Cpu expect = cpu;
    expect.set(Register::HL, 0x9ffc);

    cpu.test_step(4);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, sp_hl_sp)
{
    Cpu cpu;

    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::LD_HL_SP_r8, 0xfe);
    cpu.set(Register::SP, 0xd000);

    Cpu expect = cpu;
    expect.set(Register::HL, 0xcffe);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, ldd_loop)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.load(Opcode::LD_SP_d16, 0xfe, 0xff);
    cpu.load(Opcode::XOR_A);
    cpu.load(Opcode::LD_HL_d16, 0xff, 0x9f);
    cpu.load(Opcode::LDD_HL_A);
    cpu.load(Opcode::CB, 0x7C);
    cpu.load(Opcode::JR_NZ_R8, 0xfb);

    Cpu expect = cpu;
    expect.set(Register::PC, 0x0C);
    expect.set(Register::HL, 0x7fff);
    expect.set(Register::SP, 0xfffe);
    expect.set(CtrlReg::STAT, 0x11);

    // Run until we've zeroed out the vram
    cpu.test_step(0x2000 * 3 + 3);
}

TEST(DMGOpcodes, add_sp)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::SP, 0x8000);
    cpu.load(0xE8, 0xFF);

    Cpu expect = cpu;
    expect.set(Register::SP, 0x7fff);
    expect.set(Register::F, 0x00);

    cpu.test_step(1);

    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, add_sp2)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::SP, 0xffff);
    cpu.load(0xE8, 0x01);

    Cpu expect = cpu;
    expect.set(Register::SP, 0x0000);
    expect.set(Register::F, 0x30);

    cpu.test_step(1);

    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, ld_a_de)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::D, 0xC0);
    cpu.set(Register::E, 0x00);
    cpu.set(0xC000, 0xCC);
    cpu.load(Opcode::LD_A_DE);

    Cpu expect = cpu;
    expect.set(Register::A, 0xCC);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, popAF)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::BC, 0x1200);
    cpu.set(Register::SP, 0xff90);

    cpu.load(Opcode::PUSH_BC);
    cpu.load(Opcode::POP_AF);

    Cpu expect = cpu;
    expect.set(Register::AF, 0x1200);
    expect.set(0xff8f, 0x12);
    expect.set(0xff8e, 0x00);

    cpu.test_step(2);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, push)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::BC, 0x1301);
    cpu.set(Register::SP, 0xff90);

    cpu.load(Opcode::PUSH_BC);
    cpu.load(Opcode::POP_AF);
    cpu.load(Opcode::PUSH_AF);
    cpu.load(Opcode::POP_DE);
    cpu.load(Opcode::LD_A_C);
    cpu.load(Opcode::AND_D8, 0xF0);
    cpu.load(Opcode::CP_E);

    Cpu expect = cpu;
    expect.set(Register::DE, 0x1300);
    expect.set(Register::F, 0xC0);
    expect.set(0xff8f, 0x13);
    expect.set(0xff8e, 0x00);

    cpu.test_step(7);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, daa)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 0x64);
    cpu.set(Register::B, 0x46);

    cpu.load(Opcode::ADD_A_B);
    cpu.load(Opcode::DAA);

    Cpu expect = cpu;

    expect.set(Register::A, 0x10);
    expect.set(Register::F, 0x10);

    cpu.test_step(2);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, cpl)
{
    Cpu cpu;
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 0xAA);
    cpu.load(Opcode::CPL);

    Cpu expect = cpu;
    expect.set(Register::A, 0x55);
    expect.set(Register::F, 0x60);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

// Execute flow
TEST(DMGTest, Step)
{
    Cpu cpu, expect;
    cpu.reset();

    expect = cpu;

    // Execute the boot rom. We hang at 0xe9 because we don't have a rom
    while (cpu.get(Register::PC) != 0x00e9) cpu.step();

}

