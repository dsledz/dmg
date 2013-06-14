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

TEST(DMGOpcodes, Constructor) {
    Cpu cpu;
}

#define OPTEST(name, byte_code, start_regs, final_regs, steps) \
TEST(DMGOpcodes, name) { \
    Cpu cpu; \
    reg_t code[] = byte_code; \
    RegisterSet reg1 = start_regs; \
    RegisterSet reg2 = final_regs; \
    cpu.test_setup(reg1, code, sizeof(code)); \
    Cpu expect(cpu); \
    expect.set(reg2); \
    cpu.test_step(steps); \
    EXPECT_EQ(expect, cpu); \
}

TEST(BitOps, set)
{
    reg_t val;
    bit_set(val, 2, true);
    EXPECT_EQ(val, 0x4);
}

TEST(DMGOpcodes, op_0x01)
{
    Cpu cpu;
    reg_t code[] = {0x01, 0x12, 0x34};
    RegisterSet reg1 = { .A = 0 };
    RegisterSet reg2 = { .B = 0x34, .C=0x12 };
    cpu.test_setup(reg1, code, sizeof(code));

    Cpu expect(cpu);
    expect.set(reg2);
    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, Nop)
{
    Cpu cpu, expect;
    cpu.load(Opcode::NOP);

    expect = cpu;

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, IncA)
{
    Cpu cpu, expect;
    cpu.load(Opcode::INC_A);

    expect = cpu;
    expect.set(Register::A, 1);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, AddAB)
{
    Cpu cpu, expect;
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
    cpu.set(Register::B, 8);
    cpu.load(Opcode::LD_B_B);

    expect = cpu;

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, IncBC)
{
    Cpu cpu, expect;
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
    cpu.load(Opcode::LD_SP_d16, 0xfe, 0xff);

    Cpu expect = cpu;

    expect.set(Register::SP, 0xfffe);
    expect.set(Register::PC, 3);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, ldd)
{
    Cpu cpu;
    cpu.load(Opcode::LD_HL_d16, 0xff, 0x9f);
    cpu.load(Opcode::LDD_HL_A);
    cpu.load(Opcode::LDD_HL_A);
    cpu.load(Opcode::LDD_HL_A);

    Cpu expect = cpu;
    expect.set(Register::PC, 6);
    expect.set(Register::HL, 0x9ffc);

    cpu.test_step(4);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, sp_hl_sp)
{
    Cpu cpu;

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

    cpu.debug(false);
    // Run until we've zeroed out the vram
    cpu.test_step(0x2000 * 3 + 3);
}

TEST(DMGOpcodes, ld_a_de)
{
    Cpu cpu;
    cpu.set(Register::D, 0x01);
    cpu.set(Register::E, 0x04);
    cpu.set(0x0104, 0xCC);
    cpu.load(Opcode::LD_A_DE);

    Cpu expect = cpu;
    expect.set(Register::A, 0xCC);
    expect.set(CtrlReg::STAT, 0x10);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, popAF)
{
    Cpu cpu;
    cpu.set(Register::BC, 0x1200);
    cpu.set(Register::SP, 0xff90);

    cpu.load(Opcode::PUSH_BC);
    cpu.load(Opcode::POP_AF);

    Cpu expect = cpu;
    expect.set(Register::AF, 0x1200);
    expect.set(0xff8f, 0x12);
    expect.set(0xff8e, 0x00);
    expect.set(CtrlReg::STAT, 0x10);

    cpu.test_step(2);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, push)
{
    Cpu cpu;
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
    expect.set(CtrlReg::STAT, 0x11);

    cpu.test_step(7);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, daa)
{
    Cpu cpu;
    cpu.set(Register::A, 0x64);
    cpu.set(Register::B, 0x46);

    cpu.load(Opcode::ADD_A_B);
    cpu.load(Opcode::DAA);

    Cpu expect = cpu;

    expect.set(Register::A, 0x10);
    expect.set(Register::F, 0x30);
    expect.set(CtrlReg::STAT, 0x11);

    cpu.test_step(2);
    EXPECT_EQ(expect, cpu);
}

TEST(DMGOpcodes, cpl)
{
    Cpu cpu;
    cpu.set(Register::A, 0xAA);
    cpu.load(Opcode::CPL);

    Cpu expect = cpu;
    expect.set(Register::A, 0x55);
    expect.set(Register::F, 0x60);
    expect.set(CtrlReg::STAT, 0x11);

    cpu.test_step(1);
    EXPECT_EQ(expect, cpu);
}

// Execute flow
TEST(DMGTest, Load)
{
    Cpu cpu;
    cpu.reset();

    Cpu expect = cpu;
    cpu.test_step(1);

    expect.set(Register::PC, 0x3);
    expect.set(CtrlReg::STAT, 0x11);

    EXPECT_EQ(expect, cpu);

    cpu.test_step(1);
}

TEST(DMGTest, Step)
{
    Cpu cpu, expect;
    cpu.reset();

    expect = cpu;

    // zero the vram
    while (cpu.get(Register::PC) != 0x0100) cpu.step();
    // Execute the dmg

    while (cpu.cycles() < 40000000) cpu.step();
    while (true) cpu.step();

    cpu.to_string(std::cout);
}

