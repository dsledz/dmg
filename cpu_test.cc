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


class CpuTest: public ::testing::Test {

    public:
        CpuTest(void): bus(), cpu(&bus), ram(&bus), PC(0xD000)
        {
            bus.add_device(&ram);
            bus.add_device(&cpu);
            cpu.set(Register::PC, 0xD000);
        }

        void test_step(Cpu *cpu, unsigned steps)
        {
            cpu->set(Register::PC, 0xD000);
            for (unsigned i = 0; i < steps; i++)
                bus.step();
        }

        void load(byte_t op)
        {
            bus.write(PC++, op);
            cpu.set(Register::PC, PC);
        }

        void load(byte_t op, byte_t arg)
        {
            bus.write(PC++, op);
            bus.write(PC++, arg);
            cpu.set(Register::PC, PC);
        }

        void load(byte_t op, byte_t arg1, byte_t arg2)
        {
            bus.write(PC++, op);
            bus.write(PC++, arg1);
            bus.write(PC++, arg2);
            cpu.set(Register::PC, PC);
        }

        MemoryBus bus;
        Cpu       cpu;
        RamDevice ram;
        word_t    PC;
};

TEST_F(CpuTest, Constructor) {
    Cpu cpu(&bus);
}

TEST_F(CpuTest, Nop)
{
    load(Opcode::NOP);

    Cpu expect = cpu;

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, IncA)
{
    load(Opcode::INC_A);

    Cpu expect = cpu;
    expect.set(Register::A, 1);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, AddAB)
{
    cpu.set(Register::B, 6);
    load(Opcode::ADD_A_B);

    Cpu expect = cpu;
    expect.set(Register::A, 6);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, LdBB)
{
    cpu.set(Register::B, 8);
    load(Opcode::LD_B_B);

    Cpu expect = cpu;

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, IncBC)
{
    load(Opcode::INC_BC);

    Cpu expect = cpu;
    expect.set(Register::C, 1);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, Mem)
{
    Cpu expect = cpu;
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, LdBC_A)
{
    cpu.set(Register::A, 10);
    cpu.set(Register::BC, 0xC234);
    load(Opcode::LD_BC_A);

    Cpu expect = cpu;
    expect.set(0xC234, 10);
    expect.set(0xE234, 10);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, AndB)
{
    cpu.set(Register::A, 0x1);
    cpu.set(Register::B, 0x0);
    load(Opcode::AND_B);
    Cpu expect = cpu;

    expect.set(Register::A, 0x0);
    expect.set(Register::F, 0xA0);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, LDSP_d16)
{
    load(Opcode::LD_SP_d16, 0xfe, 0xff);

    Cpu expect = cpu;

    expect.set(Register::SP, 0xfffe);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, ldd)
{
    load(Opcode::LD_HL_d16, 0xff, 0xCf);
    load(Opcode::LDD_HL_A);
    load(Opcode::LDD_HL_A);
    load(Opcode::LDD_HL_A);

    Cpu expect = cpu;
    expect.set(Register::HL, 0xCffc);

    test_step(&cpu, 4);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, sp_hl_sp)
{
    load(Opcode::LD_HL_SP_r8, 0xfe);
    cpu.set(Register::SP, 0xd000);

    Cpu expect = cpu;
    expect.set(Register::HL, 0xcffe);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

#if 0
TEST_F(CpuTest, ldd_loop)
{
    load(Opcode::LD_SP_d16, 0xfe, 0xff);
    load(Opcode::XOR_A);
    load(Opcode::LD_HL_d16, 0xff, 0xCf);
    load(Opcode::LDD_HL_A);
    load(Opcode::CB, 0xDC);
    load(Opcode::JR_NZ_R8, 0xfb);

    Cpu expect = cpu;
    expect.set(Register::PC, 0x0C);
    expect.set(Register::HL, 0x7fff);
    expect.set(Register::SP, 0xfffe);

    // Run until we've zeroed out the vram
    test_step(&cpu, 0x1000 * 3 + 3);
}
#endif

TEST_F(CpuTest, add_sp)
{
    cpu.set(Register::SP, 0x8000);
    load(0xE8, 0xFF);

    Cpu expect = cpu;
    expect.set(Register::SP, 0x7fff);
    expect.set(Register::F, 0x00);

    test_step(&cpu, 1);

    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, add_sp2)
{
    cpu.set(Register::SP, 0xffff);
    load(0xE8, 0x01);

    Cpu expect = cpu;
    expect.set(Register::SP, 0x0000);
    expect.set(Register::F, 0x30);

    test_step(&cpu, 1);

    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, ld_a_de)
{
    cpu.set(Register::D, 0xC0);
    cpu.set(Register::E, 0x00);
    cpu.set(0xC000, 0xCC);
    load(Opcode::LD_A_DE);

    Cpu expect = cpu;
    expect.set(Register::A, 0xCC);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, popAF)
{
    cpu.set(Register::BC, 0x1200);
    cpu.set(Register::SP, 0xff90);

    load(Opcode::PUSH_BC);
    load(Opcode::POP_AF);

    Cpu expect = cpu;
    expect.set(Register::AF, 0x1200);
    expect.set(0xff8f, 0x12);
    expect.set(0xff8e, 0x00);

    test_step(&cpu, 2);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, push)
{
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::BC, 0x1301);
    cpu.set(Register::SP, 0xff90);

    load(Opcode::PUSH_BC);
    load(Opcode::POP_AF);
    load(Opcode::PUSH_AF);
    load(Opcode::POP_DE);
    load(Opcode::LD_A_C);
    load(Opcode::AND_D8, 0xF0);
    load(Opcode::CP_E);

    Cpu expect = cpu;
    expect.set(Register::DE, 0x1300);
    expect.set(Register::F, 0xC0);
    expect.set(0xff8f, 0x13);
    expect.set(0xff8e, 0x00);

    test_step(&cpu, 7);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, daa)
{
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 0x64);
    cpu.set(Register::B, 0x46);

    load(Opcode::ADD_A_B);
    load(Opcode::DAA);

    Cpu expect = cpu;

    expect.set(Register::A, 0x10);
    expect.set(Register::F, 0x10);

    test_step(&cpu, 2);
    EXPECT_EQ(expect, cpu);
}

TEST_F(CpuTest, cpl)
{
    cpu.set(Register::PC, 0xD000);
    cpu.set(Register::A, 0xAA);
    load(Opcode::CPL);

    Cpu expect = cpu;
    expect.set(Register::A, 0x55);
    expect.set(Register::F, 0x60);

    test_step(&cpu, 1);
    EXPECT_EQ(expect, cpu);
}

#if 0
// Execute flow
TEST_F(CpuTest, Step)
{
    bus.reset();

    // Execute the boot rom. We hang at 0xe9 because we don't have a rom
    while (cpu.get(Register::PC) != 0x00e9)
        bus.step();

}
#endif

TEST(BitOps, set)
{
    byte_t val;
    bit_set(val, 2, true);
    EXPECT_EQ(val, 0x4);
}

TEST(MemoryTest, test)
{
    MemoryBus m;
    RamDevice ram(&m);
    m.add_device(&ram);

    m.write(0xC500, 0x55);

    byte_t expected = 0xff;
    ram.write(0xC500, expected);
    byte_t actual = m.read(0xC500);
    EXPECT_EQ(expected, actual);
}

TEST(Bus, test)
{
    MemoryBus m;

    //Cpu cpu(&m);
}

