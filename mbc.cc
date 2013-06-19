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
 * Memory bank Controller
 */
#include "mbc.h"

#include <fstream>
#include <sys/stat.h>

using namespace DMG;

void
DMG::read_rom(const std::string &name, bvec &rom)
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

void
MBC1::load(const std::string &name)
{
    read_rom(name, _rom);

    if ((_rom[0x0143] & 0xC0) == 0xC0) {
        std::cout << "Color Gameboy Only" << std::endl;
        throw RomException(name);
    }

    // Cartridge Header
    _name.clear();
    for (unsigned i = 0x0134; i < 0x0142; i++)
        _name += _rom[i];
    _type = static_cast<Cartridge>(_rom[0x0147]);

    switch (_type) {
    case Cartridge::RomOnly:
    case Cartridge::MBC1O:
    case Cartridge::MBC1R:
    case Cartridge::MBC1RB:
    case Cartridge::MBC3RRB:
        break;
    default:
        throw RomException(name);
        break;
    }

    switch (_rom[0x0148]) {
    case 0:
        _rom_size =   32 * 1024;
        break;
    case 1:
        _rom_size =   64 * 1024;
        break;
    case 2:
        _rom_size =  128 * 1024;
        break;
    case 3:
        _rom_size =  256 * 1024;
        break;
    case 4:
        _rom_size =  512 * 1024;
        break;
    case 5:
        _rom_size = 1024 * 1024;
        break;
    case 6:
        _rom_size = 2048 * 1024;
        break;
    case 0x52:
        _rom_size = 1152 * 1024;
        break;
    case 0x53:
        _rom_size = 1280 * 1024;
        break;
    case 0x54:
        _rom_size = 1536 * 1024;
        break;
    }
    if (_rom_size != _rom.size())
        throw RomException(name);
    _rom_low = 0;
    _rom_high = std::min(_rom.size(), (unsigned long)0x8000);

    switch (_rom[0x0149]) {
    case 0:
        _ram_size = 0;
    case 1:
        _ram_size = 2 * 1024;
        break;
    case 2:
        _ram_size = 8 * 1024;
        break;
    case 3:
        _ram_size = 32 * 1024;
        break;
    case 4:
        _ram_size = 128 * 1024;
        break;
    }
    _ram.resize(_ram_size);
    _ram_low = _ram_size > 0 ? 0xA000 : 0xFFFFFF;
    _ram_high = _ram_size > 0 ? 0xC000 : 0xFFFFFF;

#if 0
    std::cout << "Cartridge Header" << std::endl;
    std::cout << "Name: " << _name << std::endl;
    std::cout << "MBC: " << Print(_type) << std::endl;
    std::cout << "Rom Size: " << Print(_rom_size) << std::endl;
#endif

}

void
MBC1::reset(void)
{
    memset(&_ram[0], 0, _ram.size());
    _ram_bank = 0;
    _rom_bank = 1;
}

bool
MBC1::valid(addr_t addr)
{
    return ((addr >= _rom_low && addr < _rom_high) ||
            (addr >= _ram_low && addr < _ram_high));
}

void
MBC1::write(addr_t addr, byte_t value)
{
    if (addr >= _ram_low && addr < _ram_high) {
        addr -= _ram_low;
        _ram[_ram_bank * 0x2000 + addr] = value;
    } else if (addr >= 0x6000 && addr < 0x8000) {
        // XXX: Select mode
    } else if (addr >= 0x4000 && addr < 0x6000) {
        _ram_bank = value & 0x03;
    } else if (addr >= 0x2000 && addr < 0x4000) {
        if (_type == MBC3RRB)
            _rom_bank = value & 0x7f;
        else
            _rom_bank = value & 0x1f;
        if (_rom_bank == 0 || (_rom_bank * 0x4000) > _rom_size)
            _rom_bank = 1;
    } else if (addr >= 0) {
        value &= 0x0F;
        if (value == 0x0A) {
            // XXX: Enable ram
        } else {
            // XXX: Disable ram
        }
    }
}

byte_t
MBC1::read(addr_t addr)
{
    if (addr < 0x4000)
        return _rom[addr];
    else if (addr >= _ram_low && addr < _ram_high) {
        addr -= _ram_low;
        return _ram.at(_ram_bank * 0x2000 + addr);
    } else {
        addr -= 0x4000;
        return _rom.at(_rom_bank * 0x4000 + addr);
    }
}


