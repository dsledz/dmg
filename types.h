#pragma once

#include <string>
#include <algorithm>
#include <future>
#include <iostream>
#include <iomanip>
#include <string>
#include <type_traits>
#include <vector>
#include <list>
#include <stdexcept>
#include <cassert>

/*  _____
 * |_   _|   _ _ __   ___  ___
 *   | || | | | '_ \ / _ \/ __|
 *   | || |_| | |_) |  __/\__ \
 *   |_| \__, | .__/ \___||___/
 *       |___/|_|
 */

typedef unsigned char byte_t;
typedef char sbyte_t;
typedef unsigned short addr_t;
typedef unsigned short word_t;
typedef std::vector<byte_t> bvec;
struct Bytes {
    byte_t l;
    byte_t h;
};
union Word {
    Bytes b;
    word_t w;
};

namespace DMG {

enum Interrupt {
    VBlank = 0,
    LCDStat = 1,
    Timeout = 2,
    Serial = 3,
    Joypad = 4,
};

/* XXX: Should we split this up more? */
enum CtrlReg {
    KEYS = 0xFF00,
    SB   = 0xFF01,
    SC   = 0xFF02,
    DIV  = 0xFF04,
    TIMA = 0xFF05,
    TMA  = 0xFF06,
    TAC  = 0xFF07,
    IF   = 0xFF0F,
    DMA  = 0xFF46,
    DMG_RESET = 0xFF50,
    IE   = 0xFFFF,
};

/*  _____                    _   _
 * | ____|_  _____ ___ _ __ | |_(_) ___  _ __  ___
 * |  _| \ \/ / __/ _ \ '_ \| __| |/ _ \| '_ \/ __|
 * | |___ >  < (_|  __/ |_) | |_| | (_) | | | \__ \
 * |_____/_/\_\___\___| .__/ \__|_|\___/|_| |_|___/
 *                    |_|
 */
class EmuException: public std::exception {
public:
    EmuException() {};
};

class CpuException: public EmuException {
public:
    CpuException() {};
};

class MemException: public EmuException {
public:
    MemException(addr_t addr): address(addr) {};
    addr_t address;
};

class OpcodeException: public EmuException {
public:
    OpcodeException(byte_t op): _op(op) { };
private:
    byte_t _op;
};

class RomException: public EmuException {
public:
    RomException(const std::string &name): rom(name) {};
    std::string rom;
};

/*  _   _      _
 * | | | | ___| |_ __   ___ _ __ ___
 * | |_| |/ _ \ | '_ \ / _ \ '__/ __|
 * |  _  |  __/ | |_) |  __/ |  \__ \
 * |_| |_|\___|_| .__/ \___|_|  |___/
 *              |_|
 */

class Print {
public:
    Print(bool arg): v(arg), w(2) {}
    Print(byte_t arg): v(arg), w(2) {}
    Print(sbyte_t arg): v(arg), w(2) {}
    Print(word_t arg): v(arg), w(4) {}
    Print(unsigned arg): v(arg), w(4) {}
    Print(int arg): v(arg), w(2) {}
    Print(Word arg): v(arg.w), w(2) {}
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

template<class Val>
class Trie {
private:
    struct Element {
        Element(addr_t mask, Val *value, int depth): depth(depth),
            mask(mask), value(value), zero(NULL), one(NULL) { }
        ~Element(void) {
            if (zero != NULL)
                delete zero;
            if (one != NULL)
                delete one;
        }

        int      depth;
        addr_t   mask;         /* Value of bitmask */
        Val     *value;        /* Pointer to value */
        Element *zero;         /* Zero */
        Element *one;          /* One */
    };

    Element **find_ele(addr_t key, int *depth_out) {
        Element **ele = &_head;
        int depth = 16;
        while (*ele != NULL && (*ele)->mask == 0) {
            depth = (*ele)->depth - 1;
            if (bit_isset(key, depth))
                ele = &(*ele)->one;
            else
                ele = &(*ele)->zero;
        }
        if (depth_out != NULL)
            *depth_out = depth;
        return ele;
    }

public:
    Trie(void): _head(NULL) {
        _head = new Element(0x0000, NULL, 16);
    }
    ~Trie(void) {
        if (_head != NULL)
            delete _head;
    }

    Val *find(addr_t key) {
        Element **ele = find_ele(key, NULL);
        if (*ele != NULL && (*ele)->mask != 0x0000)
            return (*ele)->value;
        else
            throw MemException(key);
    }

    void add(addr_t key, Val *val) {
        int depth;
        /* Locate the left most entry */
        Element **ele = find_ele(key, &depth);
        if (*ele != NULL)
            throw MemException(key);
        /* Fill out the trie with additional Inner leafs */
        while (depth > 0) {
            *ele = new Element(0x0000, NULL, depth);
            depth--;
            if (bit_isset(key, depth))
                ele = &(*ele)->one;
            else
                ele = &(*ele)->zero;
        }
        *ele = new Element(0xffff, val, depth);
    }

    void add(addr_t key, int mask, Val *val) {
        int depth;
        /* Locate the left most entry */
        Element **ele = find_ele(key, &depth);
        if (*ele != NULL)
            throw MemException(key);
        mask = 16 - mask;
        /* Fill out the trie with additional Inner leafs */
        while (depth > mask) {
            *ele = new Element(0x0000, NULL, depth);
            depth--;
            if (bit_isset(key, depth))
                ele = &(*ele)->one;
            else
                ele = &(*ele)->zero;
        }
        *ele = new Element(0xffff, val, depth);
    }

private:
    Element *_head;
};

};
