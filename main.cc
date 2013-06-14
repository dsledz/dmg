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
 * Gameboy emulator main file.
 */
#include "SDL/sdl.h"

#include "cpu.h"
#include "graphics.h"

using namespace DMG;

/**
 * Wrap the CPU, graphics, and keyboard handling in a simple runner.
 */
class Emulator {
    public:
        Emulator(void) { }
        ~Emulator(void) { }

        void load(const char *rom) {
            const std::string name(rom);
            std::cout << "Loading: " << name << std::endl;
            _cpu.load_rom(name);
        }

        void run(void) {
            SDLDisplay display;
            _cpu.reset();
            _cpu.set_video(&display);

            while (!_stop) {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                    OnEvent(&event);

                for (unsigned i = 0; i < 5000; i++)
                    _cpu.step();
            }
            _cpu.set_video(NULL);
        }

        void OnEvent(SDL_Event *event) {
            switch (event->type) {
            case SDL_QUIT:
                _stop = true;
                break;
            case SDL_KEYUP:
            case SDL_KEYDOWN: {
                bool set = (event->type == SDL_KEYDOWN);
                // Gameboy keys
                switch (event->key.keysym.sym) {
                case SDLK_a: _cpu.set_key(GBKey::A, set); break;
                case SDLK_s: _cpu.set_key(GBKey::B, set); break;
                case SDLK_z: _cpu.set_key(GBKey::Select, set); break;
                case SDLK_x: _cpu.set_key(GBKey::Start, set); break;
                case SDLK_LEFT: _cpu.set_key(GBKey::Left, set); break;
                case SDLK_RIGHT: _cpu.set_key(GBKey::Right, set); break;
                case SDLK_UP: _cpu.set_key(GBKey::Up, set); break;
                case SDLK_DOWN: _cpu.set_key(GBKey::Down, set); break;
                default:
                    break;
                }
                // Control keys
                if (set) {
                    switch (event->key.keysym.sym) {
                    case SDLK_q:
                        _stop = true;
                        break;
                    case SDLK_F2:
                        _cpu.dump();
                        break;
                    case SDLK_F1:
                        _cpu.toggle_debug();
                        break;
                    default:
                        break;
                    }
                }
                break;
            }
            default:
                break;
            }
        }

    private:
        DMG::Cpu _cpu;
        bool _stop;
};

struct SDLObject {
    SDLObject(void) {
        if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
            exit(1);
    }
    ~SDLObject() {
        SDL_Quit();
    }
};

static void usage(const std::string &app)
{
    std::cout << app << " <rom>" << std::endl;
}

extern "C" int main(int argc, char **argv)
{
    SDLObject sdl;

    Emulator emu;
    if (argc < 2) {
        usage(argv[1]);
        return 0;
    }

    try {
        emu.load(argv[1]);
    } catch (DMG::RomException &e) {
        std::cout << "Unable to read rom: " << e.rom << std::endl;
        return 1;
    }

    try {
        emu.run();
    } catch (DMG::CpuException &e) {
        std::cout << "Exiting" << std::endl;
    }

    return 0;
}
