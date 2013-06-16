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
#include "sound.h"
#include "control.h"
#include "mbc.h"

using namespace DMG;

/**
 * Wrap the CPU, graphics, and keyboard handling in a simple runner.
 */
class Emulator {
    public:
        Emulator(void) { }
        ~Emulator(void) { }

        void run(const std::string &name) {
            SDLDisplay display;
            SDLController control;
            SDLAudio audio;
            MBC1 rom;
            RamDevice ram;
            SimpleMap hiram(0xFF00, 0xFFFF);

            std::cout << "Loading: " << name << std::endl;
            rom.load(name);

            _cpu.add_mapper(&rom);
            _cpu.add_mapper(&ram);
            _cpu.add_mapper(&hiram);
            _cpu.set_video(&display);
            _cpu.add_mapper(&control);
            _cpu.add_mapper(&audio);
            _cpu.reset();

            while (!_stop) {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                    OnEvent(&control, &event);

                for (unsigned i = 0; i < 5000; i++)
                    _cpu.step();
            }
            _cpu.set_video(NULL);
            _cpu.remove_mapper(&control);
            _cpu.remove_mapper(&audio);
            _cpu.remove_mapper(&ram);
            _cpu.remove_mapper(&hiram);
        }

        void OnEvent(SDLController *control, SDL_Event *event) {
            switch (event->type) {
            case SDL_QUIT:
                _stop = true;
                break;
            case SDL_KEYUP:
                control->handle_key(event);
                break;
            case SDL_KEYDOWN: {
                // Gameboy keys
                control->handle_key(event);
                // Control keys
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
        usage(argv[0]);
        return 0;
    }

    try {
        emu.run(argv[1]);
    } catch (DMG::RomException &e) {
        std::cout << "Unable to read rom: " << e.rom << std::endl;
        return 1;
    } catch (DMG::CpuException &e) {
        std::cout << "Exiting" << std::endl;
    } catch (...) {
        std::cout << "Unknown error" << std::endl;
    }

    return 0;
}
