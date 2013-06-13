#pragma once

#include "SDL/sdl.h"

namespace DMG {

class RenderCallable: public DMG::Callable {
public:
    RenderCallable(SDL_Surface *window);

    virtual void operator ()(const reg_t *ram);

private:
    SDL_Surface *_window;
};

};
