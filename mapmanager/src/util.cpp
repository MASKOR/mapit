#include "util.h"
#include <random>

class static_init
{
public:
    static_init()
    {
        srand (time(NULL));
    }
};

static static_init _static_init;
