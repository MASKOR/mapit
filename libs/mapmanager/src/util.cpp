#include "util.h"
#include <random>

class static_init
{
public:
    static_init()
    {
        srand (0);//time(NULL));
    }
};

static static_init _static_init;
