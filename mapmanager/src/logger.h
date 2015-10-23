#ifndef __UPNS_LOGGER_H
#define __UPNS_LOGGER_H

#include "upns_globals.h"

namespace upns
{

class Logger
{
    Logger();
public:
    static Logger* getInstance();
};

}
#endif
