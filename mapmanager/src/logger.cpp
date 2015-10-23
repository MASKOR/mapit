#include "logger.h"
namespace upns
{

Logger::Logger()
{

}

upns::Logger *upns::Logger::getInstance()
{
    static Logger logger;
    return &logger;
}

}
