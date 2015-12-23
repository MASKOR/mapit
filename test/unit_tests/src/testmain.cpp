#include "autotest.h"
#include <log4cplus/configurator.h>

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator config;
    config.configure();
    return AutoTest::run(argc, argv);
}

