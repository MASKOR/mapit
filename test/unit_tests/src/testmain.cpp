#include "autotest.h"
#include <log4cplus/configurator.h>
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Test" << std::endl;
    log4cplus::BasicConfigurator config;
    config.configure();
    return AutoTest::run(argc, argv);
}

