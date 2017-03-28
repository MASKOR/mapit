#include "autotest.h"
#include <upns/logging.h>
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <iostream>

int main(int argc, char *argv[])
{
  /*
    log4cplus::BasicConfigurator config;
    config.configure();
    log4cplus::SharedAppenderPtr consoleAppender(new log4cplus::ConsoleAppender());
    consoleAppender->setName("myAppenderName");
    //consoleAppender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
    log4cplus::Logger mainLogger = log4cplus::Logger::getInstance("main");
    mainLogger.addAppender(consoleAppender);
*/
    log4cplus::PropertyConfigurator::doConfigure("logging.properties");

    log_info("running test");

    return AutoTest::run(argc, argv);
}

