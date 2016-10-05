#include <iostream>
#include "leveldb/db.h"
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include <QFile>
#include <QDir>
#include <yaml-cpp/yaml.h>
#include "error.h"
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <zmq.hpp>

#include "versioning/repositorynetworkingfactory.h"

#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();
    log4cplus::SharedAppenderPtr consoleAppender(new log4cplus::ConsoleAppender());
    consoleAppender->setName("myAppenderName");
    //consoleAppender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
    log4cplus::Logger mainLogger = log4cplus::Logger::getInstance("main");
    mainLogger.addAppender(consoleAppender);
    if(argc != 3 && argc != 4)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <port> [<url>]" << std::endl;
        std::cout << "was:\n ";
        for(int i=0 ; i<argc ; i++)
            std::cout << argv[i] << " ";
        std::cout << std::endl;
        std::cout << argc;
        return 1;
    }

    QString portStr(argv[2]);
    int port;
    if((port = portStr.toInt()) == 0)
    {
        std::cout << "port was not a number; abort" << std::endl;
        return 1;
    }

    upns::upnsString url;

    if(argc == 4)
    {
        url = argv[3];
    }

    YAML::Node config = YAML::LoadFile(std::string(argv[1]));

    upns::Repository *repo = upns::RepositoryFactory::openLocalRepository( config );

    upns::RepositoryServer *node = upns::RepositoryNetworkingFactory::openRepositoryAsServer(port, repo, url);

    while(true)
    {
        node->handleRequest(0);
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    delete node;
    delete repo;
    return 0;
}
