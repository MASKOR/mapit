#include <iostream>
#include "leveldb/db.h"
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include <QFile>
#include <QDir>
#include <yaml-cpp/yaml.h>
#include "upns_errorcodes.h"
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();
    log4cplus::SharedAppenderPtr consoleAppender(new log4cplus::ConsoleAppender());
    consoleAppender->setName("myAppenderName");
    //consoleAppender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
    log4cplus::Logger mainLogger = log4cplus::Logger::getInstance("main");
    mainLogger.addAppender(consoleAppender);
    if(argc != 4)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name> <commitmessage>" << std::endl;
        std::cout << "was:\n ";
        for(int i=0 ; i<argc ; i++)
            std::cout << argv[i] << " ";
        std::cout << std::endl;
        std::cout << argc;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(std::string(argv[1]));

    upns::Repository *repo = upns::RepositoryFactory::openLocalRepository( config );

    upns::upnsSharedPointer<upns::Checkout> co = repo->getCheckout( argv[2]);

    if(co)
    {
        const upns::CommitId ciid = repo->commit(co, argv[3]);
        std::cout << "commit " << ciid << std::endl;
    }
    else
    {
        std::cout << "failed to commit checkout " << argv[3] << std::endl;
    }
    return co == NULL;
}
