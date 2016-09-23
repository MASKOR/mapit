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

void buildCommitList(upns::Repository *repo, upns::upnsVec< upns::upnsPair<upns::CommitId, upns::upnsSharedPointer<upns::Commit> > > &commits, const ::google::protobuf::RepeatedPtrField< ::std::string> &currentParents)
{
    ::google::protobuf::RepeatedPtrField< ::std::string>::const_iterator currentParent( currentParents.cbegin() );
    while(currentParent != currentParents.cend())
    {
        if(!currentParent->empty())
        {
            upns::upnsSharedPointer<upns::Commit> ci(repo->getCommit(*currentParent));
            assert(ci);
            commits.push_back(upns::upnsPair< upns::CommitId, upns::upnsSharedPointer<upns::Commit> >(*currentParent, ci));
            buildCommitList(repo, commits, ci->parentcommitids());
        }
        currentParent++;
    }
}

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();
    log4cplus::SharedAppenderPtr consoleAppender(new log4cplus::ConsoleAppender());
    consoleAppender->setName("myAppenderName");
    //consoleAppender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
    log4cplus::Logger mainLogger = log4cplus::Logger::getInstance("main");
    mainLogger.addAppender(consoleAppender);
    if(argc != 3)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name>" << std::endl;
        std::cout << "was:\n ";
        for(int i=0 ; i<argc ; i++)
            std::cout << argv[i] << " ";
        std::cout << std::endl;
        std::cout << argc;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(std::string(argv[1]));

    upns::Repository *repo = upns::RepositoryFactory::openLocalRepository( config );

    upns::upnsSharedPointer<upns::Checkout> co = repo->getCheckout( argv[2] );

    if(co == NULL)
    {
        std::cout << "failed to log checkout " << argv[3] << std::endl;
        return 1;
    }
    upns::upnsVec< upns::upnsPair<upns::CommitId, upns::upnsSharedPointer<upns::Commit> > > commits;
    const upns::upnsVec<upns::CommitId> parents(co->getParentCommitIds());
    upns::upnsVec<upns::CommitId>::const_iterator currentParent( parents.cbegin() );
    while(currentParent != parents.cend())
    {
        if(!currentParent->empty())
        {
            upns::upnsSharedPointer<upns::Commit> ci(repo->getCommit(*currentParent));
            assert(ci);
            commits.push_back(upns::upnsPair< upns::CommitId, upns::upnsSharedPointer<upns::Commit> >(*currentParent, ci));
            buildCommitList(repo, commits, ci->parentcommitids());
        }
        currentParent++;
    }

    upns::upnsVec< upns::upnsPair<upns::CommitId, upns::upnsSharedPointer<upns::Commit> > >::const_iterator iter(commits.cbegin());
    while(iter != commits.cend())
    {
        std::cout << iter->first << " : ";
        std::cout << " " << iter->second->commitmessage();
        iter++;
    }
    return co == NULL;
}
