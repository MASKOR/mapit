#include <iostream>
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactorystandard.h"
#include "upns_errorcodes.h"
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

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

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout name>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("checkout,co", po::value<std::string>()->required(), "");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    upns::upnsSharedPointer<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );

    if(co == NULL)
    {
        std::cout << "failed to log checkout " << vars["checkout"].as<std::string>() << std::endl;
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
            buildCommitList(repo.get(), commits, ci->parentcommitids());
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
