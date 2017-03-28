#include <iostream>

#include <upns/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/errorcodes.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

void buildCommitList(upns::Repository *repo, std::vector< std::pair<upns::CommitId, std::shared_ptr<upns::Commit> > > &commits, const ::google::protobuf::RepeatedPtrField< ::std::string> &currentParents)
{
    ::google::protobuf::RepeatedPtrField< ::std::string>::const_iterator currentParent( currentParents.cbegin() );
    while(currentParent != currentParents.cend())
    {
        if(!currentParent->empty())
        {
            std::shared_ptr<upns::Commit> ci(repo->getCommit(*currentParent));
            assert(ci);
            commits.push_back(std::pair< upns::CommitId, std::shared_ptr<upns::Commit> >(*currentParent, ci));
            buildCommitList(repo, commits, ci->parentcommitids());
        }
        currentParent++;
    }
}

int main(int argc, char *argv[])
{
    upns_init_logging();

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

    std::shared_ptr<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );

    if(co == NULL)
    {
        std::cout << "failed to log checkout " << vars["checkout"].as<std::string>() << std::endl;
        return 1;
    }
    std::vector< std::pair<upns::CommitId, std::shared_ptr<upns::Commit> > > commits;
    const std::vector<upns::CommitId> parents(co->getParentCommitIds());
    std::vector<upns::CommitId>::const_iterator currentParent( parents.cbegin() );
    while(currentParent != parents.cend())
    {
        if(!currentParent->empty())
        {
            std::shared_ptr<upns::Commit> ci(repo->getCommit(*currentParent));
            assert(ci);
            commits.push_back(std::pair< upns::CommitId, std::shared_ptr<upns::Commit> >(*currentParent, ci));
            buildCommitList(repo.get(), commits, ci->parentcommitids());
        }
        currentParent++;
    }

    std::vector< std::pair<upns::CommitId, std::shared_ptr<upns::Commit> > >::const_iterator iter(commits.cbegin());
    while(iter != commits.cend())
    {
        std::cout << iter->first << " : ";
        std::cout << " " << iter->second->commitmessage();
        iter++;
    }
    return co == NULL;
}
