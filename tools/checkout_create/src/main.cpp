#include <iostream>

#include "services.pb.h"
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/errorcodes.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    upns_init_logging();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout name> <branch or commitid>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("checkout,co", po::value<std::string>()->required(), "")
            ("commitref,ref", po::value<std::string>()->required(), "");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1)
               .add("commitref",  1);

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

    upns::upnsSharedPointer<upns::Checkout> co = repo->createCheckout( vars["commitref"].as<std::string>(), vars["checkout"].as<std::string>() );
    if(co != nullptr)
    {
        std::cout << "checkout " << vars["checkout"].as<std::string>() << " successfully created" << std::endl;
    }
    else
    {
        std::cout << "failed to create checkout from " << vars["commitref"].as<std::string>() << std::endl;
    }
    return co == NULL;
}
