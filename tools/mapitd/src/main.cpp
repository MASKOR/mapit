#include <iostream>
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactorystandard.h"
#include "upns_errorcodes.h"
#include "upns_logging.h"
#include <zmq.hpp>
#include <boost/program_options.hpp>

#include "versioning/repositorynetworkingfactory.h"

#include <thread>
#include <chrono>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    upns_init_logging();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <port>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("port,p", po::value<std::string>()->required(), "");
    po::positional_options_description pos_options;
    pos_options.add("port",  1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);

    int port;
    if((port = std::stoi( vars["port"].as<std::string>() )) == 0)
    {
        std::cout << "port was not a number; abort" << std::endl;
        return 1;
    }

    upns::Repository *repo = upns::RepositoryFactoryStandard::openRepository( vars );

    upns::RepositoryServer *node = upns::RepositoryNetworkingFactory::openRepositoryAsServer(port, repo);

    while(true)
    {
        node->handleRequest(500);
    }

    // No smart pointers here to ensure sequence of deletion
    delete node;
    delete repo;
    return 0;
}
