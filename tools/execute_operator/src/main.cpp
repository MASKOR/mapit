#include <iostream>
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactorystandard.h"
#include <yaml-cpp/yaml.h>
#include "upns_errorcodes.h"
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout> <operator_name> <parameters>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("checkout,co", po::value<std::string>()->required(), "")
            ("operator,op", po::value<std::string>()->required(), "")
            ("parameters,p",po::value<std::string>()->default_value(std::string("")), "");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1)
               .add("operator",  1)
               .add("parameters",1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);
//    if(argc != 5)
//    {
//        std::cout << "usage:\n " << argv[0] << " <checkout_name> <operator_name> <paramstring>" << std::endl;
//        std::cout << "was:\n ";
//        for(int i=0 ; i<argc ; i++)
//            std::cout << argv[i] << " ";
//        std::cout << std::endl;
//        std::cout << argc;
//        return 1;
//    }

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    upns::upnsSharedPointer<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );
    if(co == nullptr)
    {
        log_error("Checkout: " + vars["checkout"].as<std::string>() + "not found");
        return 1;
    }

    upns::OperationDescription desc;
    desc.set_operatorname(vars["operator"].as<std::string>());
    desc.set_params(vars["parameters"].as<std::string>());
    log_info("Executing: " + vars["operator"].as<std::string>() + ", with params: " + vars["parameters"].as<std::string>());
    upns::OperationResult res = co->doOperation(desc);
    if(upnsIsOk(res.first))
    {
        std::cout << "success" << std::endl;
    }
    else
    {
        std::cout << "failed to execute operator" << vars["operator"].as<std::string>() << std::endl;
    }
    return res.first;
}
