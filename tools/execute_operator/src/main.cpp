#include <iostream>

#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/errorcodes.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    upns_init_logging();

    ///// read parameters from commandline /////

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout> <operator_name> <parameters>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("checkout,co", po::value<std::string>()->required(), "the checkout/version state to operate on")
            ("operator,op", po::value<std::string>()->required(), "name of the operator (with underscores)")
            ("parameters,p",po::value<std::string>()->default_value(std::string("")), "string of parameters for the operation. (any format, commonly json)");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1) // if no names are used, checkout is given first
               .add("operator",  1) // then operator
               .add("parameters",1);// followed by the parameter string

    // Let mapit RepositoryFactoryStandard add it's own options (direcory or port where repo can be found, ...)
    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);

    // Fianlly parse/get/store the parameters from commandline
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);

    // If help was requested, show it before validating input
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }

    // validate input
    po::notify(vars);

    ///// end of parameter input /////

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    std::shared_ptr<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );
    if(co == nullptr)
    {
        log_error("Checkout: " + vars["checkout"].as<std::string>() + "not found");
        return 1;
    }

    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname(vars["operator"].as<std::string>());
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
