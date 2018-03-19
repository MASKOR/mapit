/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/errorcodes.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    mapit_init_logging();

    ///// read parameters from commandline /////

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace> <operator_name> <parameters>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>()->required(), "the workspace/version state to operate on")
            ("operator,op", po::value<std::string>()->required(), "name of the operator (with underscores)")
            ("parameters,p",po::value<std::string>()->default_value(std::string("")), "string of parameters for the operation. (any format, commonly json)");
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1) // if no names are used, workspace is given first
               .add("operator",  1) // then operator
               .add("parameters",1);// followed by the parameter string

    // Let mapit RepositoryFactoryStandard add it's own options (direcory or port where repo can be found, ...)
    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);

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

    std::unique_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );

    std::shared_ptr<mapit::Workspace> workspace = repo->getWorkspace( vars["workspace"].as<std::string>() );
    if(workspace == nullptr)
    {
        log_error("Workspace: " + vars["workspace"].as<std::string>() + "not found");
        return 1;
    }

    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname(vars["operator"].as<std::string>());
    desc.set_params(vars["parameters"].as<std::string>());
    log_info("Executing: " + vars["operator"].as<std::string>() + ", with params: " + vars["parameters"].as<std::string>());
    mapit::OperationResult res = workspace->doOperation(desc);
    if(mapitIsOk(res.first))
    {
        std::cout << "success" << std::endl;
    }
    else
    {
        std::cout << "failed to execute operator" << vars["operator"].as<std::string>() << std::endl;
    }
    return res.first;
}
