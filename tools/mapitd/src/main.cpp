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
#include <zmq.hpp>
#include <boost/program_options.hpp>

#include <mapit/versioning/repositorynetworkingfactory.h>

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

    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
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

    mapit::Repository *repo = mapit::RepositoryFactoryStandard::openRepository( vars );

    mapit::RepositoryServer *node = mapit::RepositoryNetworkingFactory::openRepositoryAsServer(port, repo);

    while(true)
    {
        node->handleRequest(500);
    }

    // No smart pointers here to ensure sequence of deletion
    delete node;
    delete repo;
    return 0;
}
