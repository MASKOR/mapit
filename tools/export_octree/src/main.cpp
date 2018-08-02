/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>
#include <mapit/depthfirstsearch.h>
#include <mapit/layertypes/octomaplayer.h>
#include <pcl/conversions.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

namespace po = boost::program_options;

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>(), "the workspace (formerly checkout) to work with")
            ("path,p", po::value<std::string>(), "the path to start the search with")
            ("storrage,s", po::value<std::string>(), "the path to store the octomap");
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1);

    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    try {
        po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    } catch(...) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    if(vars.count("help")) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    try {
        po::notify(vars);
    } catch(...) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    mapit_init_logging();

    int ret = 0;
    // get parameter
    po::variables_map vars;
    ret = get_program_options(argc, argv, vars);
    if (ret != 0) {
      return ret;
    }
    bool use_workspace = vars.count("workspace");
    bool use_path = vars.count("path");

    // connect to repo and workspace
    log_info("start loading from mapit");
    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );
    std::shared_ptr<mapit::Workspace> ws = repo->getWorkspace(vars["workspace"].as<std::string>());
    std::shared_ptr<mapit::AbstractEntitydata> edAbstract = ws->getEntitydataReadOnly( vars["path"].as<std::string>() );
    std::shared_ptr<mapit::entitytypes::Octomap> ed = std::static_pointer_cast<mapit::entitytypes::Octomap>( edAbstract );
    std::shared_ptr<octomap::OcTree> octree = ed->getData();

    log_info("write to file");
    octree->writeBinary( vars["storrage"].as<std::string>() );

    return 0;
}
