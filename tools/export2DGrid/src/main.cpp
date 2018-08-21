/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2018 Michael Norget   <mnorget@amt.rwth-aachen.de>
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
#include <fstream>

#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/errorcodes.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>
#include <mapit/layertypes/grid2dHelper.h>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    mapit_init_logging();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name> <commitmessage>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>()->required(), "the workspace/version state to operate on")
            ("path,p", po::value<std::string>()->required(), "entity path to 2d grid to export")
            ("mapFile,m", po::value<std::string>()->required(), "fully qualified or relative path to where the map data should be saved, will be saved as <path>.yaml and <path>.pgm");
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1)
               .add("commitmessage",  1);

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

    if ( ! workspace) {
        std::cout << "failed to get workspace " << std::endl;
        return 1;
    }

    // do the magic
    // export to ROS compatible 2d map data, see http://wiki.ros.org/map_server
    // YAML file and some sort of picture / graphic image required
    // graphic image will be interpreted using SDL_Imgage,see http://www.libsdl.org/projects/SDL_image/docs/SDL_image.html
    // for supported formats.
    // Using pnm file, more specific pgm for greyscale as map only needs greyscales
    // definiion, see http://netpbm.sourceforge.net/doc/pgm.html

    // load entity data
    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath
            = workspace->getEntitydataReadOnly(vars["path"].as<std::string>());
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> entityData
            = std::dynamic_pointer_cast<mapit::entitytypes::Grid2DHelper>( abstractentitydataByPath );

    // write yaml file
    std::ofstream yamlFile;
    yamlFile.open (vars["path"].as<std::string>() + ".yaml");
    yamlFile << "image: " << vars["path"].as<std::string>() << ".pgm\n";
    yamlFile << "reslution: " << std::to_string(entityData->getGrid().resolution()) << "\n";
    mapit::msgs::Vector oTrans = entityData->getGrid().origin().translation();
    mapit::msgs::Quaternion oRot = entityData->getGrid().origin().rotation();

    // calc yaw (z-axis rotation) from quaternation
    // code from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double siny = +2.0 * (oRot.w() * oRot.z() + oRot.x() * oRot.y());
    double cosy = +1.0 - 2.0 * (oRot.y() * oRot.y() + oRot.z() * oRot.z());
    double yaw = atan2(siny, cosy);
    yamlFile << "origin: [" << std::to_string(oTrans.x()) << ", " << std::to_string(oTrans.y())
             << ", " << std::to_string(yaw) << "]\n";
    // TODO set through params?
    yamlFile << "occupied_thresh: 0.99";
    yamlFile << "free_thresh: 0.01";
    yamlFile << "negate: 1"; // image is negated
    yamlFile.close();

    // write image file
    // image is ASCII exchange file format, may be more efficient to use binary format
    // see
    std::ofstream imageFile;
    imageFile.open (vars["path"].as<std::string>() + ".pgm");
    // P2 width height maxVal
    imageFile << "P2\n";
    imageFile << "Export of 2d map data";
    imageFile << std::to_string(entityData->getGrid().width()) << " " << std::to_string(entityData->getGrid().width()) << "\n";
    imageFile << "100\n";
    const float resTicks = entityData->getGrid().resolution();
    for (float y = 0; y < entityData->getGrid().height(); y+= resTicks) {
        for (float x = 0; x < entityData->getGrid().width(); x+=resTicks) {
            imageFile << " " << std::to_string(entityData->getProbability(x, y)) << " ";
        }
        imageFile << "\n";
    }
    imageFile.close();

    return 0;
}
