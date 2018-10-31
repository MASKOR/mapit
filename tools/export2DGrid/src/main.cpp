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
#include <mapit/layertypes/grid2d.h>
#include <mapit/layertypes/grid2dHelper.h>
#ifdef OPENCV_VERSION_2
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

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
    std::string vla_1 = vars["workspace"].as<std::string>();
    std::string vla_2 = vars["path"].as<std::string>();
    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath
            = workspace->getEntitydataReadOnly(vars["path"].as<std::string>());
    std::string type = workspace->getEntity(vars["path"].as<std::string>())->type();
    std::shared_ptr<mapit::entitytypes::Grid2D> entityData
            = std::static_pointer_cast<mapit::entitytypes::Grid2D>( abstractentitydataByPath );
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> gridHelper = entityData->getData();

    // write yaml file
    std::ofstream yamlFile;
    std::string image_name = vars["mapFile"].as<std::string>();
    image_name = image_name.substr( image_name.find_last_of("/") + 1, image_name.size() );
    yamlFile.open (vars["mapFile"].as<std::string>() + ".yaml");
    yamlFile << "image: " << image_name << ".png" << std::endl;
    yamlFile << "resolution: " << std::to_string(gridHelper->getGrid().resolution()) << std::endl;
    mapit::msgs::Vector oTrans = gridHelper->getGrid().origin().translation();
    mapit::msgs::Quaternion oRot = gridHelper->getGrid().origin().rotation();

    // calc yaw (z-axis rotation) from quaternation
    // code from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double siny = +2.0 * (oRot.w() * oRot.z() + oRot.x() * oRot.y());
    double cosy = +1.0 - 2.0 * (oRot.y() * oRot.y() + oRot.z() * oRot.z());
    double yaw = atan2(siny, cosy);
    yamlFile << "origin: [" << std::to_string(oTrans.x()) << ", " << std::to_string(oTrans.y())
             << ", " << std::to_string(yaw) << "]" << std::endl;
    // TODO set through params?
    yamlFile << "occupied_thresh: 0.99" << std::endl;
    yamlFile << "free_thresh: 0.01" << std::endl;
    yamlFile << "negate: 0" << std::endl; // image is negated
    yamlFile.close();

    // write image file
    // image is ASCII exchange file format, may be more efficient to use binary format
    // see

    cv::Mat image(gridHelper->getGrid().height(), gridHelper->getGrid().width(), CV_8UC1);
    for (size_t i = 0; i < image.total(); ++i) {
        signed char val = gridHelper->getGrid().data().at(i);
        if (val == -1) {
             image.at<uchar>(i) = 128;
        } else {
            image.at<uchar>(i) = static_cast<uchar>(255 - 2.55 * val);
        }
    }
    cv::flip(image, image, 1);
    cv::imwrite(vars["mapFile"].as<std::string>() + ".png", image);

    return 0;
}
