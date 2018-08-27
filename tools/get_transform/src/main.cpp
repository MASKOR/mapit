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
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/errorcodes.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>
#include <mapit/breadthFirstSearch.h>
#include <mapit/time/time.h>

namespace po = boost::program_options;

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>(), "the workspace (formerly checkout) to work with")
            ("frame_id,f", po::value<std::string>(), "The source frame_id")
            ("child_frame_id,c", po::value<std::string>(), "The target frame_id")
            ("sec,s", po::value<long>(), "the sec part of the stamp")
            ("nsec,n", po::value<long>(), "the nsec part of the stamp")
            ("tf-prefix,p", po::value<std::string>()->default_value(""), "the prefix for tfs (optional)")
            ;
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1);
    pos_options.add("frame_id",  2);
    pos_options.add("child_frame_id",  3);
    pos_options.add("sec",  4);
    pos_options.add("nsec",  5);

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

    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );
    if ( ! repo ) {
        log_error("can't open repo");
        return 1;
    }
    std::shared_ptr<mapit::Workspace> ws = repo->getWorkspace( vars["workspace"].as<std::string>() );
    if ( ! ws ) {
        log_error("can't open workspace" << vars["workspace"].as<std::string>());
        return 1;
    }

    mapit::tf2::BufferCore tf_buffer(ws.get(), vars["tf-prefix"].as<std::string>());

    try {
        mapit::tf::TransformStamped tf = tf_buffer.lookupTransform(  vars["frame_id"].as<std::string>()
                                                                   , vars["child_frame_id"].as<std::string>()
                                                                   , mapit::time::from_sec_and_nsec(vars["sec"].as<long>(), vars["nsec"].as<long>())
                                                 );
        Eigen::Vector3f euler = tf.transform.rotation.toRotationMatrix().eulerAngles(0, 1, 2);
        log_info("TF " << tf.frame_id << " -> " << tf.child_frame_id << std::endl
                       << "at time " << mapit::time::to_string(tf.stamp) << " human readable: " << mapit::time::to_string_human(tf.stamp)
                       << std::endl
                       << "rotation    (rpy) rad: " << euler.transpose() << std::endl
                       << "rotation    (rpy) deg: " << (euler / M_PI * 180.).transpose() << std::endl
                       << "rotation    (wxyz):    " << tf.transform.rotation.w() << " "
                                                    << tf.transform.rotation.vec().transpose() << std::endl
                       << std::endl
                       << "translation (xyz):     " << tf.transform.translation.vector().transpose()
            );
    } catch (...) {
        log_error("can't lookup transform " << vars["frame_id"].as<std::string>() << " -> " << vars["child_frame_id"].as<std::string>()
                  << " at time " << vars["sec"].as<long>() << "." << vars["nsec"].as<long>()
            );
        return 1;
    }

    return 0;
}
