/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/logging.h>
#include <boost/filesystem.hpp>
//#include <boost/iostreams/device/mapped_file.hpp>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
//namespace iostr = boost::iostreams;

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    mapit_init_logging();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name> <destination>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>()->required(), "")
            ("destination,d", po::value<std::string>()->required(), "");
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1)
               .add("destination",  1);

    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);

    std::unique_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );

    std::shared_ptr<mapit::Workspace> workspace = repo->getWorkspace( vars["workspace"].as<std::string>() );
    if(workspace == nullptr)
    {
        log_error("Workspace: " + vars["workspace"].as<std::string>() + "not found");
        std::vector<std::string> possibleWorkspaces = repo->listWorkspaceNames();
        if (possibleWorkspaces.size() == 0) {
            log_info("No possible workspace");
        }
        for ( std::string workspace : possibleWorkspaces ) {
            log_info("Possible workspace: " + workspace);
        }
        return 1;
    }

    std::shared_ptr<Tree> currentDirectory(workspace->getRoot());
    fs::path rootPath(vars["destination"].as<std::string>());
    if ( rootPath.leaf() != vars["workspace"].as<std::string>() ) {
        // otherwise, use subfolder with workspace name
        rootPath /= fs::path( vars["workspace"].as<std::string>() );
    }

    mapit::StatusCode s = workspace->depthFirstSearch(
        [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const mapit::Path &path)
        {
            fs::path current( rootPath );
            fs::path path_new = rootPath / fs::path(path);
            if( ! path.empty() && ! fs::exists( path_new ) ) {
                if ( ! fs::create_directories(path_new) ) {
                    log_error("Path " + path_new.string() + " could not be created");
                    return false;
                }
            }
            return true;
        }, [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const mapit::Path &path)
        {
            return true;
        },
        [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const mapit::Path &path)
        {
            fs::path current(rootPath / fs::path(path));
            if ( fs::exists( current ) ) {
                 //QByteArray ar = current.readAll();
                 //Hash
            } else {
            }

            // get the stream to write into a file
            std::shared_ptr<mapit::AbstractEntitydata> reader = workspace->getEntitydataReadOnly(path);
            mapit::istream *entityStream = reader->startReadBytes();

            // calculate the step size to write into the file
            size_t entitySize = reader->size();
            size_t offsetMax = 4 * 1024 * 1024; // 4 MB // what makes sense? 100MB?
            size_t offsetStep = entitySize;
            if ( offsetStep > offsetMax ) {
                offsetStep = offsetMax;
            }

            // loop to write into the file in step width
            bool writeIsDone = false;
            std::ofstream file_out(current.string()/*, std::ofstream::app*/);
            for (size_t offsetIterator = 0; ! writeIsDone; offsetIterator++) {
                char* buffer = new char[offsetStep];

                if ( entityStream->read(buffer, offsetStep) ) { // if the end is not reached
                    file_out.write(buffer, offsetStep);
                } else {
                    size_t left = entityStream->gcount();
                    file_out.write(buffer, left);  // write the left rest
                    writeIsDone = true; // stop writing with next loop
                }
                delete[] buffer;

//                // open a mapped file of size of step
//                iostr::stream<iostr::mapped_file_sink> file;
//                file.open( iostr::mapped_file_sink(current),
//                           offsetStep,
//                           offsetIterator * offsetStep,
//                           iostr::mapped_file::readwrite );


//                iostr::mapped_file_sink file;
//                file.open(current,
//                          offsetStep,
//                          offsetIterator * offsetStep,
//                          iostr::mapped_file::readwrite);
//                if ( file.is_open() ) {
//                    // write into the file
//                    if ( ! entityStream->read(file.data(), offsetStep) ) { // if the end is reached
//                        writeIsDone = true; // stop writing with next loop
//                    }

//                    file.close();
//                } else {
//                    log_error("could not write entity: " + path);
//                }
            }
            reader->endRead(entityStream);
            file_out.close();

            return true;
        },
        [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const mapit::Path &path)
        {
            return true;
        });
}
