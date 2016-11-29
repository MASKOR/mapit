#include <iostream>
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <yaml-cpp/yaml.h>
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>
#include <fstream>
#include <sstream>

namespace fs = boost::filesystem;
namespace iostr = boost::iostreams;

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + "<checkout name> <destination>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("checkout,co", po::value<std::string>()->required(), "")
            ("destination,d", po::value<std::string>()->required(), "");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1)
               .add("destination",  1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name> <destination>" << std::endl;
        return 1;
    }
    po::notify(vars);

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    upns::upnsSharedPointer<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );
    if(co == nullptr)
    {
        log_error("Checkout: " + vars["checkout"].as<std::string>() + "not found");
        upns::upnsVec<upns::upnsString> possibleCheckouts = repo->listCheckoutNames();
        if (possibleCheckouts.size() == 0) {
            log_info("No possible checkout");
        }
        for ( upns::upnsString checkout : possibleCheckouts ) {
            log_info("Possible checkout: " + checkout);
        }
        return 1;
    }

    upns::upnsSharedPointer<upns::Tree> currentDirectory(co->getRoot());
    fs::path rootPath(argv[3]);
    if ( rootPath.filename() != argv[2] ) {
        // otherwise, use subfolder with checkout name
        rootPath /= fs::path( argv[2] );
    }

    upns::StatusCode s = co->depthFirstSearch([&](
        upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            fs::path current( rootPath );
            fs::path path_new = rootPath / fs::path(path);
            if( ! path.empty() && ! fs::exists( path_new ) ) {
                if ( ! fs::create_directories(path_new) ) {
                    log_error("Path " + path + " could not be created");
                    return false;
                }
            }
            return true;
        }, [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            fs::path current(rootPath / fs::path(path));
            if ( fs::exists( current ) ) {
                 //QByteArray ar = current.readAll();
                 //Hash
            } else {
            }

            // get the stream to write into a file
            upns::upnsSharedPointer<upns::AbstractEntitydata> reader = co->getEntitydataReadOnly(path);
            upns::upnsIStream *entityStream = reader->startReadBytes();

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
        [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        });
}
