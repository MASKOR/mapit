#ifndef REPOSITORYFACTORYSTANDARD_H
#define REPOSITORYFACTORYSTANDARD_H

#include <upns/typedefs.h>
#include <upns/versioning/repository.h>
#include "upns/repositoryserver.h"
#include <boost/program_options.hpp>

namespace upns
{

/**
 * @brief The RepositoryFactoryStandard class
 * Returned Repositories must be deleted manually. Caller has ownership.
 * Deleting the object does NOT delete the repositories contents from disk.
 * This factory is able to create repository objects configured using the command line.
 * Depending on the commandline arguments a networking repository may be created by the factory.
 */

class RepositoryFactoryStandard
{
public:

    static const char* usage();
    //static Repository* openRepository(int argc, char *argv[]);

    static void addProgramOptions(boost::program_options::options_description &desc);
    static Repository* openRepository(boost::program_options::variables_map &vars);
};

}
#endif
