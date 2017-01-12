#ifndef REPOSITORYFACTORYSTANDARD_H
#define REPOSITORYFACTORYSTANDARD_H

#include "upns_typedefs.h"
#include "versioning/repository.h"
#include "repositoryserver.h"
#include <boost/program_options.hpp>

namespace upns
{

/**
 * @brief The RepositoryNetworkingFactory class
 * Returned Repositories must be deleted manually. Caller has ownership.
 * Deleting the object does NOT delete the repositories contents from disk.
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
