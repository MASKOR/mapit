#ifndef REPOSITORYFACTORYSTANDARD_H
#define REPOSITORYFACTORYSTANDARD_H

#include <upns/typedefs.h>
#include <upns/versioning/repository.h>
#include <upns/repositoryserver.h>
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

    /**
     * @brief addProgramOptions the parameter will contain information about command
     * line arguments which can be given to @sa openRepository
     * @param desc empty or prefilled description object.
     */
    static void addProgramOptions(boost::program_options::options_description &desc);

    /**
     * @brief openRepository connect to a repository with the given configuration in vars.
     * More information about arguments can be retrieved by using @sa addProgramOptions
     * @param vars arguments from command line, parsed by boost::program_options
     * @return a repository to work with. Caller takes ownership. Delete the object after usage or initialize a smart pointer with it.
     */
    static Repository* openRepository(boost::program_options::variables_map &vars);

    /**
     * @brief openRepositorySimple If no command line should be used, this method can be
     * used programmatically
     * @param url if starting with "tcp://" this method connects remotely. Otherwise a file/folder is used/created as a repository
     * @param computeLocal
     * @return a repository to wirk with. Caller takes ownership. Delete the object after usage or initialize a smart pointer with it.
     */
    static Repository* openRepositorySimple(std::string url, bool computeLocal);
};

}
#endif
