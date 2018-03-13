/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
    static Repository* openRepository(boost::program_options::variables_map &vars, bool *specified=nullptr);

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
