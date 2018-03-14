/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/versioning/repositoryfactorystandard.h"
#include <algorithm>
#ifdef WITH_YAML
#  include <yaml-cpp/yaml.h>
#endif
#include <mapit/versioning/repositoryfactory.h>
#include <mapit/versioning/repositorynetworkingfactory.h>
#include <mapit/logging.h>

void mapit::RepositoryFactoryStandard::addProgramOptions(boost::program_options::options_description &desc)
{
    desc.add_options()
#ifdef WITH_YAML
            ("yaml", boost::program_options::value<std::string>(), "read config as yaml. Options provided to the tool will overwrite values in the yaml config")
#endif
            ("repository-directory", boost::program_options::value<std::string>(), "directory to store data locally")
            ("url", boost::program_options::value<std::string>(), "remote repository url")
            //("compute", po::value<std::string>(), "")
            ("compute-local", boost::program_options::bool_switch(), "only if remote repository with option \"--url\" is used");
}

mapit::Repository *mapit::RepositoryFactoryStandard::openRepository(boost::program_options::variables_map &vars, bool *specified)
{
#ifdef WITH_YAML
    //TODO: remove yaml, it conflicts with program options and generates too complex
    //      constructs for checking all params. Many cases not tested here.
    YAML::Node config;
    YAML::Node mapsource;
    if(vars.count("yaml"))
    {
        config = YAML::LoadFile( vars["yaml"].as<std::string>() );
        mapsource = config["mapsource"];
        if(config.IsNull() || !config.IsDefined())
        {
            log_error("Specified Yaml config file cannot be read.");
            log_info(usage());
            if(specified) *specified = false;
            return nullptr;
        }
    }
#endif
    if(vars.count("repository-directory") && vars.count("url"))
    {
        log_error("Multiple Repositories specified. (--repository-directory, --url)");
        log_info(usage());
        if(specified) *specified = false;
        return nullptr;
    }
    if(vars.count("repository-directory"))
    {
        if(specified) *specified = true;
        return RepositoryFactory::openLocalRepository(vars["repository-directory"].as<std::string>());
    }
    else if(vars.count("url"))
    {
        if(specified) *specified = true;
        return RepositoryNetworkingFactory::connectToRemoteRepository(vars["url"].as<std::string>(), nullptr, vars["compute-local"].as<bool>());
    }
#ifdef WITH_YAML
    else if(!mapsource.IsNull() && mapsource.IsDefined())
    {
        if(specified) *specified = true;
        return RepositoryFactory::openLocalRepository(config);
    }
#endif
    else
    {
        if(specified) *specified = false;
        return RepositoryFactory::openLocalRepository(".");
//        log_error("No Repository specified. (--repository-directory, --url)");
//        log_info(usage());
//        return nullptr;
    }
}

mapit::Repository *mapit::RepositoryFactoryStandard::openRepositorySimple(std::string url, bool computeLocal)
{
    std::string urlPrefix("tcp://");
    if(!url.compare(0, urlPrefix.length(), urlPrefix))
    {
        return RepositoryNetworkingFactory::connectToRemoteRepository(url, nullptr, computeLocal);
    }
    else
    {
        return RepositoryFactory::openLocalRepository(url);
    }
}

const char* mapit::RepositoryFactoryStandard::usage()
{
    return  "mapit tool usage:"
#ifdef WITH_YAML
            "--yaml <config filename> read config as yaml. Options provided to the tool will overwrite values in the yaml config"
#endif
            "--repository-directory <directory>"
            "--url <remote repository url>"
            "  --compute <local|remote> only if remote repository with option \"--url\" is used"
            "  --compute-local"
            "";
}
