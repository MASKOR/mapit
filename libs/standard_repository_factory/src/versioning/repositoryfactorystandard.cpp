#include "versioning/repositoryfactorystandard.h"
#include <algorithm>
#ifdef WITH_YAML
#  include <yaml-cpp/yaml.h>
#endif
#include "versioning/repositoryfactory.h"
#include "versioning/repositorynetworkingfactory.h"
#include "upns_logging.h"

void upns::RepositoryFactoryStandard::addProgramOptions(boost::program_options::options_description &desc)
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

upns::Repository *upns::RepositoryFactoryStandard::openRepository(boost::program_options::variables_map &vars)
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
            return nullptr;
        }
    }
#endif
    if(vars.count("repository-directory") && vars.count("url"))
    {
        log_error("Multiple Repositories specified. (--repository-directory, --url)");
        log_info(usage());
        return nullptr;
    }
    if(vars.count("repository-directory"))
    {
        return RepositoryFactory::openLocalRepository(vars["repository-directory"].as<std::string>());
    }
    else if(vars.count("url"))
    {
        return RepositoryNetworkingFactory::connectToRemoteRepository(vars["url"].as<std::string>(), nullptr, vars["compute-local"].as<bool>());
    }
#ifdef WITH_YAML
    else if(!mapsource.IsNull() && mapsource.IsDefined())
    {
        return RepositoryFactory::openLocalRepository(config);
    }
#endif
    else
    {
        log_error("No Repository specified. (--repository-directory, --url)");
        log_info(usage());
        return nullptr;
    }
}

const char* upns::RepositoryFactoryStandard::usage()
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
