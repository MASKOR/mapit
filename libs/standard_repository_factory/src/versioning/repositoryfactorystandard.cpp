#include "versioning/repositoryfactorystandard.h"
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include "versioning/repositoryfactory.h"
#include "versioning/repositorynetworkingfactory.h"
#include "upns_logging.h"

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

upns::Repository *upns::RepositoryFactoryStandard::openRepository(int argc, char * argv[])
{
    //TODO: Never tested...
    YAML::Node config;
    YAML::Node mapsource;
    bool mapsourceSpecified = false;
    char* yamlFile = getCmdOption(argv, argv + argc, "--yaml");
    if(yamlFile)
    {
        config = YAML::LoadFile( yamlFile );
        if(!config.IsNull() || !config.IsDefined())
        {
            log_error("Specified Yaml config file cannot be read.");
            log_info(usage());
            return nullptr;
        }
    }
    char* repositoryDirectory = getCmdOption(argv, argv + argc, "--repository-directory");
    char* repositoryUrl = getCmdOption(argv, argv + argc, "--url");
    if(repositoryDirectory && repositoryUrl)
    {
        log_error("Multiple Repositories specified. (--repository-directory, --url)");
        log_info(usage());
        return nullptr;
    }
    if(repositoryDirectory)
    {
        mapsourceSpecified = true;
        mapsource["name"] = "FileSystem";
        mapsource["filename"] = repositoryDirectory;
        config["mapsource"] = mapsource;
        return RepositoryFactory::openLocalRepository(config);
    }
    else if(repositoryUrl)
    {
        bool computeLocal = cmdOptionExists(argv, argv + argc, "--compute-local");
        char* compute = getCmdOption(argv, argv + argc, "--compute");
        if(computeLocal && compute)
        {
            log_error("Please use only --compute-local, or --compute");
            log_info(usage());
            return nullptr;
        }
        if(compute)
        {
            if(0 == strcasecmp(compute, "local"))
            {
                computeLocal = true;
            }
            else if(0 == strcasecmp(compute, "remote"))
            {
                computeLocal = false;
            }
            else
            {
                log_error("Wrong paramter for --compute (allowed: local, remote)");
                log_info(usage());
                return nullptr;
            }
        }
        return RepositoryNetworkingFactory::connectToRemoteRepository(repositoryUrl, nullptr, computeLocal);
    }
    else
    {
        log_error("No Repository specified. (--repository-directory, --url)");
        log_info(usage());
        return nullptr;
    }
}

void upns::RepositoryFactoryStandard::addProgramOptions(boost::program_options::options_description &desc)
{
    desc.add_options()
            ("yaml", boost::program_options::value<std::string>(), "read config as yaml. Options provided to the tool will overwrite values in the yaml config")
            ("repository-directory", boost::program_options::value<std::string>(), "directory to store data locally")
            ("url", boost::program_options::value<std::string>(), "remote repository url")
            //("compute", po::value<std::string>(), "")
            ("compute-local", boost::program_options::bool_switch(), "only if remote repository with option \"--url\" is used");
}

upns::Repository *upns::RepositoryFactoryStandard::openRepository(boost::program_options::variables_map &vars)
{
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
    if(vars.count("repository-directory") && vars.count("url"))
    {
        log_error("Multiple Repositories specified. (--repository-directory, --url)");
        log_info(usage());
        return nullptr;
    }
    if(vars.count("repository-directory"))
    {
        mapsource["name"] = "FileSystem";
        mapsource["filename"] = vars["repository-directory"].as<std::string>();
        config["mapsource"] = mapsource;
        return RepositoryFactory::openLocalRepository(config);
    }
    else if(vars.count("url"))
    {
        return RepositoryNetworkingFactory::connectToRemoteRepository(vars["url"].as<std::string>(), nullptr, vars["compute-local"].as<bool>());
    }
    else if(!mapsource.IsNull() && mapsource.IsDefined())
    {
        return RepositoryFactory::openLocalRepository(config);
    }
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
            "--yaml <config filename> read config as yaml. Options provided to the tool will overwrite values in the yaml config"
            "--repository-directory <directory>"
            "--url <remote repository url>"
            "  --compute <local|remote> only if remote repository with option \"--url\" is used"
            "  --compute-local"
            "";
}
