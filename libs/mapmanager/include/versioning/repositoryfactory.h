#ifndef REPOSITORYFACTORY_H
#define REPOSITORYFACTORY_H

#include "upns_typedefs.h"
#include "yaml-cpp/yaml.h"
#include "versioning/repository.h"

namespace upns
{
class RepositoryFactory
{
public:
    /**
     * @brief openLocalRepository. Opens a Repository on disc or creates an empty repository.
     * @param filename of yaml config
     * It communicates directly to file serializer.
     * @return
     */
    static upns::Repository* openLocalRepository(const upnsString &filename);

    /**
     * @brief openLocalRepository. Opens a Repository on disc or creates an empty repository.
     * @param config Yaml configuration of the repository
     * It communicates directly to file serializer.
     * @return
     */
    static upns::Repository* openLocalRepository(const YAML::Node &config);
};

}
#endif
