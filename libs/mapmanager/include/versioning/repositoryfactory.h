#ifndef REPOSITORYFACTORY_H
#define REPOSITORYFACTORY_H

#include "upns_typedefs.h"
#include "versioning/repository.h"

namespace upns
{
class RepositoryFactory
{
public:
    /**
     * @brief openLocalRepository. Opens a Repository on disc or creates an empty repository.
     * @param directory for repository
     * It communicates directly to file serializer.
     * @return
     */
    static upns::Repository *openLocalRepository(std::__cxx11::string directory);
};

}
#endif
