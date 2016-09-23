#ifndef REPOSITORYSERVER_H
#define REPOSITORYSERVER_H

#include <thread>

namespace upns {

class RepositoryServer
{
public:
    virtual void pollRequest() = 0;
};

}

#endif
