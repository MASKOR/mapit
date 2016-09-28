#ifndef REPOSITORYSERVER_H
#define REPOSITORYSERVER_H

#include <thread>

namespace upns {

class RepositoryServer
{
public:
    virtual void handleRequest() = 0;
};

}

#endif