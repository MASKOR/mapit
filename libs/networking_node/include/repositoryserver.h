#ifndef REPOSITORYSERVER_H
#define REPOSITORYSERVER_H

namespace upns {

class RepositoryServer
{
public:
    virtual ~RepositoryServer() {}

    /**
     * @brief handleRequest handles the next request and blocks until one is received or timeout is reached.
     * If milliseconds is 0, it returns immediatly if there is no request.
     * If milliseconds is set to -1, this will wait infinite.
     * Can only be stopped by an interrupt manually.
     */
    virtual void handleRequest(int milliseconds) = 0;
};

}

#endif
