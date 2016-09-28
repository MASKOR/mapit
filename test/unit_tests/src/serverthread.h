#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H
#include <QThread>
#include "upns_globals.h"
#include <repositoryserver.h>

class ServerThread : public QThread
{
    Q_OBJECT
public:
    ServerThread(upns::upnsSharedPointer<upns::RepositoryServer> server):m_srv(server) {}
    upns::upnsSharedPointer<upns::RepositoryServer> m_srv;
    void run() Q_DECL_OVERRIDE {
        m_srv->handleRequest();
    }
};
#endif
