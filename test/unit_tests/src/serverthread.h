#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <upns/typedefs.h>
#include <repositoryserver.h>

// Client Calls send() followed by stopServer().
// The server waits MAX_NETWORK_LATENCY ms and tries to hanlde the last message in that time
#define MAX_NETWORK_LATENCY 20

class ServerThread : public QThread
{
    Q_OBJECT
    upns::upnsSharedPointer<upns::RepositoryServer> m_srv;
    bool m_isStopped;
    QMutex m_mutex;
public:
    ServerThread(upns::upnsSharedPointer<upns::RepositoryServer> server):m_srv(server), m_isStopped(false) {}

    void run() Q_DECL_OVERRIDE
    {
        QMutexLocker l(&m_mutex);
        Q_UNUSED(l);
        while(!m_isStopped)
        {
            m_srv->handleRequest(100);
        }
    }
public slots:
    void stop()
    {
        m_isStopped = true;
        QMutexLocker l(&m_mutex); // finish loop
        m_isStopped = false;
        m_srv->handleRequest(MAX_NETWORK_LATENCY); // there might be something not send/received yet.
    }
};
#endif
