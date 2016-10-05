#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include "upns_globals.h"
#include <repositoryserver.h>

#define MAX_NETWORK_LATENCY 2

class ServerThread : public QThread
{
    Q_OBJECT
    upns::upnsSharedPointer<upns::RepositoryServer> m_srv;
    bool m_isStarted;
    QMutex m_mutex;
public:
    ServerThread(upns::upnsSharedPointer<upns::RepositoryServer> server):m_srv(server), m_isStarted(false) {}

    void run() Q_DECL_OVERRIDE
    {
        m_mutex.lock();
        m_isStarted = true;
        m_mutex.unlock();
        while(true)
        {
            QThread::currentThread()->msleep(10);

            QMutexLocker l(&m_mutex);
            Q_UNUSED(l);
            if(!m_isStarted) break;
            m_srv->handleRequest(0);
        }
    }
public slots:
    void stop()
    {
        m_mutex.lock();
        m_isStarted = false;
        m_srv->handleRequest(MAX_NETWORK_LATENCY); // there might be something not send/received yet.
        m_mutex.unlock();
    }
};
#endif
