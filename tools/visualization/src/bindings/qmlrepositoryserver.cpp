#include "upns/ui/bindings/qmlrepositoryserver.h"
#include "upns/versioning/repositorynetworkingfactory.h"
#include "../serverthread.h"

QmlRepositoryServer::QmlRepositoryServer(QObject *parent)
    :QObject(parent)
    ,m_running(false)
    ,m_port(55512)
{

}

QmlRepositoryServer::~QmlRepositoryServer()
{

}

QmlRepository* QmlRepositoryServer::repository() const
{
    return m_repository;
}

bool QmlRepositoryServer::running() const
{
    return m_running;
}

void QmlRepositoryServer::setRepository(QmlRepository *repository)
{
    if (m_repository == repository)
        return;
    if(m_repository != NULL && m_connection != NULL)
    {
        disconnect(*m_connection);
        m_connection = NULL;
    }
    m_repository = repository;
    if(m_repository)
    {
        std::shared_ptr<QMetaObject::Connection> con( new QMetaObject::Connection(
                                                                  connect(m_repository,
                                                                          &QmlRepository::internalRepositoryChanged,
                                                                          this,
                                                                          &QmlRepositoryServer::reconnect)));
        m_connection = con;
    }
    reconnect();
    Q_EMIT repositoryChanged(m_repository);
}

int QmlRepositoryServer::port() const
{
    return m_port;
}

void QmlRepositoryServer::setPort(int port)
{
    if (m_port == port)
        return;

    m_port = port;
    reconnect();
    Q_EMIT portChanged(m_port);
}

void QmlRepositoryServer::setRunning(bool running)
{
    if (m_thread && m_thread->isRunning() == running)
        return;
    m_running = running;
    if(running)
    {
        reconnect();
    }
    else
    {
        reconnect();
    }
    //TODO: running change event should be deferred
    Q_EMIT runningChanged(m_running);
}

void QmlRepositoryServer::reconnect()
{
    if(m_running) {
        std::shared_ptr<upns::RepositoryServer> server( upns::RepositoryNetworkingFactory::openRepositoryAsServer(m_port, repository()->getRepository().get()) );
        m_thread.reset(new ServerThread(server));
        m_thread->start();
    }
    else
    {
        if(m_thread)
        {
            m_thread->stop();
        }
    }
}
