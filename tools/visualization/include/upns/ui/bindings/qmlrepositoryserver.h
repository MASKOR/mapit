#ifndef QMLREPOSITORYSERVER_H
#define QMLREPOSITORYSERVER_H

#include <QtCore>
#include <QMetaObject>
#include "upns/ui/bindings/qmlrepository.h"
#include <upns/versioning/repository.h>

class ServerThread;
class QmlRepositoryServer : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool running READ running WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(QmlRepository* repository READ repository WRITE setRepository NOTIFY repositoryChanged)
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY portChanged)

public:
    QmlRepositoryServer(QObject *parent = nullptr);
    ~QmlRepositoryServer();
    QmlRepository* repository() const;
    bool running() const;
    int port() const;

public Q_SLOTS:
    void setRepository(QmlRepository* repository);
    void setPort(int port);
    void setRunning(bool running);

Q_SIGNALS:

    void repositoryChanged(QmlRepository* repository);
    void runningChanged(bool running);
    void portChanged(int port);

private Q_SLOTS:
    void reconnect();

private:
    bool m_running;
    int m_port;
    QmlRepository* m_repository;
    std::shared_ptr<ServerThread> m_thread;
    std::shared_ptr<QMetaObject::Connection> m_connection;
};

#endif
