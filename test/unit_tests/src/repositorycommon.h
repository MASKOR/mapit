#ifndef REPOSITORYCOMMON_H
#define REPOSITORYCOMMON_H

#include <QTest>

#include "versioning/repository.h"

namespace upns {
class RepositoryServer;
}
class ServerThread;

class RepositoryCommon: public QObject
{
    Q_OBJECT
protected:
    void createTestdata();
    void initTestdata();
private:
    upns::upnsSharedPointer<upns::Repository> m_repo[3];
    upns::upnsSharedPointer<upns::RepositoryServer> m_srv;
    upns::upnsSharedPointer<upns::Checkout> m_checkout[3];
    std::function<void()> m_serverCallback;
    QSharedPointer<ServerThread> m_serverThread;
};

#endif
