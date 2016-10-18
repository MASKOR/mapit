#ifndef REPOSITORYCOMMON_H
#define REPOSITORYCOMMON_H

#include <QTest>
#include <QMutex>

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
    void cleanupTestdata();
    void startServer();
    void stopServer();
private:
    upns::upnsSharedPointer<upns::Repository> m_repo[3];
    upns::upnsSharedPointer<upns::Checkout> m_checkout[3];
    std::function<void()> m_serverCallback;
    upns::upnsSharedPointer<ServerThread> m_serverThread;
    upns::upnsSharedPointer<upns::Repository> m_networkRepo;

    // Start only after stop has finished. Let the last recv() run into it's timeout.
    QMutex m_serverThreadMutex;
};

#endif
