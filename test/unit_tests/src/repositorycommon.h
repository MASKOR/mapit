#ifndef REPOSITORYCOMMON_H
#define REPOSITORYCOMMON_H

#include <QTest>
#include <QMutex>

#include <upns/versioning/repository.h>

namespace upns {
class RepositoryServer;
}
class ServerThread;

class RepositoryCommon: public QObject
{
    Q_OBJECT
protected:
    void createTestdata(bool withServer = false);
    void initTestdata();
    void cleanupTestdata();
    void startServer();
    void stopServer();
private:
    std::shared_ptr<upns::Repository> m_repo[3];
    std::shared_ptr<upns::Checkout> m_checkout[3];
    std::function<void()> m_serverCallback;
    std::shared_ptr<ServerThread> m_serverThread;
    std::shared_ptr<upns::Repository> m_networkRepo;

    // Start only after stop has finished. Let the last recv() run into it's timeout.
    QMutex m_serverThreadMutex;
};

#endif
