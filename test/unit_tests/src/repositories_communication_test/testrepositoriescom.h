#ifndef TESTREPOSITORIESCOM_H
#define TESTREPOSITORIESCOM_H

#include <QTest>

#include "versioning/repository.h"

namespace upns {
class RepositoryServer;
}

class TestRepositoriesCommunication : public QObject
{
    Q_OBJECT
private slots:

    void initTestCase();
    void cleanupTestCase();

    void init();
    void cleanup();
    void testReadLocalComputeRemoteWriteLocal();
    void testReadRemoteComputeLocalWriteRemote();
    void testReadRemoteComputeRemoteWriteLocal();
private:
    upns::upnsSharedPointer<upns::Repository> initEmptyLocal();
    upns::upnsSharedPointer<upns::Repository> initNetwork(bool computeLocal);

    //// functions to execute on the repositories
    void readPcd(upns::Checkout *checkout);
    void voxelgrid(upns::Checkout* checkout);
    void writePcdLocalOnly(upns::Checkout* checkout, std::string filename);
};

#endif
