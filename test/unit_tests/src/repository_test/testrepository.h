#ifndef TESTREPOSITORY_H
#define TESTREPOSITORY_H

#include <QTest>

#include "versioning/repository.h"
#include "../repositorycommon.h"

namespace upns {
class RepositoryServer;
}

class TestRepository : public RepositoryCommon
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateCheckout_data();
    void testCreateCheckout();
    void testGetCheckout_data();
    void testGetCheckout();
    void testCommit_data();
    void testCommit();
    void testVoxelgridfilter_data();
    void testVoxelgridfilter();
};

#endif
