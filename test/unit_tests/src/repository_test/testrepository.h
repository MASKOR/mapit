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
private:
    std::string filename_;
    std::string checkoutPath_;
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
    void testReadCheckout_data();
    void testReadCheckout();
    void testCommit_data();
    void testCommit();
    void testVoxelgridfilter_data();
    void testVoxelgridfilter();
};

#endif
