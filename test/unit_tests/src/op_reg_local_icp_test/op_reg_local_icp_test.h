#ifndef __OP_REG_LOCAL_ICP_TEST_H
#define __OP_REG_LOCAL_ICP_TEST_H

#include <QTest>
#include "../repositorycommon.h"

class OPRegLocalICPTest : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_icp_tf_add();
private:
    std::string fileSystemName_;
    std::shared_ptr<upns::Repository> repo_;
    std::shared_ptr<upns::Checkout> checkout_;

    void createTestdata();
};

#endif
