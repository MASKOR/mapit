#ifndef __TF_TEST_H
#define __TF_TEST_H

#include <QTest>
#include "../repositorycommon.h"

namespace upns {
namespace tf {
    struct TransformStamped;
}
}

class TFTest : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_input_output();
    void test_chain_of_2_tfs();
    void test_interpolation();

    void test_layertype_to_buffer();
private:
    char* fileSystemName_;
    std::shared_ptr<upns::Repository> repo_;
    std::shared_ptr<upns::Checkout> checkout_;

    void createTestdata();

    void compareTfs(upns::tf::TransformStamped a, upns::tf::TransformStamped b);
};

#endif
