#ifndef __TF_TEST_H
#define __TF_TEST_H

#include <QTest>

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

    void test_tf();
private:
    void createTestdata();
    void compareTfs(upns::tf::TransformStamped a, upns::tf::TransformStamped b);
    void test_input_output();
    void test_chain_of_2_tfs();
    void test_interpolation();

};

#endif
