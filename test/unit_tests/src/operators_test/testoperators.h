#ifndef __TESTOPERATORS_H
#define __TESTOPERATORS_H

#include <QTest>
#include "../repositorycommon.h"

class TestOperators : public RepositoryCommon
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testOperatorLoadPointcloud_data();
    void testOperatorLoadPointcloud();
    void testInlineOperator_data();
    void testInlineOperator();
    void testPointcloudToMesh_data();
    void testPointcloudToMesh();
//    void testOperatorGrid_data();
//    void testOperatorGrid();
};

#endif
