#ifndef __TESTOPERATORS_H
#define __TESTOPERATORS_H

#include <QTest>

#include "../../../mapmanager/src/mapmanager.h"

class TestOperators : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testOperatorLoadPointcloud();
private:
    upns::MapManager *m_mapManager;
    upns::MapService *m_mapService;
};

#endif
