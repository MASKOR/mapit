#ifndef __TESTENTITYDATA_H
#define __TESTENTITYDATA_H

#include <QTest>

#include "../../../libs/mapmanager/src/mapmanager.h"
#include "../../../libs/mapmanager/src/mapservice.h"

class TestEntitydata : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateLayer_data();
    void testCreateLayer();

private:
    upns::MapManager *m_mapManager;
    upns::MapService *m_mapService;
};

#endif
