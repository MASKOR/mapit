#ifndef __TESTMAPMANAGER_H
#define __TESTMAPMANAGER_H

#include <QTest>

#include "../../../libs/mapmanager/src/mapmanager.h"

class TestMapManager : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testListMaps_data();
    void testListMaps();

    void testGetMaps();

    void testStoreMaps();

private:
    upns::MapManager *m_mapManager;
    upns::MapService *m_mapService;
};

#endif
