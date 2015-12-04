#ifndef __TESTMAPFILESERVICE_H
#define __TESTMAPFILESERVICE_H

#include <QTest>

#include "../../../mapmanager/src/mapleveldb/mapleveldbserializer.h"

class TestMapFileService : public QObject
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

    void testStreamProvider();
private:
    upns::MapLeveldbSerializer *m_mapService;
};

#endif
