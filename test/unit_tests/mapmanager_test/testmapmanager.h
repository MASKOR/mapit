#ifndef __TESTMAPFILESERVICE_H
#define __TESTMAPFILESERVICE_H

#include <QTest>

#include "../../../mapmanager/src/mapmanager.h"

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
    upns::MapManager *m_mapService;
};

#endif