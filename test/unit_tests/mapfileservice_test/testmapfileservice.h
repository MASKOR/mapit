#ifndef __TESTMAPFILESERVICE_H
#define __TESTMAPFILESERVICE_H

#include <QTest>

#include "../../../mapmanager/src/mapfileservice.h"

class TestMapFileService : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void testListMaps();
    void testGetMaps();
    void testStoreMaps();

private:
    upns::MapFileService *m_mapService;
};

#endif
