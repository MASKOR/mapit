#ifndef __TESTMAPFILESERVICE_H
#define __TESTMAPFILESERVICE_H

#include <QTest>

#include "../../../mapmanager/src/mapmanager.h"

class TestLayerdata : public QObject
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
    upns::MapManager *m_mapService;
};

#endif
