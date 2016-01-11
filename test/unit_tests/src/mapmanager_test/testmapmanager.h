#ifndef TESTMAPMANAGER_H
#define TESTMAPMANAGER_H

#include <QTest>

#include "../../../libs/mapmanager/src/versioning/repository.h"

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
    upns::Repository *m_repo;
};

#endif
