#include "testmapfileservice.h"
#include "upns_globals.h"
#include "../../src/autotestall.h"

using namespace upns;

void TestMapFileService::init()
{
    m_mapService = new upns::MapFileService("test.db");
}

void TestMapFileService::cleanup()
{
    delete m_mapService;
}

void TestMapFileService::testListMaps()
{
    m_mapService->listMaps();
}

void TestMapFileService::testGetMaps()
{
}

void TestMapFileService::testStoreMaps()
{
    upnsString testmapName("hello world");
    upns::upnsSharedPointer<upns::Map> map = m_mapService->createMap( testmapName );
    QVERIFY(map != NULL);
    upnsVec<MapIdentifier> maps = m_mapService->listMaps();
    QVERIFY(maps.size() >= 1);
    MapVector mapVec = m_mapService->getMaps( maps );
    bool foundTestMap = false;
    for(MapVector::const_iterator iter(mapVec.begin()) ; iter != mapVec.end() ; iter++)
    {
        if( testmapName == (*iter)->name())
        {
            foundTestMap = true;
            break;
        }
    }
    QVERIFY(foundTestMap);
}

DECLARE_TEST(TestMapFileService)
