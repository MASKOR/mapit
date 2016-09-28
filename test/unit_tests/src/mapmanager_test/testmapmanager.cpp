#include "testmapmanager.h"
#include "upns_globals.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"

using namespace upns;

using Strings = upnsVec<upnsString>;
using Layers = upnsVec<Layer>;

void TestMapManager::init()
{
}

void TestMapManager::cleanup()
{
}

void TestMapManager::initTestCase()
{
    const char* databaseName = "test.db";
    QDir dir(databaseName);
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        QVERIFY( result );
    }
    YAML::Node conf;
    YAML::Node mapsource;
    mapsource["name"] = "MapFileService";
    mapsource["filename"] = databaseName;
    conf["mapsource"] = mapsource;

    m_mapManager = new upns::MapManager(conf);
    m_mapService = m_mapManager->getInternalMapService();
}

void TestMapManager::cleanupTestCase()
{
    delete m_mapService;
}

void TestMapManager::testListMaps_data()
{
    // Test basic CRUD
    QTest::addColumn< Strings >("names");
    QTest::addColumn< Layers >("layers");

    Layer testlayer1;
    testlayer1.set_name("testlayer1");
    testlayer1.set_type(LayerType::POINTCLOUD2);
    testlayer1.set_usagetype(LayerUsageType::LASER);
    Layer testlayer2;
    testlayer2.set_name("testlayer2");
    testlayer2.set_type(LayerType::OCTOMAP);
    testlayer2.set_usagetype(LayerUsageType::NAVIGATION);

    QTest::newRow("first")
            << Strings{ "hello"}
            << Layers{};

    QTest::newRow("with space")
            << Strings{ "hello world"}
            << Layers{};

    QTest::newRow("with delimiter")
            << Strings{ "hello!world"}
            << Layers{};

    QTest::newRow("with both")
            << Strings{ "hello! world"}
            << Layers{};

    QTest::newRow("short")
            << Strings{ "a"}
            << Layers{};

    QTest::newRow("same name twice")
            << Strings{ "hello",
                        "hello"}
            << Layers{};

    QTest::newRow("same name and delim")
            << Strings{ "hello",
                        "hello!hello",
                        "hello hello"}
            << Layers{};

    QTest::newRow("numbers and special characters")
            << Strings{ "1234567890",
                        "1testmap2",
                        "\0 special \0 characters",
                        "test \0 special",
                        "test \0 special2"}
            << Layers{};

    QTest::newRow("empty") << Strings{}
                           << Layers{};

    QTest::newRow("empty name")
            << Strings{""}
            << Layers{};

    QTest::newRow("with layer")
            << Strings{"layermap"}
            << Layers{  testlayer1 };
    QTest::newRow("with multiple layers")
            << Strings{"layermap"}
            << Layers{  testlayer1,
                        testlayer2 };
    QTest::newRow("with same layer twice")
            << Strings{"layermap"}
            << Layers{  testlayer1,
                        testlayer1,
                        testlayer2 };
}

void TestMapManager::testListMaps()
{
    QFETCH(Strings, names);
    QFETCH(Layers, layers);

    upnsVec<MapIdentifier> maps = m_mapService->listMaps();
    QCOMPARE(static_cast<int>(maps.size()), 0);

    // create maps
    MapVector storedMaps;
    foreach(upnsString name, names)
    {
        upns::upnsSharedPointer<upns::Map> map = m_mapService->createMap( name );
        QVERIFY(map != NULL);
        foreach(Layer layer, layers)
        {
            Layer *newLayer = map->add_layers();
            newLayer->CopyFrom( layer );
        }
        storedMaps.push_back( map );
    }
    // save map with layers
    MapResultsVector storeResults = m_mapService->storeMaps( storedMaps );
    bool suc = upnsCheckResultVector( storeResults );
    QVERIFY( suc );

    // list maps
    maps = m_mapService->listMaps();
    QCOMPARE(maps.size(), names.size());

    // get and read maps
    MapVector mapVec = m_mapService->getMaps( maps );
    QCOMPARE(maps.size(), mapVec.size());
    foreach(upnsString name, names)
    {
        int count = 0;          //< repeatition of name in original testdata
        int occurrences = 0;    //< repeatition of name in written/read data
        foreach(upnsString n2, names)
        {
            count += n2 == name;
        }
        bool atLeastOneWithSameLayers = false;
        for(MapVector::const_iterator iter(mapVec.begin()) ; iter != mapVec.end() ; iter++)
        {
            if( name == (*iter)->name())
            {
                occurrences++;

                // futher more test for exact same layers (ids must differ!)
                const int currentLayersSize = (*iter)->layers_size();
                if(layers.size() != currentLayersSize)
                {
                    // layers are not the same, difference in number of layers
                    continue;
                }
                // zero layers will jump over foreach.
                if(currentLayersSize == 0)
                {
                    atLeastOneWithSameLayers |= true;
                }
                else
                {
                    for(int i=0; i < currentLayersSize ; ++i)
                    {
                        const Layer &currentLayer = (*iter)->layers(i);
                        QVERIFY(currentLayer.id() != 0); //< After "store" id is given to layer.
                        foreach(Layer layer, layers)
                        {
                            bool equals = true;
                            equals &= currentLayer.name() == layer.name();
                            equals &= currentLayer.type() == layer.type();
                            equals &= currentLayer.usagetype() == layer.usagetype();
                            QVERIFY(layer.id() == 0);    //< id in testdata was never set.
                            atLeastOneWithSameLayers |= equals;
                            if(atLeastOneWithSameLayers)
                            {
                                break;
                            }
                        }
                        if(atLeastOneWithSameLayers)
                        {
                            break;
                        }
                    }
                }
            }
        }
        QVERIFY( atLeastOneWithSameLayers );
        QCOMPARE(occurrences, count);
    }

    // check last change timestamp
    for(MapVector::const_iterator iter(mapVec.begin()) ; iter != mapVec.end() ; iter++)
    {
        QDateTime lastChange;
        lastChange.setMSecsSinceEpoch( (*iter)->lastchange() );
        // last change was less than 5 secons ago and should have been set on create/store
        QVERIFY(QDateTime::currentDateTime().secsTo( lastChange ) < 5);
    }

    // remove all maps
    m_mapService->removeMaps( maps );

    maps = m_mapService->listMaps();
    QCOMPARE(static_cast<int>(maps.size()), 0);
}

void TestMapManager::testGetMaps()
{
}

void TestMapManager::testStoreMaps()
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

DECLARE_TEST(TestMapManager)
