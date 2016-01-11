#include "testmapfileservice.h"
#include "upns_globals.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include "error.h"

using namespace upns;

using Strings = upnsVec<upnsString>;
using Layers = upnsVec<Tree>;

Q_DECLARE_METATYPE( Strings );
Q_DECLARE_METATYPE( Layers );

void TestMapFileService::init()
{
}

void TestMapFileService::cleanup()
{
}

void TestMapFileService::initTestCase()
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

    m_repo = new upns::Repository(mapsource);
}

void TestMapFileService::cleanupTestCase()
{
    delete m_repo;
}

void TestMapFileService::testListMaps_data()
{
    // Test basic CRUD
    QTest::addColumn< Strings >("names");
    QTest::addColumn< Layers >("layers");

    Tree testlayer1;
    testlayer1.set_name("testlayer1");
    //testlayer1.set_type(LayerType::POINTCLOUD2);
    //testlayer1.set_usagetype(LayerUsageType::LASER);
    Layer testlayer2;
    testlayer2.set_name("testlayer2");
    //testlayer2.set_type(LayerType::OCTOMAP);
    //testlayer2.set_usagetype(LayerUsageType::NAVIGATION);

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

void TestMapFileService::testListMaps()
{
    QFETCH(Strings, names);
    QFETCH(Layers, layers);

    m_repo->getBranches();
    upnsVec<MapIdentifier> maps = m_repo->listMaps();
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



void TestMapFileService::testStreamProvider()
{
    upnsString testmapName("hello world");
    upns::upnsSharedPointer<upns::Map> map = m_mapService->createMap( "map123" );
    QVERIFY(map != NULL);
    Layer* layer = map->add_layers();
    layer->set_id(0); // will be overwritten
    layer->set_name("layer");
    layer->set_type(LayerType::OCTOMAP);
    layer->set_usagetype(LayerUsageType::NAVIGATION);
    Entity *entity = layer->add_entities();
    MapVector maps;
    maps.push_back( map );
    upns::MapResultsVector results = m_mapService->storeMaps( maps );
    QVERIFY( upnsCheckResultVector( results ) );
    QVERIFY( layer->id() != 0 );
    QVERIFY( layer->id() != -1 );
    QVERIFY( entity->id() != 0 );
    QVERIFY( entity->id() != -1 );
    upnsSharedPointer<AbstractEntityDataStreamProvider> streamProv = m_mapService->getStreamProvider(map->id(), layer->id(), entity->id());

    ///// Start of the StreamProvider Test /////

    std::string t1 = "Hello World";
    std::string t2 = "Dump data with no real layer interpretation";
    upnsOStream *os = streamProv->startWrite(0,0);
    *os << t1 << std::endl;
    *os << t2;
    *os << 1 << " " << 2 << " " << 3 << " ";
    streamProv->endWrite(os);
    upnsIStream *is = streamProv->startRead(0,0);
    char szBuff[128];
    is->getline(szBuff, sizeof(szBuff)); // getline
    QCOMPARE(t1, std::string(szBuff));
    is->get(szBuff, t2.size()+1);        // get
    szBuff[t2.size()] = '\0';
    QCOMPARE(t2, std::string(szBuff));
    int i1, i2, i3;
    *is >> i1;
    QCOMPARE(i1, 1);
    *is >> i2;
    QCOMPARE(i2, 2);
    *is >> i3;
    QCOMPARE(i3, 3);
    streamProv->endRead( is );
    is = streamProv->startRead(t1.length()+1,0);
    is->get(szBuff, t2.size()+1);
    szBuff[t2.size()] = '\0';
    QCOMPARE(t2, std::string(szBuff));
    *is >> i1;
    QCOMPARE(i1, 1);
    *is >> i2;
    QCOMPARE(i2, 2);
    *is >> i3;
    QCOMPARE(i3, 3);
    streamProv->endRead( is );
}

DECLARE_TEST(TestMapFileService)
