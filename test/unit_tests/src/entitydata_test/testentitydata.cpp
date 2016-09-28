#include "testentitydata.h"
#include "upns_globals.h"
#include "services.pb.h"
#include "../../src/autotest.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include <QJsonDocument>
#include <QJsonObject>
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include "versioning/repositorynetworkingfactory.h"
#include <functional>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "../src/serialization/file_system/fs_serializer.h"
#include "../src/serialization/leveldb/leveldbserializer.h"
using namespace upns;

Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::AbstractMapSerializer>)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::AbstractEntityDataStreamProvider>)

void TestEntitydata::init()
{
}

void TestEntitydata::cleanup()
{
}

void TestEntitydata::initTestCase()
{
    YAML::Node mapsource;
    mapsource["name"] = "FileSystem";
    mapsource["filename"] = "test_filesystem";
    m_ed[0] = upns::upnsSharedPointer<upns::AbstractMapSerializer>( new upns::FSSerializer( mapsource ));
    YAML::Node mapsource2;
    mapsource2["name"] = "leveldb";
    mapsource2["filename"] = "test_leveldb";
    m_ed[1] = upns::upnsSharedPointer<upns::AbstractMapSerializer>( new upns::LevelDBSerializer( mapsource2 ));
}

void TestEntitydata::cleanupTestCase()
{
}

void TestEntitydata::testCreateLayer_data() { createTestdata(); }
void TestEntitydata::testCreateLayer()
{
    QFETCH(upns::upnsSharedPointer<upns::AbstractMapSerializer>, serializer);
    QFETCH(upns::upnsSharedPointer<upns::AbstractEntityDataStreamProvider>, streamprovider);
    QVERIFY(serializer->canRead());
    QVERIFY(serializer->canWrite());
    // TODO: Call every method once. The class changed quite a lot.
//    upnsSharedPointer<Map> map = serializer->createMap("myNewMap");
//    Layer *layer = map->add_layers();
//    layer->set_name("testlayer");
//    layer->set_type(LayerType::POINTCLOUD2);
//    layer->set_usagetype(LayerUsageType::LASER);
//    Entity *entity = layer->add_entities();

//    m_mapService->storeMap( map );
//    assert(map->id() != 0);
//    assert(map->id() != -1);
//    assert(layer->id() != 0);
//    assert(layer->id() != -1);
//    assert(entity->id() != 0);
//    assert(entity->id() != -1);
//    upnsSharedPointer<AbstractEntityData> abstractData = m_mapService->getEntityData( map->id(), layer->id(), entity->id() );
//    QCOMPARE(abstractData->layerType(), LayerType::POINTCLOUD2);
//    upnsSharedPointer<PointcloudEntitydata> pointclouddata;
//    pointclouddata = upns::static_pointer_cast<PointcloudEntitydata>(abstractData);
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    cloud.push_back(pcl::PointXYZ(-1.0, 0.0, 1.0));
//    cloud.push_back(pcl::PointXYZ(-2.0, 3.0, 4.5));

//    upnsPointcloud2Ptr pclpc2(new pcl::PCLPointCloud2());
//    pcl::toPCLPointCloud2<pcl::PointXYZ>(cloud, *pclpc2);
//    pointclouddata->setData(pclpc2);

//    upnsSharedPointer<AbstractEntityData> abstractData2 = m_mapService->getEntityData( map->id(), layer->id(), entity->id() );
//    QCOMPARE(abstractData2->layerType(), LayerType::POINTCLOUD2);
//    upnsSharedPointer<PointcloudEntitydata> pointclouddata2;
//    pointclouddata2 = upns::static_pointer_cast<PointcloudEntitydata>(abstractData2);

//    upnsPointcloud2Ptr pclpc22 = pointclouddata2->getData();
//    QCOMPARE(pclpc22->height, pclpc2->height);
//    QCOMPARE(pclpc22->width, pclpc2->width);
//    pcl::PointCloud<pcl::PointXYZ> cloud2;
//    pcl::fromPCLPointCloud2( *pclpc22, cloud2);
//    QCOMPARE(cloud2.at(0).x, -1.0);
//    QCOMPARE(cloud2.at(0).y, -0.0);
//    QCOMPARE(cloud2.at(0).z,  1.0);
//    QCOMPARE(cloud2.at(1).x, -2.0);
//    QCOMPARE(cloud2.at(1).y,  3.0);
//    QCOMPARE(cloud2.at(1).z,  4.5);
}

void TestEntitydata::createTestdata()
{
    QTest::addColumn< upns::upnsSharedPointer<upns::AbstractMapSerializer> >("serializer");
    QTest::addColumn< upns::upnsSharedPointer<upns::AbstractEntityDataStreamProvider> >("streamprovider");

    QTest::newRow("filesystem")       << m_ed[0] << m_edsp[0];
    QTest::newRow("leveldb database") << m_ed[1] << m_edsp[1];
}

DECLARE_TEST(TestEntitydata)
