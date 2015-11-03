#include "testlayerdata.h"
#include "upns_globals.h"
#include "../../src/autotestall.h"
#include "layertypes/pointcloud2/src/pointcloudlayer.h"
#include "yaml-cpp/yaml.h"
#include <QDir>
#include <QVector>
#include <QString>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

using namespace upns;

void TestLayerdata::init()
{
}

void TestLayerdata::cleanup()
{
}

void TestLayerdata::initTestCase()
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

    m_mapService = new upns::MapManager(conf);
}

void TestLayerdata::cleanupTestCase()
{
    delete m_mapService;
}

void TestLayerdata::testCreateLayer_data()
{
}

void TestLayerdata::testCreateLayer()
{
    QVERIFY(m_mapService->canRead());
    QVERIFY(m_mapService->canWrite());
    upnsSharedPointer<Map> map = m_mapService->createMap("myNewMap");
    Layer *layer = map->add_layers();
    layer->set_name("testlayer");
    layer->set_type(LayerType::POINTCLOUD2);
    layer->set_usagetype(LayerUsageType::LASER);

    m_mapService->storeMap( map );
    assert(map->id() != 0);
    assert(map->id() != -1);
    assert(layer->id() != 0);
    assert(layer->id() != -1);
    upnsSharedPointer<AbstractLayerData> abstractLayerData = m_mapService->getLayerData( map->id(), layer->id() );
    QCOMPARE(abstractLayerData->layerType(), LayerType::POINTCLOUD2);
    upnsSharedPointer<PointcloudLayerdata> pointcloudLayerdata;
    pointcloudLayerdata = upns::static_pointer_cast<PointcloudLayerdata>(abstractLayerData);
    upnsPointcloud2Ptr pclpc2(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(-1.0, 0.0, 1.0));
    cloud.push_back(pcl::PointXYZ(-2.0, 3.0, 4.5));
    pcl::toPCLPointCloud2<pcl::PointXYZ>(cloud, *pclpc2);
    pointcloudLayerdata->setData(pclpc2);

    upnsSharedPointer<AbstractLayerData> abstractLayerData2 = m_mapService->getLayerData( map->id(), layer->id() );
    QCOMPARE(abstractLayerData2->layerType(), LayerType::POINTCLOUD2);
    upnsSharedPointer<PointcloudLayerdata> pointcloudLayerdata2;
    pointcloudLayerdata2 = upns::static_pointer_cast<PointcloudLayerdata>(abstractLayerData2);

    upnsPointcloud2Ptr pclpc22 = pointcloudLayerdata2->getData();
    QCOMPARE(pclpc22->height, pclpc2->height);
    QCOMPARE(pclpc22->width, pclpc2->width);
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::fromPCLPointCloud2( *pclpc22, cloud2);
    QCOMPARE(cloud2.at(0).x, -1.0);
    QCOMPARE(cloud2.at(0).y, -0.0);
    QCOMPARE(cloud2.at(0).z,  1.0);
    QCOMPARE(cloud2.at(1).x, -2.0);
    QCOMPARE(cloud2.at(1).y,  3.0);
    QCOMPARE(cloud2.at(1).z,  4.5);
}

DECLARE_TEST(TestLayerdata)
