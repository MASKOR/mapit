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

Q_DECLARE_METATYPE(upns::Repository*)
Q_DECLARE_METATYPE(std::function<void()>)

using namespace upns;

void TestEntitydata::init()
{
}

void TestEntitydata::cleanup()
{
}

void TestEntitydata::initTestCase()
{
    {
        //// Setup Repository in Filesystem
        const char* fileSystemName = ".mapit";
        QDir dir(fileSystemName);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        YAML::Node conf;
        YAML::Node mapsource;
        mapsource["name"] = "FileSystem";
        mapsource["filename"] = fileSystemName;//databaseName;
        conf["mapsource"] = mapsource;

        m_repo[0] = upns::RepositoryFactory::openLocalRepository(conf);
        m_checkout[0] = m_repo[0]->createCheckout("master", "testcheckout");
    }
    {
        //// Setup Repository as Database
        const char* databaseName = "test.db";
        QDir dir(databaseName);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        YAML::Node conf;
        YAML::Node mapsource;
        mapsource["name"] = "leveldb";
        mapsource["filename"] = databaseName;
        conf["mapsource"] = mapsource;

        m_repo[1] = upns::RepositoryFactory::openLocalRepository(conf);
        m_checkout[1] = m_repo[1]->createCheckout("master", "testcheckout");
    }
    {
        //// Setup Repository as Network connection
        const char* databaseName = "test.db";
        QDir dir(databaseName);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        YAML::Node conf;
        YAML::Node mapsource;
        mapsource["name"] = "leveldb";
        mapsource["filename"] = databaseName;
        conf["mapsource"] = mapsource;

        m_srv = upns::RepositoryNetworkingFactory::openRepositoryAsServer(1234, m_repo[1]);
        m_repo[2] = upns::RepositoryNetworkingFactory::connectToRemoteRepository("localhost:1234", NULL);
        m_checkout[2] = m_repo[2]->createCheckout("master", "testcheckout");
    }
}

void TestEntitydata::cleanupTestCase()
{
    delete m_checkout[0];
    m_checkout[0] = NULL;
    delete m_checkout[1];
    m_checkout[1] = NULL;
    delete m_checkout[2];
    m_checkout[2] = NULL;

    delete m_repo[0];
    m_repo[0] = NULL;
    delete m_repo[1];
    m_repo[1] = NULL;
    delete m_repo[2];
    m_repo[2] = NULL;
}

void TestEntitydata::testCreateLayer_data()
{
}

void TestEntitydata::testCreateLayer()
{
    QVERIFY(m_checkout->canRead());
    QVERIFY(m_checkout->canWrite());
    upnsSharedPointer<Map> map = m_mapService->createMap("myNewMap");
    Layer *layer = map->add_layers();
    layer->set_name("testlayer");
    layer->set_type(LayerType::POINTCLOUD2);
    layer->set_usagetype(LayerUsageType::LASER);
    Entity *entity = layer->add_entities();

    m_mapService->storeMap( map );
    assert(map->id() != 0);
    assert(map->id() != -1);
    assert(layer->id() != 0);
    assert(layer->id() != -1);
    assert(entity->id() != 0);
    assert(entity->id() != -1);
    upnsSharedPointer<AbstractEntityData> abstractData = m_mapService->getEntityData( map->id(), layer->id(), entity->id() );
    QCOMPARE(abstractData->layerType(), LayerType::POINTCLOUD2);
    upnsSharedPointer<PointcloudEntitydata> pointclouddata;
    pointclouddata = upns::static_pointer_cast<PointcloudEntitydata>(abstractData);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(-1.0, 0.0, 1.0));
    cloud.push_back(pcl::PointXYZ(-2.0, 3.0, 4.5));

    upnsPointcloud2Ptr pclpc2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2<pcl::PointXYZ>(cloud, *pclpc2);
    pointclouddata->setData(pclpc2);

    upnsSharedPointer<AbstractEntityData> abstractData2 = m_mapService->getEntityData( map->id(), layer->id(), entity->id() );
    QCOMPARE(abstractData2->layerType(), LayerType::POINTCLOUD2);
    upnsSharedPointer<PointcloudEntitydata> pointclouddata2;
    pointclouddata2 = upns::static_pointer_cast<PointcloudEntitydata>(abstractData2);

    upnsPointcloud2Ptr pclpc22 = pointclouddata2->getData();
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

DECLARE_TEST(TestEntitydata)
