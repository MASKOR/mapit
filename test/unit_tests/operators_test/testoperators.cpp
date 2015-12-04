#include "testoperators.h"
#include "upns_globals.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include "layertypes/pointcloud2/include/pointcloudlayer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace upns;

void TestOperators::init()
{
}

void TestOperators::cleanup()
{
}

void TestOperators::initTestCase()
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

void TestOperators::cleanupTestCase()
{
    delete m_mapService;
}

//TODO: rename service/serializer. Is abstraction ok? (If networking is implemented, will there be another layer? Maybe MapService has to handle all protobuf stuff. Protobuf can not be changed easily, can it? (Ids, addLayers, ...)))
void TestOperators::testOperatorLoadPointcloud()
{
    //TODO: Use strict paramter-struct for each individual operation.
    OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    OperationParameter* param = desc.add_params();
    param->set_key("filename");
    param->set_strval("data/bunny.pcd");
    m_mapManager->doOperation( desc );

    upnsVec<MapIdentifier> maps = m_mapManager->listMaps();
    QVERIFY( maps.size() == 1);
    upnsSharedPointer<Map> map = m_mapManager->getMap(maps[0]);
    upnsSharedPointer<AbstractEntityData> abstractEntityData = m_mapService->getEntityData(map->id(), map->layers(0).id(), map->layers(0).entities(0).id());
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(abstractEntityData);
    upnsPointcloud2Ptr pc2 = entityData->getData(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr file (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *file) == -1) //* load the file
    {
        QFAIL ("Couldn't read file data/bunny.pcd \n");
    }

    for(int i=0 ; qMin(100, (int)file->width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = cloud->at(i);
        pcl::PointXYZ &p2 = file->at(i);
        QCOMPARE_REALVEC3(p1, p2);
    }
}

DECLARE_TEST(TestOperators)
