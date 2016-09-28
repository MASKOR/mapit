#include "testoperators.h"
#include "upns_globals.h"
#include "error.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "datastructs.pb.h"

using namespace upns;

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Repository>)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

void TestOperators::init()
{
}

void TestOperators::cleanup()
{
}

void TestOperators::initTestCase()
{
    createTestdata();
}

void TestOperators::cleanupTestCase()
{
}

void TestOperators::testOperatorLoadPointcloud()
{
    QFETCH(upns::upnsSharedPointer<upns::Checkout>, checkout);
    OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    desc.set_params("{\"filename\":\"data/1465223257087387.pcd\", \"target\":\"corridor/laser/eins\"}");
    OperationResult ret = checkout->doOperation( desc );
    QVERIFY( upnsIsOk(ret.first) );
    upnsSharedPointer<Entity> ent = checkout->getEntity("corridor/laser/eins");
    QVERIFY( ent != nullptr );
    QVERIFY( ent->type() == upns::POINTCLOUD2 );
    upnsSharedPointer<Tree> parent = checkout->getTree("corridor/laser");
    QVERIFY( parent != nullptr );
    QVERIFY( !(*parent->mutable_refs())["eins"].id().empty() );
    upnsSharedPointer<AbstractEntityData> abstractentitydataByPath = checkout->getEntitydataReadOnly("corridor/laser/eins");
    upnsSharedPointer<AbstractEntityData> abstractentitydataByRef = checkout->getEntitydataReadOnly( (*parent->mutable_refs())["eins"].id() );

    upnsSharedPointer<PointcloudEntitydata> entitydataPC2ByPath = upns::static_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
    upnsSharedPointer<PointcloudEntitydata> entitydataPC2ByRef = upns::static_pointer_cast<PointcloudEntitydata>(abstractentitydataByRef);
    upnsPointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
    upnsPointcloud2Ptr pc2ref  = entitydataPC2ByPath->getData(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2ref, *cloudRef);

    pcl::PointCloud<pcl::PointXYZ>::Ptr file (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *file) == -1) //* load the file
    {
        QFAIL ("Couldn't read file data/bunny.pcd \n");
    }

    for(int i=0 ; qMin(100, (int)file->width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = cloudPath->at(i);
        pcl::PointXYZ &p2 = cloudRef->at(i);
        pcl::PointXYZ &p3 = file->at(i);
        QCOMPARE_REALVEC3(p1, p2);
        QCOMPARE_REALVEC3(p1, p3);
    }
}

DECLARE_TEST(TestOperators)
