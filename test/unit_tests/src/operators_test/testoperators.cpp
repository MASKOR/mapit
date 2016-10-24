#include "testoperators.h"
#include "upns_typedefs.h"
#include "upns_errorcodes.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "datastructs.pb.h"

#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"

#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"

using namespace upns;

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Repository>)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

void TestOperators::init()
{
    startServer();
}

void TestOperators::cleanup()
{
    stopServer();
}

void TestOperators::initTestCase()
{
    initTestdata();
}

void TestOperators::cleanupTestCase()
{
    cleanupTestdata();
}

void TestOperators::testOperatorLoadPointcloud_data() { createTestdata(); }
void TestOperators::testOperatorLoadPointcloud()
{
    QFETCH(upns::upnsSharedPointer<upns::Checkout>, checkout);
    OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    //desc.set_params("{\"filename\":\"data/1465223257087387.pcd\", \"target\":\"corridor/laser/eins\"}");
    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"corridor/laser/eins\"}");
    OperationResult ret = checkout->doOperation( desc );
    QVERIFY( upnsIsOk(ret.first) );
    upnsSharedPointer<Entity> ent = checkout->getEntity("corridor/laser/eins");
    QVERIFY( ent != nullptr );
    QVERIFY( ent->type() == upns::POINTCLOUD2 );
    upnsSharedPointer<Tree> parent = checkout->getTree("corridor/laser");
    QVERIFY( parent != nullptr );
    QVERIFY( !(*parent->mutable_refs())["eins"].id().empty() );
    upnsSharedPointer<AbstractEntitydata> abstractentitydataByPath = checkout->getEntitydataReadOnly("corridor/laser/eins");
    //upnsSharedPointer<AbstractEntitydata> abstractentitydataByRef = checkout->getEntitydataReadOnly( (*parent->mutable_refs())["eins"].id() );

    upnsSharedPointer<PointcloudEntitydata> entitydataPC2ByPath = upns::static_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
    //upnsSharedPointer<PointcloudEntitydata> entitydataPC2ByRef = upns::static_pointer_cast<PointcloudEntitydata>(abstractentitydataByRef);
    upnsPointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
    //upnsPointcloud2Ptr pc2ref  = entitydataPC2ByRef->getData(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(*pc2ref, *cloudRef);

    pcl::PointCloud<pcl::PointXYZ>::Ptr file (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *file) == -1) //* load the file
    {
        QFAIL ("Couldn't read file data/bunny.pcd \n");
    }

    for(int i=0 ; qMin(100, (int)file->width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = cloudPath->at(i);
        //pcl::PointXYZ &p2 = cloudRef->at(i);
        pcl::PointXYZ &p3 = file->at(i);
        //QCOMPARE_REALVEC3(p1, p2);
        QCOMPARE_REALVEC3(p1, p3);
    }
}

void TestOperators::testInlineOperator_data() { createTestdata(); }
void TestOperators::testInlineOperator()
{
    upnsString epath("/hello/test/entity");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.f,  2.f,  3.f));
    cloud.push_back(pcl::PointXYZ(4.f,  5.f,  6.f));
    cloud.push_back(pcl::PointXYZ(7.f,  8.f,  9.f));
    cloud.push_back(pcl::PointXYZ(0.f,  0.f,  0.f));
    cloud.push_back(pcl::PointXYZ(7.5f, 8.5f, 9.5f));

    QFETCH(upns::upnsSharedPointer<upns::Checkout>, checkout);
    OperationDescription desc;
    desc.set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    upns::OperationResult res = checkout->doUntraceableOperation(desc, [&cloud, &epath](upns::OperationEnvironment* env)
    {
        upns::CheckoutRaw *coraw = env->getCheckout();
        upns::upnsSharedPointer<upns::Entity> e(new upns::Entity);
        e->set_type(upns::POINTCLOUD2);
        upns::StatusCode status = coraw->storeEntity(epath, e);
        if(!upnsIsOk(status))
        {
            return UPNS_STATUS_ERROR;
        }
        upnsSharedPointer<AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );

        upns::upnsSharedPointer<pcl::PCLPointCloud2> cloud2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *cloud2);
        entityData->setData(cloud2);
        return UPNS_STATUS_OK;
    });
    QVERIFY(upnsIsOk(res.first));

    upnsSharedPointer<AbstractEntitydata> abstractentitydataByPath = checkout->getEntitydataReadOnly(epath);
    upnsSharedPointer<PointcloudEntitydata> entitydataPC2ByPath = upns::static_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
    upnsPointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
    for(int i=0 ; qMin(100, (int)cloud.width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = cloudPath->at(i);
        pcl::PointXYZ &p3 = cloud.at(i);
        QCOMPARE_REALVEC3(p1, p3);
    }
}

DECLARE_TEST(TestOperators)
