#include "testrepository.h"
#include "upns_typedefs.h"
#include "services.pb.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include <QJsonDocument>
#include <QJsonObject>
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include "versioning/repositorynetworkingfactory.h"
#include <pointcloudlayer.h>
#include <functional>
#include <pcl/io/pcd_io.h>
#include <iostream>

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Repository>)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

using namespace upns;

void TestRepository::init()
{
    filename_ = "data/bunny.pcd";
    checkoutPath_ = "/testmap/testlayer/secondentity";
    startServer();
}

void TestRepository::cleanup()
{
    stopServer();
}

void TestRepository::initTestCase()
{
    initTestdata();
}

void TestRepository::cleanupTestCase()
{
    cleanupTestdata();
}

void TestRepository::testCreateCheckout_data() { createTestdata(); }
void TestRepository::testCreateCheckout()
{
    QFETCH(upns::upnsSharedPointer<upns::Repository>, repo);
    upnsSharedPointer<Checkout> co(repo->createCheckout("master", "testcheckout_created_new"));
    QVERIFY(co != nullptr);
    OperationDescription operationCreateTree;
    operationCreateTree.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = filename_.c_str();
    QString entityPath(checkoutPath_.c_str());
    params["target"] = entityPath;
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operationCreateTree.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operationCreateTree);
    upnsSharedPointer<Tree> tr = co->getTree( entityPath.mid(0, entityPath.lastIndexOf('/')).toStdString() );
    QVERIFY(tr != nullptr);
    QVERIFY(tr->refs_size() != 0);
    if(tr && tr->refs_size() != 0)
    {
        upnsString childName(entityPath.mid( entityPath.lastIndexOf('/')+1 ).toStdString());
        bool childFound = false;
        for(google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator ch( tr->refs().cbegin() );
            ch != tr->refs().cend();
            ++ch)
        {
            childFound |= (ch->first == childName);
        }
        QVERIFY2(childFound, "Created entity was not child of parent tree");
    }
    upnsSharedPointer<Entity> ent = co->getEntity( entityPath.toStdString() );
    QVERIFY(ent != NULL);
    if(ent)
        QVERIFY(strcmp(ent->type(), PointcloudEntitydata::TYPENAME()) == 0);
}

void TestRepository::testGetCheckout_data() { createTestdata(); }
void TestRepository::testGetCheckout()
{
    QFETCH(upns::upnsSharedPointer<upns::Repository>, repo);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);

    OperationDescription operation;
    operation.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = filename_.c_str();
    params["target"] = checkoutPath_.c_str();
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    if(co)
    {
        co->doOperation(operation);
    }
}

void TestRepository::testReadCheckout_data() { createTestdata(); }
void TestRepository::testReadCheckout()
{
    QFETCH(upns::upnsSharedPointer<upns::Repository>, repo);
    QVERIFY2( repo->listCheckoutNames().size() > 0, "Can't find checkouts in repo");

    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);

    upns::Path path = checkoutPath_;
    upns::upnsSharedPointer<upns::Entity> entity = co->getEntity(path);

    QVERIFY(entity != nullptr);

    upns::upnsSharedPointer<upns::AbstractEntitydata> entityDataAbstract = co->getEntitydataReadOnly(path);
    QVERIFY(strcmp(entityDataAbstract->type(), PointcloudEntitydata::TYPENAME()) == 0);
    upns::upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(entityDataAbstract);

    // compare pointcloud from repo with pointcloud from filesystem
    upnsPointcloud2Ptr cloud_repo_2 = entityData->getData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_repo(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_repo_2, *cloud_repo);

    upnsPointcloud2Ptr cloud_fs_2( new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fs(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_fs_2, *cloud_fs);
    pcl::PCDReader reader;
    if ( reader.read(filename_, *cloud_fs) < 0 )
    {
        QVERIFY2(false, "Couldn't read file data/bunny.pcd");
    }

    QVERIFY2(cloud_repo->size() == cloud_fs->size(), "Size of clouds are not simmilar");
    QVERIFY2(cloud_repo->size() > 0, "Size of clouds is 0");

    for (int i = 0; i < cloud_repo->size(); i++) {
        pcl::PointXYZ p1 = cloud_repo->at(i);
        pcl::PointXYZ p2 = cloud_fs->at(i);
        QVERIFY2(p1.x == p2.x, (std::string("Points in cloud not simmilat x at ") + std::to_string(i)).c_str());
        QVERIFY2(p1.y == p2.y, (std::string("Points in cloud not simmilat y at ") + std::to_string(i)).c_str());
        QVERIFY2(p1.z == p2.z, (std::string("Points in cloud not simmilat z at ") + std::to_string(i)).c_str());
    }

}

void TestRepository::testCommit_data() { createTestdata(); }
void TestRepository::testCommit()
{
    return; // skip for now due to network test
    QFETCH(upns::upnsSharedPointer<upns::Repository>, repo);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);
    repo->commit( co, "This is the commit message of a TestCommit");
}

void TestRepository::testVoxelgridfilter_data() { createTestdata(); }
void TestRepository::testVoxelgridfilter()
{
    QFETCH(upns::upnsSharedPointer<upns::Repository>, repo);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);
    OperationDescription operation;
    operation.set_operatorname("voxelgridfilter");
    QJsonObject params;
    params["leafsize"] = 0.01;
    params["target"] = checkoutPath_.c_str();
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
    //Skip due to networking tests: repo->commit( co, "Two different pointclouds inside");
}

DECLARE_TEST(TestRepository)
