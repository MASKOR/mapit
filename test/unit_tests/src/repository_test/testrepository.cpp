#include "testrepository.h"
#include "upns_typedefs.h"
#include "services.pb.h"
#include "../../src/autotest.h"
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

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Repository>)
Q_DECLARE_METATYPE(upns::upnsSharedPointer<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

using namespace upns;

void TestRepository::init()
{
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
    params["filename"] = "data/bunny.pcd";
    QString entityPath("/testmap/testlayer/testentity");
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
        QVERIFY(ent->type() == upns::POINTCLOUD);
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
    params["filename"] = "data/bunny.pcd";
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    if(co)
    {
        co->doOperation(operation);
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
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
    //Skip due to networking tests: repo->commit( co, "Two different pointclouds inside");
}

DECLARE_TEST(TestRepository)
