#include "testrepository.h"
#include "upns_globals.h"
#include "services.pb.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"
#include <QJsonDocument>
#include <QJsonObject>

using namespace upns;

void TestRepository::init()
{
}

void TestRepository::cleanup()
{
}

void TestRepository::initTestCase()
{
    const char* databaseName = "test.db";
    QDir dir(databaseName);
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        QVERIFY( result );
    }
    const char* fileSystemName = ".mapit";
    dir = fileSystemName;
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

    m_repo = new upns::Repository(conf);
}

void TestRepository::cleanupTestCase()
{
    delete m_repo;
    m_repo = NULL;
}

void TestRepository::testCreateCheckout()
{
    upnsSharedPointer<Checkout> co(m_repo->createCheckout("master", "testcheckout"));

    OperationDescription operationCreateTree;
    operationCreateTree.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = "data/bunny.pcd";
    params["target"] = "/testmap/testlayer/testentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operationCreateTree.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operationCreateTree);
}

void TestRepository::testGetCheckout()
{
    upnsSharedPointer<Checkout> co(m_repo->getCheckout("testcheckout"));

    OperationDescription operation;
    operation.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = "data/bunny.pcd";
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
}

void TestRepository::testCommit()
{
    upnsSharedPointer<Checkout> co(m_repo->getCheckout("testcheckout"));
    m_repo->commit( co, "This is the commit message of a TestCommit");
}

void TestRepository::testVoxelgridfilter()
{
    upnsSharedPointer<Checkout> co(m_repo->getCheckout("testcheckout"));
    OperationDescription operation;
    operation.set_operatorname("voxelgridfilter");
    QJsonObject params;
    params["leafsize"] = 0.01;
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
    m_repo->commit( co, "Two different pointclouds inside");
}




DECLARE_TEST(TestRepository)
