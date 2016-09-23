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
#include "versioning/repository.h"
#include "versioning/repositoryfactory.h"
#include "versioning/repositorynetworkingfactory.h"
#include <functional>

Q_DECLARE_METATYPE(upns::Repository*)
Q_DECLARE_METATYPE(std::function<void()>)

using namespace upns;

void TestRepository::init()
{
}

void TestRepository::cleanup()
{
}

void TestRepository::initTestCase()
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
        m_repo[2] = upns::RepositoryNetworkingFactory::connectToRemoteRepository("localhost:1234");
    }
}

void TestRepository::cleanupTestCase()
{
    delete m_repo[0];
    m_repo[0] = NULL;
    delete m_repo[1];
    m_repo[1] = NULL;
    delete m_repo[2];
    m_repo[2] = NULL;
}

void TestRepository::testCreateCheckout_data() { createTestdata(); }
void TestRepository::testCreateCheckout()
{
    QFETCH(upns::Repository*, repo);
    QFETCH(std::function<void()>, serverHandleRequest);
    upnsSharedPointer<Checkout> co(repo->createCheckout("master", "testcheckout"));
    serverHandleRequest();

    OperationDescription operationCreateTree;
    operationCreateTree.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = "data/bunny.pcd";
    params["target"] = "/testmap/testlayer/testentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operationCreateTree.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operationCreateTree);
    serverHandleRequest();
}

void TestRepository::testGetCheckout_data() { createTestdata(); }
void TestRepository::testGetCheckout()
{
    QFETCH(upns::Repository*, repo);
    QFETCH(std::function<void()>, serverHandleRequest);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    serverHandleRequest();

    OperationDescription operation;
    operation.set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = "data/bunny.pcd";
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
    serverHandleRequest();
}

void TestRepository::testCommit_data() { createTestdata(); }
void TestRepository::testCommit()
{
    QFETCH(upns::Repository*, repo);
    QFETCH(std::function<void()>, serverHandleRequest);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    serverHandleRequest();
    repo->commit( co, "This is the commit message of a TestCommit");
    serverHandleRequest();
}

void TestRepository::testVoxelgridfilter_data() { createTestdata(); }
void TestRepository::testVoxelgridfilter()
{
    QFETCH(upns::Repository*, repo);
    QFETCH(std::function<void()>, serverHandleRequest);
    upnsSharedPointer<Checkout> co(repo->getCheckout("testcheckout"));
    serverHandleRequest();
    OperationDescription operation;
    operation.set_operatorname("voxelgridfilter");
    QJsonObject params;
    params["leafsize"] = 0.01;
    params["target"] = "/testmap/testlayer/secondentity";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operation);
    serverHandleRequest();
    repo->commit( co, "Two different pointclouds inside");
    serverHandleRequest();
}

void TestRepository::createTestdata()
{
    QTest::addColumn<upns::Repository*>("repo");
    QTest::addColumn<std::function<void()> >("serverHandleRequest");

    QTest::newRow("local filesystem") << m_repo[0] << std::function<void()>([](){});
    QTest::newRow("local database")   << m_repo[1] << std::function<void()>([](){});
    QTest::newRow("remote")           << m_repo[2] << std::function<void()>([this](){m_srv->handleRequest();});
}




DECLARE_TEST(TestRepository)
