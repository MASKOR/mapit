#include "testrepository.h"
#include "upns_globals.h"
#include "services.pb.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"

using namespace upns;

void TestRepository::init()
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

    m_repo = new upns::Repository(conf);
    m_repo->init();
}

void TestRepository::cleanup()
{
}

void TestRepository::initTestCase()
{
}

void TestRepository::cleanupTestCase()
{
    delete m_repo;
}

void TestRepository::testCreateCheckout()
{
    upnsSharedPointer<Checkout> co(m_repo->createCheckout("master", "testcheckout"));

    OperationDescription operationCreateTree;
    operationCreateTree.set_operatorname("load_pointcloud");
    OperationParameter *filename = operationCreateTree.add_params();
    filename->set_key("filename");
    filename->set_strval("data/bunny.pcd");
    OperationParameter *target = operationCreateTree.add_params();
    target->set_key("target");
    target->set_strval("/testmap/testlayer/testentity");
    co->doOperation(operationCreateTree);
}

void TestRepository::testGetCheckout()
{
    upnsSharedPointer<Checkout> co(m_repo->getCheckout("master", "testcheckout"));

    OperationDescription operationCreateTree;
    operationCreateTree.set_operatorname("load_pointcloud");
    OperationParameter *filename = operationCreateTree.add_params();
    filename->set_key("filename");
    filename->set_strval("data/bunny.pcd");
    OperationParameter *target = operationCreateTree.add_params();
    target->set_key("target");
    target->set_strval("/testmap/testlayer/secondentity");
    co->doOperation(operationCreateTree);
}

void TestRepository::testCommit()
{
    upnsSharedPointer<Checkout> co(m_repo->getCheckout("master", "testcheckout"));
    m_repo->commit( co, "This is the commit message of a TestCommit");
}




DECLARE_TEST(TestRepository)
