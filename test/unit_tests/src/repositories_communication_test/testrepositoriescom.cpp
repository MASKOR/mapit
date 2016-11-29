#include "testrepositoriescom.h"
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
#include "../serverthread.h"
#include "modules/operationenvironment.h"
#include "upns_errorcodes.h"

#include "modules/versioning/checkoutraw.h"
#include "pointcloudlayer.h"

#include <pcl/io/pcd_io.h>

#include <unistd.h>
#include <sys/mman.h>

static bool *test_finished;
static bool *server_running;
static bool *quit_server;

void TestRepositoriesCommunication::initTestCase()
{
    test_finished = static_cast<bool*>(mmap(NULL, sizeof *test_finished, PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    server_running = static_cast<bool*>(mmap(NULL, sizeof *server_running, PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    quit_server = static_cast<bool*>(mmap(NULL, sizeof *quit_server, PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    *test_finished = 0;
    *server_running = 0;
    *quit_server = 0;

    // Create child process to be able to change the current directory to simulate a server.
    pid_t pid = fork();
    if (pid == 0)
    {
        // you are the server now
        chdir("./serverdirectory");
        while(true)
        {
            if(!*test_finished)
            {
                upns::upnsSharedPointer<upns::Repository> serverRepo(initEmptyLocal());
                upns::upnsSharedPointer<upns::RepositoryServer> server(upns::RepositoryNetworkingFactory::openRepositoryAsServer(5555, serverRepo.get()));
                while(!*test_finished)
                {
                    *server_running = true;
                    server->handleRequest(100);
                }
                *server_running = false;
            }
            if(*quit_server)
            {
                QEXPECT_FAIL("", "This is output from server process. Server should just stop now.", Abort);

                QVERIFY2(!*quit_server, "Stopping server");
            }
            QThread::currentThread()->msleep(5);
        }
    }
    else
    {
        //qDebug() << "Client running";
    }
}

void TestRepositoriesCommunication::cleanupTestCase()
{
    *test_finished = true;
    *quit_server = true;
    while(*server_running) QThread::currentThread()->msleep(5);

    munmap(test_finished, sizeof *test_finished);
    munmap(server_running, sizeof *server_running);
    munmap(quit_server, sizeof *quit_server);
}

void TestRepositoriesCommunication::init()
{
    *test_finished = false;
}

void TestRepositoriesCommunication::cleanup()
{
    *test_finished = true;
    while(*server_running) {
        QThread::currentThread()->msleep(5);
    }
}

void TestRepositoriesCommunication::testReadLocalComputeRemoteWriteLocal()
{
    upns::upnsSharedPointer<upns::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    upns::upnsSharedPointer<upns::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    upns::upnsSharedPointer<upns::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    QVERIFY( nullptr != checkoutLocal );
    upns::upnsSharedPointer<upns::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    QVERIFY( nullptr != checkoutRemote );
    readPcd(checkoutLocal.get());
    voxelgrid(checkoutRemote.get());
    writePcdLocalOnly(checkoutLocal.get(), "bunny_voxelgrid_locally_read_remote_computed.pcd");
}

void TestRepositoriesCommunication::testReadRemoteComputeLocalWriteRemote()
{
    upns::upnsSharedPointer<upns::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    upns::upnsSharedPointer<upns::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    upns::upnsSharedPointer<upns::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    upns::upnsSharedPointer<upns::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    readPcd(checkoutRemote.get());
    voxelgrid(checkoutLocal.get());
    // Note: This writes local, there is not yet a way to distinguish which thread wrote the file.
    writePcdLocalOnly(checkoutRemote.get(), "bunny_voxelgrid_remote_read_locally_computed.pcd");
}

void TestRepositoriesCommunication::testReadRemoteComputeRemoteWriteLocal()
{
    upns::upnsSharedPointer<upns::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    upns::upnsSharedPointer<upns::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    upns::upnsSharedPointer<upns::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    upns::upnsSharedPointer<upns::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    readPcd(checkoutRemote.get());
    voxelgrid(checkoutRemote.get());
    // Note: This writes local, there is not yet a way to distinguish which thread wrote the file.
    writePcdLocalOnly(checkoutLocal.get(), "bunny_voxelgrid_remote_read_remote_computed.pcd");
}

upns::upnsSharedPointer<upns::Repository> TestRepositoriesCommunication::initEmptyLocal()
{
    const char* fileSystemName = "testrepocom.mapit";
    YAML::Node config;
    YAML::Node mapsource;
    mapsource["name"] = "FileSystem";
    mapsource["filename"] = fileSystemName;
    config["mapsource"] = mapsource;
    QDir dir(fileSystemName);
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        if( false == result )
        {
            return upns::upnsSharedPointer<upns::Repository>(nullptr);
        }
    }
    return upns::upnsSharedPointer<upns::Repository>(upns::RepositoryFactory::openLocalRepository( config ));
}

upns::upnsSharedPointer<upns::Repository> TestRepositoriesCommunication::initNetwork(/*upns::upnsSharedPointer<upns::Repository> other,*/ bool computeLocal)
{
//    // get is okay here, server and localRepo have same lifecycle. Don't copy/paste this.
//    upns::upnsSharedPointer<upns::RepositoryServer> server(upns::RepositoryNetworkingFactory::openRepositoryAsServer(5555, other.get()));
//    ServerThread *serverThread(new ServerThread(server));
//    // networkRepo will be returned. server and serverThread should have the same lifecycle as networkRepo.
//    // The customDeleter should implement that and stops the server when the repo-object is deleted.
//    upns::upnsSharedPointer<upns::Repository> networkRepo(upns::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5555", nullptr, computeLocal),
//                                                          [serverThread](upns::Repository *data)
//    {
//        serverThread->stop();
//        delete serverThread;
//    });
//    serverThread->start();
//    return networkRepo;
    return upns::upnsSharedPointer<upns::Repository>(upns::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5555", nullptr, computeLocal));
}

void TestRepositoriesCommunication::readPcd(upns::Checkout *checkout)
{
    upns::OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"themap/thelayer/bunny\"}");
    upns::OperationResult ret = checkout->doOperation( desc );
    //QVERIFY( upnsIsOk(ret.first) );
}

void TestRepositoriesCommunication::voxelgrid(upns::Checkout *checkout)
{
    upns::OperationDescription operation;
    operation.set_operatorname("voxelgridfilter");
    QJsonObject params;
    params["leafsize"] = 0.01;
    params["target"] = "themap/thelayer/bunny";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    upns::OperationResult ret = checkout->doOperation(operation);
    //QVERIFY( upnsIsOk(ret.first) );
}

void TestRepositoriesCommunication::writePcdLocalOnly(upns::Checkout *checkout, std::string filename)
{
    OperationDescription desc;
    desc.set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    upns::OperationResult res = checkout->doUntraceableOperation(desc, [&filename](upns::OperationEnvironment* env)
    {
        upns::CheckoutRaw *coraw = env->getCheckout();
        upns::upnsSharedPointer<upns::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataReadOnly("themap/thelayer/bunny");
        if( abstractEntitydata == nullptr)
        {
            return UPNS_STATUS_ERROR;
        }
        upns::upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );

        pcl::PCDWriter writer;

        upnsPointcloud2Ptr pc2(entityData->getData());
        writer.writeASCII(filename, *pc2.get());

        return UPNS_STATUS_OK;
    });
}

DECLARE_TEST(TestRepositoriesCommunication)
