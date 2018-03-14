/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "testrepositoriescom.h"
#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactory.h>
#include <mapit/versioning/repositorynetworkingfactory.h>
#include <functional>
#include "../serverthread.h"
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>

#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/layertypes/pointcloudlayer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

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
        // you are the server now (child process)
        chdir("./serverdirectory");
        while(true)
        {
            if(!*test_finished)
            {
                std::shared_ptr<mapit::Repository> serverRepo(initEmptyLocal());
                std::shared_ptr<mapit::RepositoryServer> server(mapit::RepositoryNetworkingFactory::openRepositoryAsServer(5555, serverRepo.get()));
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
        // this is the client (parent process). This runs all tests, while server will loop and skip tests.
        QFile out1(FILENAME_OUT1);
        QFile out2(FILENAME_OUT1);
        QFile out3(FILENAME_OUT1);
        if(out1.exists()) out1.remove();
        if(out2.exists()) out2.remove();
        if(out3.exists()) out3.remove();
        QVERIFY(!out1.exists());
        QVERIFY(!out2.exists());
        QVERIFY(!out3.exists());
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
    std::shared_ptr<mapit::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    std::shared_ptr<mapit::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    std::shared_ptr<mapit::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    QVERIFY( nullptr != checkoutLocal );
    std::shared_ptr<mapit::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    QVERIFY( nullptr != checkoutRemote );
    readPcd(checkoutLocal.get());
    voxelgrid(checkoutRemote.get());
    const char* filename = FILENAME_OUT1;
    writePcdLocalOnly(checkoutLocal.get(), filename);
    compareFileToExpected(filename);
}

void TestRepositoriesCommunication::testReadRemoteComputeLocalWriteRemote()
{
    std::shared_ptr<mapit::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    std::shared_ptr<mapit::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    std::shared_ptr<mapit::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    std::shared_ptr<mapit::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    readPcd(checkoutRemote.get());
    voxelgrid(checkoutLocal.get());
    const char* filename = FILENAME_OUT2;
    // Note: This writes local, there is not yet a way to distinguish which thread wrote the file.
    writePcdLocalOnly(checkoutRemote.get(), filename);
    compareFileToExpected(filename);
}

void TestRepositoriesCommunication::testReadRemoteComputeRemoteWriteLocal()
{
    std::shared_ptr<mapit::Repository> local(initNetwork(true));
    QVERIFY( nullptr != local );
    std::shared_ptr<mapit::Repository> networkRemoteCompute(initNetwork(false));
    QVERIFY( nullptr != networkRemoteCompute );

    std::shared_ptr<mapit::Checkout> checkoutLocal(local->createCheckout("master", "testcheckout"));
    std::shared_ptr<mapit::Checkout> checkoutRemote(networkRemoteCompute->getCheckout("testcheckout"));
    readPcd(checkoutRemote.get());
    voxelgrid(checkoutRemote.get());
    // Note: This writes local, there is not yet a way to distinguish which thread wrote the file.
    const char* filename = FILENAME_OUT3;
    writePcdLocalOnly(checkoutLocal.get(), filename);
    compareFileToExpected(filename);
}


void TestRepositoriesCommunication::compareFileToExpected(const char* filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr fileWritten(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fileSource(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *fileWritten) == -1)
    {
        QFAIL ((std::string("Couldn't read file)") + filename + "\n").c_str());
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (FILENAME, *fileSource) == -1)
    {
        QFAIL ((std::string("Couldn't read file)") + FILENAME + "\n").c_str());
    }
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr fileSource_const = fileSource;
    sor.setInputCloud(fileSource_const);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExpected(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter (*cloudExpected);
    QCOMPARE(fileWritten->width, cloudExpected->width);
    QCOMPARE(fileWritten->height, cloudExpected->height);
    for(int i=0 ; qMin(100, (int)fileWritten->width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = fileWritten->at(i);
        pcl::PointXYZ &p2 = cloudExpected->at(i);
        QCOMPARE_REALVEC3(p1, p2);
    }
}

std::shared_ptr<mapit::Repository> TestRepositoriesCommunication::initEmptyLocal()
{
    const char* fileSystemName = "testrepocom.mapit";
    QDir dir(fileSystemName);
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        if( false == result )
        {
            return std::shared_ptr<mapit::Repository>(nullptr);
        }
    }
    return std::shared_ptr<mapit::Repository>(mapit::RepositoryFactory::openLocalRepository( fileSystemName ));
}

std::shared_ptr<mapit::Repository> TestRepositoriesCommunication::initNetwork(/*std::shared_ptr<mapit::Repository> other,*/ bool computeLocal)
{
//    // get is okay here, server and localRepo have same lifecycle. Don't copy/paste this.
//    std::shared_ptr<mapit::RepositoryServer> server(mapit::RepositoryNetworkingFactory::openRepositoryAsServer(5555, other.get()));
//    ServerThread *serverThread(new ServerThread(server));
//    // networkRepo will be returned. server and serverThread should have the same lifecycle as networkRepo.
//    // The customDeleter should implement that and stops the server when the repo-object is deleted.
//    std::shared_ptr<mapit::Repository> networkRepo(mapit::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5555", nullptr, computeLocal),
//                                                          [serverThread](mapit::Repository *data)
//    {
//        serverThread->stop();
//        delete serverThread;
//    });
//    serverThread->start();
//    return networkRepo;
    return std::shared_ptr<mapit::Repository>(mapit::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5555", nullptr, computeLocal));
}

void TestRepositoriesCommunication::readPcd(mapit::Checkout *checkout)
{
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("load_pointcloud");
    desc.set_params(std::string("{\"filename\":\"") + FILENAME + "\", \"target\":\"themap/thelayer/bunny\"}");
    mapit::OperationResult ret = checkout->doOperation( desc );
    //QVERIFY( upnsIsOk(ret.first) );
}

void TestRepositoriesCommunication::voxelgrid(mapit::Checkout *checkout)
{
    OperationDescription operation;
    operation.mutable_operator_()->set_operatorname("voxelgridfilter");
    QJsonObject params;
    params["leafsize"] = LEAF_SIZE;
    params["target"] = "themap/thelayer/bunny";
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operation.set_params( paramsDoc.toJson().toStdString() );
    mapit::OperationResult ret = checkout->doOperation(operation);
    //QVERIFY( upnsIsOk(ret.first) );
}

void TestRepositoriesCommunication::writePcdLocalOnly(mapit::Checkout *checkout, std::string filename)
{
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    mapit::OperationResult res = checkout->doUntraceableOperation(desc, [&filename](mapit::OperationEnvironment* env)
    {
        mapit::CheckoutRaw *coraw = env->getCheckout();
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataReadOnly("themap/thelayer/bunny");
        if( abstractEntitydata == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }
        pcl::PCDWriter writer;

        upnsPointcloud2Ptr pc2(entityData->getData());
        writer.writeASCII(filename, *pc2.get());

        return MAPIT_STATUS_OK;
    });
}

DECLARE_TEST(TestRepositoriesCommunication)
