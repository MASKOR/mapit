/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "repositorycommon.h"
#include <upns/repositoryserver.h>
#include <upns/versioning/repositoryfactory.h>
#include <upns/versioning/repositorynetworkingfactory.h>
#include <upns/versioning/checkout.h>
#include <QThread>
#include "serverthread.h"

Q_DECLARE_METATYPE(std::shared_ptr<upns::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

void RepositoryCommon::createTestdata(bool withServer, bool withServerLocalyCalculated)
{
    const bool testLevelDB = false;
    const bool testRemote = withServer;
    QTest::addColumn< std::shared_ptr<upns::Repository> >("repo");
    QTest::addColumn< std::shared_ptr<upns::Checkout> >("checkout");
    QTest::addColumn< std::function<void()> >("startServer");
    QTest::addColumn< std::function<void()> >("stopServer");

    QTest::newRow("local filesystem")  << m_repo[0] << m_checkout[0] << std::function<void()>([](){}) << std::function<void()>([](){});
    if(testLevelDB)
    {
        QTest::newRow("local database")<< m_repo[1] << m_checkout[1] << std::function<void()>([](){}) << std::function<void()>([](){});
    }
    if(testRemote)
    {
        QTest::newRow("remote")        << m_repo[2] << m_checkout[2] << std::function<void()>([this]()
            {
                QMutexLocker l(&m_serverThreadMutex);
                m_serverThread[0]->start();
            })
            << std::function<void()>([this]()
            {
                QMutexLocker l(&m_serverThreadMutex);
                m_serverThread[0]->stop();
            });
    }
    if (withServerLocalyCalculated)
    {
        QTest::newRow("remote with local execution")        << m_repo[3] << m_checkout[3] << std::function<void()>([this]()
            {
                QMutexLocker l(&m_serverThreadMutex);
                m_serverThread[1]->start();
            })
            << std::function<void()>([this]()
            {
                QMutexLocker l(&m_serverThreadMutex);
                m_serverThread[1]->stop();
            });
    }
}

void RepositoryCommon::initTestdata()
{
    {
        //// Setup Repository in Filesystem
        const char* fileSystemName = "local.mapit";
        QDir dir(fileSystemName);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        m_repo[0] = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(fileSystemName));
        m_checkout[0] = std::shared_ptr<upns::Checkout>(m_repo[0]->createCheckout("master", "testcheckout"));
    }
//    {
//        //// Setup Repository as Database
//        const char* databaseName = "test.db";
//        QDir dir(databaseName);
//        if(dir.exists())
//        {
//            bool result = dir.removeRecursively();
//            QVERIFY( result );
//        }
//        YAML::Node conf;
//        YAML::Node mapsource;
//        mapsource["name"] = "leveldb";
//        mapsource["filename"] = databaseName;
//        conf["mapsource"] = mapsource;

//        m_repo[1] = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(conf));
//        m_checkout[1] = std::shared_ptr<upns::Checkout>(m_repo[1]->createCheckout("master", "testcheckout"));
//    }
    {
        //// Setup Repository as Network connection
        const char* fileSystemName2 = "remote.mapit";
        QDir dir(fileSystemName2);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        m_networkRepo[0] = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(fileSystemName2));

        // get is okay here, m_srv and m_repo[1] have same lifecycle. Don't copy/paste this.
        std::shared_ptr<upns::RepositoryServer> srv = std::shared_ptr<upns::RepositoryServer>(upns::RepositoryNetworkingFactory::openRepositoryAsServer(5555, m_networkRepo[0].get()));
        m_repo[2] = std::shared_ptr<upns::Repository>(upns::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5555", NULL));
        m_serverThread[0] = std::shared_ptr<ServerThread>(new ServerThread(srv));
        m_serverThread[0]->start();
        m_checkout[2] = std::shared_ptr<upns::Checkout>(m_repo[2]->createCheckout("master", "testcheckout"));
        m_serverThread[0]->stop();
    }
    {
        //// Setup Repository as Network connection
        const char* fileSystemName2 = "remote-with-local-execution.mapit";
        QDir dir(fileSystemName2);
        if(dir.exists())
        {
            bool result = dir.removeRecursively();
            QVERIFY( result );
        }
        m_networkRepo[1] = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(fileSystemName2));

        // get is okay here, m_srv and m_repo[1] have same lifecycle. Don't copy/paste this.
        std::shared_ptr<upns::RepositoryServer> srv = std::shared_ptr<upns::RepositoryServer>(upns::RepositoryNetworkingFactory::openRepositoryAsServer(5655, m_networkRepo[1].get()));
        m_repo[3] = std::shared_ptr<upns::Repository>(upns::RepositoryNetworkingFactory::connectToRemoteRepository("tcp://localhost:5655", NULL, true));
        m_serverThread[1] = std::shared_ptr<ServerThread>(new ServerThread(srv));
        m_serverThread[1]->start();
        m_checkout[3] = std::shared_ptr<upns::Checkout>(m_repo[3]->createCheckout("master", "testcheckout"));
        m_serverThread[1]->stop();
    }
}

void RepositoryCommon::cleanupTestdata()
{
    m_checkout[0] = nullptr;
    m_checkout[1] = nullptr;
    m_checkout[2] = nullptr;
    m_checkout[3] = nullptr;
    m_repo[0] = nullptr;
    m_repo[1] = nullptr;
    m_repo[2] = nullptr;
    m_repo[3] = nullptr;
    m_networkRepo[0] = nullptr;
    m_networkRepo[1] = nullptr;
    m_serverThread[0]->wait(600);
    m_serverThread[1]->wait(600);
    m_serverThread[0] = nullptr;
    m_serverThread[1] = nullptr;
}

void RepositoryCommon::startServer()
{
    QFETCH(std::function<void()>, startServer);
    startServer();
}

void RepositoryCommon::stopServer()
{
    QFETCH(std::function<void()>, stopServer);
    stopServer();
}
