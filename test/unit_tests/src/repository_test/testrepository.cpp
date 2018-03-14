/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "testrepository.h"
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
#include <mapit/layertypes/pointcloudlayer.h>
#include <functional>
#include <pcl/io/pcd_io.h>
#include <iostream>

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

using namespace mapit::msgs;
using namespace mapit;

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
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    std::shared_ptr<Checkout> co(repo->createCheckout("master", "testcheckout_created_new"));
    QVERIFY(co != nullptr);
    OperationDescription operationCreateTree;
    operationCreateTree.mutable_operator_()->set_operatorname("load_pointcloud");
    QJsonObject params;
    params["filename"] = filename_.c_str();
    QString entityPath(checkoutPath_.c_str());
    params["target"] = entityPath;
    QJsonDocument paramsDoc;
    paramsDoc.setObject( params );
    operationCreateTree.set_params( paramsDoc.toJson().toStdString() );
    co->doOperation(operationCreateTree);
    std::shared_ptr<Tree> tr = co->getTree( entityPath.mid(0, entityPath.lastIndexOf('/')).toStdString() );
    QVERIFY(tr != nullptr);
    QVERIFY(tr->refs_size() != 0);
    if(tr && tr->refs_size() != 0)
    {
        std::string childName(entityPath.mid( entityPath.lastIndexOf('/')+1 ).toStdString());
        bool childFound = false;
        for(google::protobuf::Map< ::std::string, ::ObjectReference >::const_iterator ch( tr->refs().cbegin() );
            ch != tr->refs().cend();
            ++ch)
        {
            childFound |= (ch->first == childName);
        }
        QVERIFY2(childFound, "Created entity was not child of parent tree");
    }
    std::shared_ptr<Entity> ent = co->getEntity( entityPath.toStdString() );
    QVERIFY(ent != NULL);
    if(ent)
        QVERIFY(ent->type().compare(PointcloudEntitydata::TYPENAME()) == 0);
}

void TestRepository::testGetCheckout_data() { createTestdata(); }
void TestRepository::testGetCheckout()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    std::shared_ptr<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);

    OperationDescription operation;
    operation.mutable_operator_()->set_operatorname("load_pointcloud");
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
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QVERIFY2( repo->listCheckoutNames().size() > 0, "Can't find checkouts in repo");

    std::shared_ptr<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);

    mapit::Path path = checkoutPath_;
    std::shared_ptr<Entity> entity = co->getEntity(path);

    QVERIFY(entity != nullptr);

    std::shared_ptr<mapit::AbstractEntitydata> entityDataAbstract = co->getEntitydataReadOnly(path);
    QVERIFY(strcmp(entityDataAbstract->type(), PointcloudEntitydata::TYPENAME()) == 0);
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>(entityDataAbstract);
    QVERIFY(entityData != nullptr);

    // compare pointcloud from repo with pointcloud from filesystem
    mapit::entitytypes::Pointcloud2Ptr cloud_repo_2 = entityData->getData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_repo(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_repo_2, *cloud_repo);

    mapit::entitytypes::Pointcloud2Ptr cloud_fs_2( new pcl::PCLPointCloud2);
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
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    std::shared_ptr<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);
    repo->commit( co, "This is the commit message of a TestCommit");
}

void TestRepository::testVoxelgridfilter_data() { createTestdata(); }
void TestRepository::testVoxelgridfilter()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    std::shared_ptr<Checkout> co(repo->getCheckout("testcheckout"));
    QVERIFY(co != nullptr);
    OperationDescription operation;
    operation.mutable_operator_()->set_operatorname("voxelgridfilter");
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
