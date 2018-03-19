/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
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

#include "testoperators.h"
#include <mapit/typedefs.h>
#include <mapit/logging.h>
#include <mapit/errorcodes.h>
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include <mapit/layertypes/pointcloudlayer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <mapit/msgs/datastructs.pb.h>

#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>

#include <mapit/layertypes/pointcloudlayer.h>

using namespace mapit;

//TODO: Why must this be redeclared here ( @sa repositorycommon.cpp)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Workspace>)
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
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("load_pointcloud");
    //desc.set_params("{\"filename\":\"data/1465223257087387.pcd\", \"target\":\"corridor/laser/eins\"}");
    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"corridor/laser/eins\"}");
    OperationResult ret = workspace->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );
    std::shared_ptr<Entity> ent = workspace->getEntity("corridor/laser/eins");
    QVERIFY( ent != nullptr );
    QVERIFY( ent->type().compare(PointcloudEntitydata::TYPENAME()) == 0 );
    std::shared_ptr<Tree> parent = workspace->getTree("corridor/laser");
    QVERIFY( parent != nullptr );
    QVERIFY( !(*parent->mutable_refs())["eins"].path().empty() );
    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly("corridor/laser/eins");
    //std::shared_ptr<AbstractEntitydata> abstractentitydataByRef = workspace->getEntitydataReadOnly( (*parent->mutable_refs())["eins"].id() );
    std::shared_ptr<PointcloudEntitydata> entitydataPC2ByPath = std::dynamic_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
    QVERIFY( entitydataPC2ByPath != nullptr );
    //std::shared_ptr<PointcloudEntitydata> entitydataPC2ByRef = std::d_pointer_cast<PointcloudEntitydata>(abstractentitydataByRef);
    mapit::entitytypes::Pointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
    //mapit::entitytypes::Pointcloud2Ptr pc2ref  = entitydataPC2ByRef->getData(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(*pc2ref, *cloudRef);

    pcl::PointCloud<pcl::PointXYZ>::Ptr file (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *file) == -1) //* load the file
    {
        QFAIL ("Couldn't read file data/bunny.pcd \n");
    }
    QCOMPARE(cloudPath->width, file->width);
    QCOMPARE(cloudPath->height, file->height);
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
    std::string epath("/hello/test/entity");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.f,  2.f,  3.f));
    cloud.push_back(pcl::PointXYZ(4.f,  5.f,  6.f));
    cloud.push_back(pcl::PointXYZ(7.f,  8.f,  9.f));
    cloud.push_back(pcl::PointXYZ(0.f,  0.f,  0.f));
    cloud.push_back(pcl::PointXYZ(7.5f, 8.5f, 9.5f));

    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    mapit::OperationResult res = workspace->doUntraceableOperation(desc, [&cloud, &epath](mapit::OperationEnvironment* env)
    {
        mapit::operators::WorkspaceWritable *coraw = env->getWorkspace();
        std::shared_ptr<Entity> e(new Entity);
        e->set_type(PointcloudEntitydata::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<pcl::PCLPointCloud2> cloud2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *cloud2);
        entityData->setData(cloud2);
        return MAPIT_STATUS_OK;
    });
    QVERIFY(mapitIsOk(res.first));

    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly(epath);
    std::shared_ptr<PointcloudEntitydata> entitydataPC2ByPath = std::dynamic_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
    QVERIFY( entitydataPC2ByPath != nullptr );
    mapit::entitytypes::Pointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
    for(int i=0 ; qMin(100, (int)cloud.width) > i ; ++i)
    {
        pcl::PointXYZ &p1 = cloudPath->at(i);
        pcl::PointXYZ &p3 = cloud.at(i);
        QCOMPARE_REALVEC3(p1, p3);
    }
}

void TestOperators::testPointcloudToMesh_data() { createTestdata(); }
void TestOperators::testPointcloudToMesh()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("load_pointcloud");
    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"bunny/laser/eins\"}");
    OperationResult ret = workspace->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );

#if MAPIT_ENABLE_OPENVDB
    desc.mutable_operator_()->set_operatorname("surfrecon_openvdb");
    desc.set_params("{\"voxelsize\":0.1, \"radius\":1, \"input\":\"bunny/laser/eins\", \"output\":\"bunny/laser/levelset\"}");
    ret = workspace->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );

    desc.mutable_operator_()->set_operatorname("levelset_to_mesh");
    desc.set_params("{\"input\":\"bunny/laser/levelset\", \"output\":\"bunny/laser/asset\"}");
    ret = workspace->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );
#endif
    log_info("Test finished!");
}

//void TestOperators::testOperatorGrid_data() { createTestdata(); }
//void TestOperators::testOperatorGrid()
//{
//    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

//    OperationDescription desc;
//    desc.set_operatorname("load_pointcloud");
//    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"corridor/laser/eins\"}");
//    OperationResult ret = workspace->doOperation( desc );
//    QVERIFY( mapitIsOk(ret.first) );
//    desc.set_operatorname("grid");
//    desc.set_params("{\"target\":\"corridor/laser/eins\", \"leafsize\":\"0.01\"}");
//    ret = workspace->doOperation( desc );
//    QVERIFY( mapitIsOk(ret.first) );
//    std::shared_ptr<Entity> ent = workspace->getEntity("corridor/laser/eins");
//    QVERIFY( ent != nullptr );
//    QVERIFY( ent->type() == mapit::POINTCLOUD2 );
//    std::shared_ptr<Tree> parent = workspace->getTree("corridor/laser/eins");
//    QVERIFY( parent != nullptr );
//    QVERIFY( !(*parent->mutable_refs())["eins"].id().empty() );
//    std::shared_ptr<AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly("corridor/laser/eins");
//    //std::shared_ptr<AbstractEntitydata> abstractentitydataByRef = workspace->getEntitydataReadOnly( (*parent->mutable_refs())["eins"].id() );

//    std::shared_ptr<PointcloudEntitydata> entitydataPC2ByPath = std::d_pointer_cast<PointcloudEntitydata>(abstractentitydataByPath);
//    //std::shared_ptr<PointcloudEntitydata> entitydataPC2ByRef = std::d_pointer_cast<PointcloudEntitydata>(abstractentitydataByRef);
//    mapit::entitytypes::Pointcloud2Ptr pc2path = entitydataPC2ByPath->getData(0);
//    //mapit::entitytypes::Pointcloud2Ptr pc2ref  = entitydataPC2ByRef->getData(0);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPath(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(*pc2path, *cloudPath);
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
//    //pcl::fromPCLPointCloud2(*pc2ref, *cloudRef);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr file (new pcl::PointCloud<pcl::PointXYZ>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *file) == -1) //* load the file
//    {
//        QFAIL ("Couldn't read file data/bunny.pcd \n");
//    }

//    for(int i=0 ; qMin(100, (int)file->width) > i ; ++i)
//    {
//        pcl::PointXYZ &p1 = cloudPath->at(i);
//        //pcl::PointXYZ &p2 = cloudRef->at(i);
//        pcl::PointXYZ &p3 = file->at(i);
//        //QCOMPARE_REALVEC3(p1, p2);
//        QCOMPARE_REALVEC3(p1, p3);
//    }
//}

DECLARE_TEST(TestOperators)
