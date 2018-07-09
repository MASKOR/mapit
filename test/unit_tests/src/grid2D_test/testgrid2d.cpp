/*******************************************************************************
 *
 * Copyright      2015 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "testgrid2d.h"

#include "../../src/autotest.h"

#include <mapit/errorcodes.h>
#include <mapit/versioning/repositoryfactory.h>
#include <mapit/versioning/workspace.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>


Q_DECLARE_METATYPE(std::shared_ptr<mapit::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Workspace>)
Q_DECLARE_METATYPE(std::function<void()>)

void TestGrid2D::init()
{
    startServer();
}

void TestGrid2D::cleanup()
{
    stopServer();
}


void TestGrid2D::initTestCase()
{
    initTestdata();
}

void TestGrid2D::cleanupTestCase()
{
    cleanupTestdata();
}

void TestGrid2D::testGetData()
{

    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    std::string epath("/hello/test/gridentity");
    Grid2D grid;

    // test data
    float resolution = 100;
    float width = 2;
    float height = 3;
    // Position of origin, create quarernation & vector
    mapit::msgs::Quaternion origin_rot;
    origin_rot.set_w(1);
    origin_rot.set_x(2);
    origin_rot.set_y(3);
    origin_rot.set_z(4);

    mapit::msgs::Vector origin_trans;
    origin_trans.set_x(5);
    origin_trans.set_y(6);
    origin_trans.set_z(7);


    mapit::msgs::Pose origin;

    origin.set_allocated_rotation(&origin_rot);
    origin.set_allocated_translation(&origin_trans);
    // data as bytes but converted to string in c++, so how does the string look like?
    //bytes data = 5;

    grid.set_resolution(resolution);
    grid.set_width(width);
    grid.set_height(height);
    grid.set_allocated_origin(&origin);
    std::string data = "12345";
    grid.set_allocated_data(&data);

    std::cout << "Grid at creation, rot(w) is: " << grid.origin().rotation().w() << "\n";
//    origin.release_rotation();
//    origin.release_translation();
//    grid.release_origin();
//    grid.release_data();


    std::cout << "Grid at after release, rot(w) is: " << grid.origin().rotation().w() << "\n";


    // create operation
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    QVERIFY(repo     != nullptr);
    QVERIFY(workspace != nullptr);
    mapit::OperationResult res = workspace->doUntraceableOperation(desc
                                                                   , [&grid, &epath](mapit::OperationEnvironment* env)
    {
        mapit::operators::WorkspaceWritable *coraw = env->getWorkspace();
        std::shared_ptr<Entity> e(new Entity);
        e->set_type(Grid2DEntitydata::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<Grid2DEntitydata> entityData = std::dynamic_pointer_cast<Grid2DEntitydata>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }
        std::cout << "Grid input, rot(w) is: " << grid.origin().rotation().w() << "\n";

        //copy from scan, then setData
        std::shared_ptr<mapit::msgs::Grid2D> grid2d(new mapit::msgs::Grid2D);
        grid2d->CopyFrom(grid);
        std::cout << "Grid before copy, rot(w) is: " << grid2d->origin().rotation().w() << "\n";
        entityData->setData(grid2d);
        return MAPIT_STATUS_OK;
    });
    QVERIFY(mapitIsOk(res.first));


    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly(epath);
    std::shared_ptr<Grid2DEntitydata> entityData = std::dynamic_pointer_cast<Grid2DEntitydata>( abstractentitydataByPath );
    QVERIFY( entityData != nullptr );
    mapit::entitytypes::Grid2DType gridFromWS = entityData->getData(0);
    QVERIFY(gridFromWS->resolution() == resolution);
    QVERIFY(gridFromWS->width() == width);
    QVERIFY(gridFromWS->height() == height);
    mapit::msgs::Pose poseFromWS = gridFromWS->origin();
    //QVERIFY( poseFromWS != nullptr );
    mapit::msgs::Quaternion rotationFromWS = poseFromWS.rotation();
    mapit::msgs::Vector translationFromWS = poseFromWS.translation();
    //QVERIFY( rotationFromWS != nullptr );
   // QVERIFY( translationFromWS != nullptr );
    std::cout << "WS rot(w): " << rotationFromWS.x() << "\n";
    std::cout << "OV rot(w): " << origin_rot.w() << "\n";
    QVERIFY(rotationFromWS.w() == origin_rot.w());
    QVERIFY(rotationFromWS.x() == origin_rot.x());
    QVERIFY(rotationFromWS.y() == origin_rot.y());
    QVERIFY(rotationFromWS.z() == origin_rot.z());
    QVERIFY(translationFromWS.x() == origin_trans.x());
    QVERIFY(translationFromWS.y() == origin_trans.y());
    QVERIFY(translationFromWS.z() == origin_trans.z());

    // release objects for memory integrity, seems like protobuf does not take care of that
    // altough documentation says it should
    origin.release_rotation();
    origin.release_translation();
    grid.release_origin();
    grid.release_data();
}

void TestGrid2D::testSetData()
{

    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    std::string epath("/hello/test/gridentity");
    Grid2D grid;

    // test data
    float resolution = 100;
    float width = 2;
    float height = 3;
    // Position of origin, create quarernation & vector
    mapit::msgs::Quaternion origin_rot;
    origin_rot.set_w(1);
    origin_rot.set_x(2);
    origin_rot.set_y(3);
    origin_rot.set_z(4);

    mapit::msgs::Vector origin_trans;
    origin_trans.set_x(5);
    origin_trans.set_y(6);
    origin_trans.set_z(7);


    mapit::msgs::Pose origin;

    origin.set_allocated_rotation(&origin_rot);
    origin.set_allocated_translation(&origin_trans);
    // data as bytes but converted to string in c++, so how does the string look like?
    //bytes data = 5;

    grid.set_resolution(resolution);
    grid.set_width(width);
    grid.set_height(height);
    grid.set_allocated_origin(&origin);
    std::string data = "12345";
    grid.set_allocated_data(&data);

    // create operation
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("myUntraceable");
    desc.set_params("{\"source:\":\"testInlineOperator\"}");
    QVERIFY(repo     != nullptr);
    QVERIFY(workspace != nullptr);
    mapit::OperationResult res = workspace->doUntraceableOperation(desc
                                                                   , [&grid, &epath](mapit::OperationEnvironment* env)
    {
        mapit::operators::WorkspaceWritable *coraw = env->getWorkspace();
        std::shared_ptr<Entity> e(new Entity);
        e->set_type(Grid2DEntitydata::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<Grid2DEntitydata> entityData = std::dynamic_pointer_cast<Grid2DEntitydata>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }

        //copy from scan, then setData
        std::shared_ptr<mapit::msgs::Grid2D> grid2d(new mapit::msgs::Grid2D);
        grid2d->CopyFrom(grid);
        entityData->setData(grid2d);
        return MAPIT_STATUS_OK;
    });
    QVERIFY(mapitIsOk(res.first));

    // release objects, see testGetData
    origin.release_rotation();
    origin.release_translation();
    grid.release_origin();
    grid.release_data();
}

void TestGrid2D::testGetData_data() { createTestdata(false, false); }

void TestGrid2D::testSetData_data() { createTestdata(false, false); }

DECLARE_TEST(TestGrid2D)
