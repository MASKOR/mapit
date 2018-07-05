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
    QVERIFY(true);
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



    QVERIFY(true);
}

void TestGrid2D::testGetData_data() { createTestdata(false, false); }

void TestGrid2D::testSetData_data() { createTestdata(false, false); }

DECLARE_TEST(TestGrid2D)
