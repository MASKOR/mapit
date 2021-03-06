/*******************************************************************************
 *
 * Copyright      2015 Daniel Bulla	<d.bulla@fh-aachen.de>
- *          2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2018 Michael Norget   <mnorget@amt.rwth-aachen.de>
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

//void TestGrid2D::testGetData() {
//    QVERIFY(true);
//}

//void TestGrid2D::testSetData() {
//    QVERIFY(true);
//}

//void TestGrid2D::testGetData2() {
//    QVERIFY(true);
//}

void TestGrid2D::testGetData()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    std::string epath("/hello/test/gridentity");
    mapit::msgs::Grid2D grid;

    // test data
    float resolution = 100;
    unsigned int width = 2;
    unsigned int height = 3;
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
    grid.mutable_data()->assign(data);
    //grid.set_allocated_data(&data);

//    origin.release_rotation();
//    origin.release_translation();
//    grid.release_origin();
//    grid.release_data();

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
        e->set_type(mapit::entitytypes::Grid2D::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<mapit::entitytypes::Grid2D> entityData = std::dynamic_pointer_cast<mapit::entitytypes::Grid2D>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }

        //copy from scan, then setData
        std::shared_ptr<mapit::msgs::Grid2D> grid2d = std::make_shared<mapit::msgs::Grid2D>();
        grid2d->CopyFrom(grid);
        std::shared_ptr<mapit::entitytypes::Grid2DHelper> help = std::make_shared<mapit::entitytypes::Grid2DHelper>();
        help->setGrid(*grid2d);
        entityData->setData(help);
        return MAPIT_STATUS_OK;
    });
    QVERIFY(mapitIsOk(res.first));

    float epsilon = 0.000001f;

    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly(epath);
    std::shared_ptr<mapit::entitytypes::Grid2D> entityData = std::dynamic_pointer_cast<mapit::entitytypes::Grid2D>( abstractentitydataByPath );
    QVERIFY( entityData != nullptr );
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> helpOut = entityData->getData(0);
    mapit::msgs::Grid2D gridFromWS = helpOut->getGrid();
    QVERIFY(std::abs(gridFromWS.resolution() - resolution) < epsilon);
    QVERIFY(std::abs(static_cast<long>(gridFromWS.width()) - width)< epsilon);
    QVERIFY(std::abs(static_cast<long>(gridFromWS.height()) - height) < epsilon);
    mapit::msgs::Pose poseFromWS = gridFromWS.origin();
    //QVERIFY( poseFromWS != nullptr );
    mapit::msgs::Quaternion rotationFromWS = poseFromWS.rotation();
    mapit::msgs::Vector translationFromWS = poseFromWS.translation();
    //QVERIFY( rotationFromWS != nullptr );
   // QVERIFY( translationFromWS != nullptr );
    QVERIFY(std::abs(rotationFromWS.w() - origin_rot.w()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.x() - origin_rot.x()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.y() - origin_rot.y()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.z() - origin_rot.z()) < epsilon);
    QVERIFY(std::abs(translationFromWS.x() - origin_trans.x()) < epsilon);
    QVERIFY(std::abs(translationFromWS.y() - origin_trans.y()) < epsilon);
    QVERIFY(std::abs(translationFromWS.z() - origin_trans.z()) < epsilon);

    // release objects for memory integrity, seems like protobuf does not take care of that
    // altough documentation says it should
    origin.release_rotation();
    origin.release_translation();
    grid.release_origin();
    grid.release_data();
}

void TestGrid2D::testGetData2()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    std::string epath("/hello/test/gridentity");
    Grid2D grid;

    // test data
    float resolution = 100;
    unsigned int width = 2;
    unsigned int height = 3;
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
    signed char ch1 = -1;
    signed char ch2 = 100;
    signed char ch3 = 15;
    std::string data;
    data += ch1;
    data += ch2;
    data += ch3;
    //std::string data = "12345";
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
        e->set_type(mapit::entitytypes::Grid2D::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<mapit::entitytypes::Grid2D> entityData = std::dynamic_pointer_cast<mapit::entitytypes::Grid2D>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }

        //copy from scan, then setData
        std::shared_ptr<mapit::msgs::Grid2D> grid2d(new mapit::msgs::Grid2D);
        grid2d->CopyFrom(grid);
        std::shared_ptr<mapit::entitytypes::Grid2DHelper> help = std::make_shared<mapit::entitytypes::Grid2DHelper>();
        help->setGrid(*grid2d);
        entityData->setData(help);
        return MAPIT_STATUS_OK;
    });
    QVERIFY(mapitIsOk(res.first));

    float epsilon = 0.000001f;

    std::shared_ptr<mapit::AbstractEntitydata> abstractentitydataByPath = workspace->getEntitydataReadOnly(epath);
    std::shared_ptr<mapit::entitytypes::Grid2D> entityData = std::dynamic_pointer_cast<mapit::entitytypes::Grid2D>( abstractentitydataByPath );
    QVERIFY( entityData != nullptr );
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> helpOut = entityData->getData(0);
    mapit::msgs::Grid2D gridFromWS = helpOut->getGrid();
    QVERIFY(std::abs(gridFromWS.resolution() - resolution) < epsilon);
    QVERIFY(std::abs(static_cast<long>(gridFromWS.width()) - width) < epsilon);
    QVERIFY(std::abs(static_cast<long>(gridFromWS.height()) - height) < epsilon);
    mapit::msgs::Pose poseFromWS = gridFromWS.origin();
    //QVERIFY( poseFromWS != nullptr );
    mapit::msgs::Quaternion rotationFromWS = poseFromWS.rotation();
    mapit::msgs::Vector translationFromWS = poseFromWS.translation();
    //QVERIFY( rotationFromWS != nullptr );
   // QVERIFY( translationFromWS != nullptr );
    QVERIFY(std::abs(rotationFromWS.w() - origin_rot.w()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.x() - origin_rot.x()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.y() - origin_rot.y()) < epsilon);
    QVERIFY(std::abs(rotationFromWS.z() - origin_rot.z()) < epsilon);
    QVERIFY(std::abs(translationFromWS.x() - origin_trans.x()) < epsilon);
    QVERIFY(std::abs(translationFromWS.y() - origin_trans.y()) < epsilon);
    QVERIFY(std::abs(translationFromWS.z() - origin_trans.z()) < epsilon);

    std::string dataFromWS = gridFromWS.data();


    size_t n = dataFromWS.length();
    char char_array[n+1];
    strcpy(char_array, dataFromWS.c_str());

    QVERIFY(static_cast<int>(char_array[0]) == static_cast<int>(ch1));
    QVERIFY(static_cast<int>(char_array[1]) == static_cast<int>(ch2));
    QVERIFY(static_cast<int>(char_array[2]) == static_cast<int>(ch3));

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
        e->set_type(mapit::entitytypes::Grid2D::TYPENAME());
        mapit::StatusCode status = coraw->storeEntity(epath, e);
        if(!mapitIsOk(status))
        {
            return MAPIT_STATUS_ERROR;
        }
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = coraw->getEntitydataForReadWrite( epath );
        std::shared_ptr<mapit::entitytypes::Grid2D> entityData = std::dynamic_pointer_cast<mapit::entitytypes::Grid2D>( abstractEntitydata );
        if(entityData == nullptr)
        {
            return MAPIT_STATUS_ERROR;
        }

        //copy from scan, then setData
        std::shared_ptr<mapit::msgs::Grid2D> grid2d(new mapit::msgs::Grid2D);
        grid2d->CopyFrom(grid);
        std::shared_ptr<mapit::entitytypes::Grid2DHelper> help = std::make_shared<mapit::entitytypes::Grid2DHelper>();
        help->setGrid(*grid2d);
        entityData->setData(help);
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
void TestGrid2D::testGetData2_data() { createTestdata(false, false); }

void TestGrid2D::testSetData_data() { createTestdata(false, false); }

void TestGrid2D::testSetDataHelper()
{
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(0);
    origin.mutable_translation()->set_y(0);
    int value_grey = mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN;
    int value_occu = mapit::entitytypes::Grid2DHelper::GRID_OCCUPIED;
    int value_free = mapit::entitytypes::Grid2DHelper::GRID_FREE;
    grid->initGrid(100.0f, 100.0f, static_cast<float>(1.0), origin);
    // set some vars
    grid->setProbability(2.0f, 4.0f, value_grey);
    grid->setProbability(2.0f, 5.0f, value_free);
    grid->setProbability(2.0f, 6.0f, value_occu);

    grid->setProbability(12.0f, 14.0f, value_free);
    grid->setProbability(12.0f, 15.0f, value_free);
    grid->setProbability(12.0f, 16.0f, value_free);

    grid->setProbability(22.0f, 24.0f, value_occu);
    grid->setProbability(22.0f, 25.0f, value_free);
    grid->setProbability(22.0f, 26.0f, value_occu);

    mapit::msgs::Grid2D g = grid->getGrid();
    std::string data = g.data();

    QVERIFY(grid->getProbability(2.0f, 4.0f) == value_grey);
    QVERIFY(grid->getProbability(2.0f, 5.0f) == value_free);
    QVERIFY(grid->getProbability(2.0f, 6.0f) == value_occu);

    QVERIFY(grid->getProbability(12.0f, 14.0f) == value_free);
    QVERIFY(grid->getProbability(12.0f, 15.0f) == value_free);
    QVERIFY(grid->getProbability(12.0f, 16.0f) == value_free);

    QVERIFY(grid->getProbability(22.0f, 24.0f) == value_occu);
    QVERIFY(grid->getProbability(22.0f, 25.0f) == value_free);
    QVERIFY(grid->getProbability(22.0f, 26.0f) == value_occu);
}

void TestGrid2D::testGetDataHelper()
{
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(-50);
    origin.mutable_translation()->set_y(-50);
    grid->initGrid(100.0f, 100.0f, static_cast<float>(1.0), origin);

    mapit::msgs::Grid2D g = grid->getGrid();
    std::string data = g.data();

    QVERIFY(grid->getGrid().width() == 100);
    QVERIFY(grid->getGrid().height() == 100);

    for (float x = 0.0f; x < 100.0f; x++) {
        for (float y = 0.0f; y < 100.0f; y++) {
            //std::cout << "Get value at x=" << x << " and y=" << y << " \n";
            QVERIFY(grid->getProbability(x, y) == mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN);
        }
    }
}

void TestGrid2D::testGridHelper()
{
    float size_x = 4.0f, size_y = 10.0f;
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(roundf(-size_x/2));
    origin.mutable_translation()->set_y(roundf(-size_y/2));
    grid->initGrid(size_x, size_y, static_cast<float>(1.0), origin);
    QVERIFY(grid->getGrid().width() == static_cast<unsigned int>(size_x));
    QVERIFY(grid->getGrid().height() == static_cast<unsigned int>(size_y));

    int value_grey = mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN;
    int value_occu = mapit::entitytypes::Grid2DHelper::GRID_OCCUPIED;
    int value_free = mapit::entitytypes::Grid2DHelper::GRID_FREE;

    grid->setProbability(2.0f, 2.0f, value_occu);

    grid->setProbability(2.0f, 4.0f, value_grey);
    grid->setProbability(2.0f, 5.0f, value_free);
    grid->setProbability(2.0f, 6.0f, value_occu);

    mapit::msgs::Grid2D g = grid->getGrid();
    std::string data = g.data();

    // Print grid via grid and via helper, no need to put that in an automated test
//    std::cout << "Testing grid, " << size_x << "x" << size_y << ", manipulation: \n";
//    for (float y = 0.0f; y < size_y; y++) {
//        for (float x = 0.0f; x < size_x; x++) {
//            std::cout << grid->getProbability(x, y) << " ";
//        }
//        std::cout << "\n";
//    }

//    std::cout << "Testing grid direct, " << size_x << "x" << size_y << ", manipulation: \n";
//    for (int xy = 0; xy < data.length(); xy++) {
//        std::cout << static_cast<int>(data.at(xy)) << " ";
//        if (xy % static_cast<int>(size_x) == static_cast<int>(size_x - 1)) {
//            std::cout << "\n";
//        }
//    }

    // integrity check, getting pos via helper must be same pos as direct from grid
    int xyGridPos = 0;
    for (float y = 0.0f; y < size_y; y++) {
        for (float x = 0.0f; x < size_x; x++) {
            QVERIFY(grid->getProbability(x, y) == static_cast<int>(data.at(xyGridPos++)));
        }
    }
}

void TestGrid2D::testGridNegative()
{
    float size_x = 4.0f, size_y = 4.0f;
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(0);
    origin.mutable_translation()->set_y(0);
    grid->initGrid(size_x, size_y, static_cast<float>(1.0), origin);

    int retVal = mapit::entitytypes::Grid2DHelper::GRID_FREE;
    grid->setProbability(-1.0f, -1.0f, retVal);

    // -1 -1 -1 -1
    // -1 +0 -1 -1
    // -1 -1 -1 -1
    // -1 -1 -1 -1

    QVERIFY(grid->getProbability(-1.0f, -1.0f) == retVal);
    QVERIFY(grid->getGrid().data().at(5) == retVal);
}

void TestGrid2D::testRangeCheckX()
{
    float size_x = 4.0f, size_y = 4.0f;
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(roundf(-size_x/2));
    origin.mutable_translation()->set_y(roundf(-size_y/2));
    grid->initGrid(size_x, size_y, static_cast<float>(1.0), origin);

    int retVal = mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN;

    try {
        grid->setProbability(5.0f, 0.0f, retVal);
        QVERIFY(false);
    } catch (const std::out_of_range &ex) {
        QVERIFY(true);
    }
}

void TestGrid2D::testRangeCheckY()
{
    float size_x = 4.0f, size_y = 4.0f;
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(roundf(-size_x/2));
    origin.mutable_translation()->set_y(roundf(-size_y/2));
    grid->initGrid(size_x, size_y, static_cast<float>(1.0), origin);

    int retVal = mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN;

    try {
        grid->setProbability(0.0f, 5.0f, retVal);
        QVERIFY(false);
    } catch (const std::out_of_range &ex) {
        QVERIFY(true);
    }
}

void TestGrid2D::testGridResolution1()
{
    float size_x = 4.0f, size_y = 8.0f, resolution = 0.1f;
    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(0);
    origin.mutable_translation()->set_y(0);
    grid->initGrid(size_x, size_y, static_cast<float>(resolution), origin);

    const float epsilon = 0.000001f;

    QVERIFY(std::abs(grid->getGrid().height() - size_y / resolution) < epsilon);
    QVERIFY(std::abs(grid->getGrid().width() - size_x / resolution) < epsilon);
    QVERIFY(grid->getGrid().data().length() == grid->getGrid().height() * grid->getGrid().width());
}

void TestGrid2D::testSetDataHelper_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testGetDataHelper_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testGridHelper_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testRangeCheckX_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testRangeCheckY_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testGridNegative_data()
{
    createTestdata(false, false);
}

void TestGrid2D::testGridResolution1_data()
{
    createTestdata(false, false);
}

DECLARE_TEST(TestGrid2D)
