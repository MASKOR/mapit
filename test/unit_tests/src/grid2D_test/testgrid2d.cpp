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
    QVERIFY(true);
}

void TestGrid2D::testGetData_data() { createTestdata(false, false); }

void TestGrid2D::testSetData_data() { createTestdata(false, false); }

DECLARE_TEST(TestGrid2D)
