/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "operation_description_history_test.h"
#include "../../src/autotest.h"

#include <mapit/errorcodes.h>
#include <mapit/versioning/workspace.h>
#include <mapit/versioning/repositoryfactory.h>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>

#include <mapit/layertypes/tflayer.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/time/time.h>
#include <mapit/logging.h>
#include <mapit/depthfirstsearch.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <typeinfo>
#include <iostream>

Q_DECLARE_METATYPE(std::shared_ptr<mapit::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Workspace>)
Q_DECLARE_METATYPE(std::function<void()>)

namespace fs = boost::filesystem;

void OpDescHistTest::init()
{
    startServer();
}

void OpDescHistTest::cleanup()
{
    stopServer();
}

void OpDescHistTest::initTestCase()
{
    initTestdata();
}

void OpDescHistTest::cleanupTestCase()
{
    cleanupTestdata();
}


void OpDescHistTest::test_description_after_operation_data() { createTestdata(true, true); }

void OpDescHistTest::test_description_after_operation()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    QVERIFY( workspace->getRollingcommit().ops_size() == 0 );

    OperationDescription desc_bunny;
    mapit::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = workspace->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    QVERIFY( workspace->getRollingcommit().ops_size() == 1 );
    QVERIFY( 0 == desc_bunny.params().compare( workspace->getRollingcommit().ops(0).params() ) );
    QVERIFY( 0 == desc_bunny.operator_().operatorname().compare( workspace->getRollingcommit().ops(0).operator_().operatorname() ) );

    mapit::CommitId coID = repo->commit(workspace, "CommitTest: first commit\n\nWith some text that is more describing of the whole situation", "the mapit system", "mapit@mascor.fh-aachen.de");

    QVERIFY( workspace->getRollingcommit().ops_size() == 0 );

    std::shared_ptr<mapit::msgs::Commit> co = repo->getCommit( coID );
    QVERIFY( co->ops_size() == 1 );
    QVERIFY( 0 == desc_bunny.params().compare( co->ops(0).params() ) );
    QVERIFY( 0 == desc_bunny.operator_().operatorname().compare( co->ops(0).operator_().operatorname() ) );
}

DECLARE_TEST(OpDescHistTest)
