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

#include "delete_test.h"
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

#include <typeinfo>
#include <iostream>

Q_DECLARE_METATYPE(std::shared_ptr<mapit::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Workspace>)
Q_DECLARE_METATYPE(std::function<void()>)

void DeleteTest::init()
{
    startServer();
}

void DeleteTest::cleanup()
{
    stopServer();
}

void DeleteTest::initTestCase()
{
    initTestdata();
}

void DeleteTest::cleanupTestCase()
{
    cleanupTestdata();
}

void DeleteTest::add_bunny(std::shared_ptr<mapit::Workspace> workspace, std::string path)
{
    OperationDescription desc_bunny;
    mapit::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"" + path + "\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = workspace->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );
}

void DeleteTest::test_delete_entity_data() { createTestdata(true, true); }

void DeleteTest::test_delete_entity()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);
    //add bunny
    add_bunny(workspace, "bunny1");

    QVERIFY( workspace->getEntity( "bunny1" ) != nullptr);

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunny1\""
                "}"
                );
    mapit::OperationResult ret_del = workspace->doOperation( desc_del );
    QVERIFY( mapitIsOk(ret_del.first) );

    QVERIFY( workspace->getEntity( "bunny1" ) == nullptr);
}

void DeleteTest::test_delete_tree_data() { createTestdata(true, true); }

void DeleteTest::test_delete_tree()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    //add bunnys
    add_bunny(workspace, "bunnys/bun1");
    add_bunny(workspace, "bunnys/bun2");
    add_bunny(workspace, "bunnys/bun3");
    add_bunny(workspace, "keep1/bun1");
    add_bunny(workspace, "keep2/bun1");
    add_bunny(workspace, "keep_bun1");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys\""
                "}"
                );
    mapit::OperationResult ret_del = workspace->doOperation( desc_del );
    QVERIFY( mapitIsOk(ret_del.first) );

    QVERIFY( workspace->getTree("bunnys" ) == nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun1" ) == nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun3" ) == nullptr);
    QVERIFY( workspace->getEntity( "keep1/bun1" ) != nullptr);
    QVERIFY( workspace->getEntity( "keep2/bun1" ) != nullptr);
    QVERIFY( workspace->getEntity( "keep_bun1" ) != nullptr);
}

void DeleteTest::test_delete_sub_entity_data() { createTestdata(true, true); }

void DeleteTest::test_delete_sub_entity()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    //add bunnys
    add_bunny(workspace, "bunnys/bun1");
    add_bunny(workspace, "bunnys/bun2");
    add_bunny(workspace, "bunnys/bun3");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys/bun2\""
                "}"
                );
    mapit::OperationResult ret_del = workspace->doOperation( desc_del );
    QVERIFY( mapitIsOk(ret_del.first) );

    QVERIFY( workspace->getTree("bunnys" ) != nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun1" ) != nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( workspace->getEntity( "bunnys/bun3" ) != nullptr);
}

void DeleteTest::test_delete_sub_tree_data() { createTestdata(true, true); }

void DeleteTest::test_delete_sub_tree()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    //add bunnys
    add_bunny(workspace, "suuub/bunnys/bun1");
    add_bunny(workspace, "suuub/bunnys/bun2");
    add_bunny(workspace, "suuub/bunnys/bun3");
    add_bunny(workspace, "suuub/keep/bun");
    add_bunny(workspace, "suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"suuub/bunnys\""
                "}"
                );
    mapit::OperationResult ret_del = workspace->doOperation( desc_del );
    QVERIFY( mapitIsOk(ret_del.first) );

    QVERIFY( workspace->getTree("suuub" ) != nullptr);
    QVERIFY( workspace->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( workspace->getTree("suuub/keep" ) != nullptr);
    QVERIFY( workspace->getEntity( "suuub/keep_bun" ) != nullptr);
}

void DeleteTest::test_delete_entities_and_trees_mixed_data() { createTestdata(true, true); }

void DeleteTest::test_delete_entities_and_trees_mixed()
{
    QFETCH(std::shared_ptr<mapit::Workspace>, workspace);

    //add bunnys
    add_bunny(workspace, "suuub/bunnys/del1");
    add_bunny(workspace, "suuub/bunnys/del2");
    add_bunny(workspace, "suuub/bunnys2/keep1");
    add_bunny(workspace, "suuub/bunnys2/del2");
    add_bunny(workspace, "suuub/keep/bun");
    add_bunny(workspace, "suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : [ \"suuub/bunnys\", \"suuub/bunnys2/del2\" ]"
                "}"
                );
    mapit::OperationResult ret_del = workspace->doOperation( desc_del );
    QVERIFY( mapitIsOk(ret_del.first) );

    QVERIFY( workspace->getTree("suuub" ) != nullptr);
    QVERIFY( workspace->getTree("suuub/keep" ) != nullptr);
    QVERIFY( workspace->getEntity( "suuub/keep_bun" ) != nullptr);
    QVERIFY( workspace->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( workspace->getTree("suuub/bunnys2" ) != nullptr);
    QVERIFY( workspace->getEntity( "suuub/bunnys2/keep1" ) != nullptr);
    QVERIFY( workspace->getEntity( "suuub/bunnys2/del2" ) == nullptr);
}

DECLARE_TEST(DeleteTest)
