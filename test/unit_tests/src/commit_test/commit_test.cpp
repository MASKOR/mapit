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

#include "commit_test.h"
#include "../../src/autotest.h"

#include <upns/errorcodes.h>
#include <upns/versioning/checkout.h>
#include <upns/versioning/repositoryfactory.h>

#include <mapit/msgs/datastructs.pb.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>

#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/time/time.h>
#include <upns/logging.h>
#include <upns/depthfirstsearch.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <typeinfo>
#include <iostream>

Q_DECLARE_METATYPE(std::shared_ptr<upns::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<upns::Checkout>)
Q_DECLARE_METATYPE(std::function<void()>)

namespace fs = boost::filesystem;

void CommitTest::init()
{
    startServer();
}

void CommitTest::cleanup()
{
    stopServer();
}

void CommitTest::initTestCase()
{
    initTestdata();
}

void CommitTest::cleanupTestCase()
{
    cleanupTestdata();
}


void CommitTest::test_commit_of_single_entity_data() { createTestdata(false, false); }

void CommitTest::test_commit_of_single_entity()
{
    QFETCH(std::shared_ptr<upns::Repository>, repo);
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    OperationDescription desc_bunny;
    upns::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );

    repo->commit(checkout, "CommitTest: first commit\n\nWith some text that is more describing of the whole situation", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    std::string rootTreeName   = "local.mapit/.mapit/trees/48aded3491401756a9894909c068326e88c4a82abfbb4889bcda27de5c9940c4";
    std::string entityName     = "local.mapit/.mapit/entities/0c30127d58aff7f988b894ec79f3fce87084c25e8296c759584f2eba6858cd14";
    std::string entitydataName = "local.mapit/.mapit/entities_data/9eba34864de5a749476dafb21ecf0acdbc28224aa79ed559beac48f5981fcaf1";

    QVERIFY( fs::exists(rootTreeName) );
    QVERIFY( fs::exists(entityName) );
    QVERIFY( fs::exists(entitydataName) );
}

void CommitTest::test_commit_of_trees_and_entities_data() { createTestdata(false, false); }

void CommitTest::test_commit_of_trees_and_entities()
{
    QFETCH(std::shared_ptr<upns::Repository>, repo);
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    OperationDescription desc_bunny;
    upns::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"/data/bunny_1\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"/data/bunny_2\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 3 "
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );

    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );

    repo->commit(checkout, "CommitTest: first commit\n\nWith some text that is more describing of the whole situation", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    fs::path entitydataName = "local.mapit/.mapit/entities_data/9eba34864de5a749476dafb21ecf0acdbc28224aa79ed559beac48f5981fcaf1";

    QVERIFY( fs::exists(entitydataName) );

    // check if the data is only stored once
    for( fs::directory_iterator file( entitydataName.parent_path() );
         file != fs::directory_iterator();
         ++file ) {
        QVERIFY( 0 == entitydataName.compare(*file) );
    }

    fs::path rootTreeName      = "local.mapit/.mapit/trees/7ad50df13b7c5c2577b6ce6dbd2342bb1f16b363694ebe96e9aca95987c0aab0";
    fs::path dataTreeName      = "local.mapit/.mapit/trees/afe677748a3dbe8c9ea58a2d8eb1678313f14802dfa23021d60d642be2ec3135";
    fs::path bunnyEntityName   = "local.mapit/.mapit/entities/0c30127d58aff7f988b894ec79f3fce87084c25e8296c759584f2eba6858cd14";
    fs::path bunny_1EntityName = bunnyEntityName;
    fs::path bunny_2EntityName = "local.mapit/.mapit/entities/885c3f4451b8ad917b0bab9f9b98414b31db56b56e9fff029efb73a612b19c9a";
    QVERIFY( fs::exists(rootTreeName) );
    QVERIFY( fs::exists(dataTreeName) );
    QVERIFY( fs::exists(bunnyEntityName) );
    QVERIFY( fs::exists(bunny_1EntityName) );
    QVERIFY( fs::exists(bunny_2EntityName) );
}

DECLARE_TEST(CommitTest)
