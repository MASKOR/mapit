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
    std::string rootTreeName   = "local.mapit/.mapit/trees/4a48864ad186f6eb57586b2904cbdccd21c836061d2086114161a3407d2f7d92";
    std::string entityName     = "local.mapit/.mapit/entities/6ae742b98fd0867c98e2238596fe7ee40153af0d22b368845b354882436971ae";
    std::string entitydataName = "local.mapit/.mapit/entities_data/3a8d491c40779b2325a78ad3b385dd1d3f12650dc68da2f42a8603ff67ed1c87";

    QVERIFY( fs::exists(entitydataName) );
    QVERIFY( fs::exists(entityName) );
    QVERIFY( fs::exists(rootTreeName) );
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
    fs::path entitydataName = "local.mapit/.mapit/entities_data/3a8d491c40779b2325a78ad3b385dd1d3f12650dc68da2f42a8603ff67ed1c87";

    QVERIFY( fs::exists(entitydataName) );

    // check if the data is only stored once
    for( fs::directory_iterator file( entitydataName.parent_path() );
         file != fs::directory_iterator();
         ++file ) {
        QVERIFY( 0 == entitydataName.compare(*file) );
    }

    fs::path rootTreeName      = "local.mapit/.mapit/trees/7a3afbfa9f6de7e3c5c1cb2d1ab6e32e5bf04581873c7ca93091241f896f3edc";
    fs::path dataTreeName      = "local.mapit/.mapit/trees/6f46956354353622d3e5257fa3ea675ee5f0878f7af8e4b86de914b383ed3caf";
    fs::path bunnyEntityName   = "local.mapit/.mapit/entities/6ae742b98fd0867c98e2238596fe7ee40153af0d22b368845b354882436971ae";
    fs::path bunny_1EntityName = bunnyEntityName;
    fs::path bunny_2EntityName = "local.mapit/.mapit/entities/0aa53db5a1f74bf77661a6f169f07cc771717ba534c93044a6c84d08a6e5a355";
    QVERIFY( fs::exists(bunny_1EntityName) );
    QVERIFY( fs::exists(bunny_2EntityName) );

    QVERIFY( fs::exists(bunnyEntityName) );
    QVERIFY( fs::exists(dataTreeName) );

    QVERIFY( fs::exists(rootTreeName) );
}

DECLARE_TEST(CommitTest)
