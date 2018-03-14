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
    std::string rootTreeName   = "local.mapit/.mapit/trees/ae24c810976196461762b56addaba892514464ab406bf258374148885ce3dc26";
    std::string entityName     = "local.mapit/.mapit/entities/0c30127d58aff7f988b894ec79f3fce87084c25e8296c759584f2eba6858cd14";
    std::string entitydataName = "local.mapit/.mapit/entities_data/9eba34864de5a749476dafb21ecf0acdbc28224aa79ed559beac48f5981fcaf1";

    QVERIFY( fs::exists(rootTreeName) );
    QVERIFY( fs::exists(entityName) );
    QVERIFY( fs::exists(entitydataName) );
}

DECLARE_TEST(CommitTest)
