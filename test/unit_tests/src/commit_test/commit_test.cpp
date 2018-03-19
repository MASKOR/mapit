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

#include <mapit/errorcodes.h>
#include <mapit/versioning/checkout.h>
#include <mapit/versioning/repositoryfactory.h>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/operators/versioning/checkoutraw.h>
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
Q_DECLARE_METATYPE(std::shared_ptr<mapit::Checkout>)
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
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Checkout>, checkout);

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
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    repo->commit(checkout, "CommitTest: first commit\n\nWith some text that is more describing of the whole situation", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    std::string rootTreeName   = "local.mapit/.mapit/trees/05031e86c8499a066ebd1d2e83f0b281c3b3c85d10970cdadcb673362ca56e62";
    std::string entityName     = "local.mapit/.mapit/entities/42cc531231c7d904b7b20640c506dd8b25099d4a509d39f88ad474d4b4fbc703";
    std::string entitydataName = "local.mapit/.mapit/entities_data/71e2f4bdb1932c22e3436341a30b2aa4de5e985a1c8d50b1f6f21c145b330944";

    QVERIFY( fs::exists(entitydataName) );
    QVERIFY( fs::exists(entityName) );
    QVERIFY( fs::exists(rootTreeName) );
}

void CommitTest::test_commit_of_trees_and_entities_data() { createTestdata(false, false); }

void CommitTest::test_commit_of_trees_and_entities()
{
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Checkout>, checkout);

    OperationDescription desc_bunny;
    mapit::OperationResult ret;
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
    QVERIFY( mapitIsOk(ret.first) );
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
    QVERIFY( mapitIsOk(ret.first) );

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
    QVERIFY( mapitIsOk(ret.first) );

    repo->commit(checkout, "CommitTest: first commit\n\nWith some text that is more describing of the whole situation", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    fs::path entitydataName = "local.mapit/.mapit/entities_data/71e2f4bdb1932c22e3436341a30b2aa4de5e985a1c8d50b1f6f21c145b330944";

    QVERIFY( fs::exists(entitydataName) );

    // check if the data is only stored once
    for( fs::directory_iterator file( entitydataName.parent_path() );
         file != fs::directory_iterator();
         ++file ) {
        QVERIFY( 0 == entitydataName.compare(*file) );
    }

    fs::path rootTreeName      = "local.mapit/.mapit/trees/11c0f0747fd1939ab0b22f8876a1bd465591706b74a2b51885957b0a267d3138";
    fs::path dataTreeName      = "local.mapit/.mapit/trees/42ad145a4182925afe09a90d1d5a1d803971584d7be40e443ca09e5ed8691af3";
    fs::path bunnyEntityName   = "local.mapit/.mapit/entities/42cc531231c7d904b7b20640c506dd8b25099d4a509d39f88ad474d4b4fbc703";
    fs::path bunny_1EntityName = bunnyEntityName;
    fs::path bunny_2EntityName = "local.mapit/.mapit/entities/1ca50ae5f33d0c67013cf877be6505a379231e2de6ea6f7522b784f81b582030";
    QVERIFY( fs::exists(bunny_1EntityName) );
    QVERIFY( fs::exists(bunny_2EntityName) );

    QVERIFY( fs::exists(bunnyEntityName) );
    QVERIFY( fs::exists(dataTreeName) );

    QVERIFY( fs::exists(rootTreeName) );
}

void CommitTest::test_delete_data() { createTestdata(false, false); }

void CommitTest::test_delete()
{
    // I want a clean repo for this testcase
    const char* repoName = "local-commit-delete-test.mapit";
    QDir repoFolder(repoName);
    if ( repoFolder.exists() ) {
        bool result = repoFolder.removeRecursively();
        QVERIFY( result );
    }
    std::shared_ptr<mapit::Repository> repo = std::shared_ptr<mapit::Repository>(mapit::RepositoryFactory::openLocalRepository(repoName));
    std::shared_ptr<mapit::Checkout> checkout = std::shared_ptr<mapit::Checkout>(repo->createCheckout("master", "test-delete-checkout"));;

    OperationDescription desc_bunny;
    mapit::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"/deltest/bunny\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    mapit::CommitId commit1ID = repo->commit(checkout, "CommitTest: Add bunny", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    std::string rootTreeName    = (std::string)repoName + "/.mapit/trees/a5fd3c9261d1732e5bc2e8ddaca187fd597fdea4c96827ef413eb786eea2370b";
    std::string deltestTreeName = (std::string)repoName + "/.mapit/trees/05031e86c8499a066ebd1d2e83f0b281c3b3c85d10970cdadcb673362ca56e62";
    std::string entityName      = (std::string)repoName + "/.mapit/entities/42cc531231c7d904b7b20640c506dd8b25099d4a509d39f88ad474d4b4fbc703";
    std::string entitydataName  = (std::string)repoName + "/.mapit/entities_data/71e2f4bdb1932c22e3436341a30b2aa4de5e985a1c8d50b1f6f21c145b330944";

    QVERIFY( fs::exists(entitydataName) );
    QVERIFY( fs::exists(entityName) );
    QVERIFY( fs::exists(deltestTreeName) );
    QVERIFY( fs::exists(rootTreeName) );

    // the data is now added, now we remove the data and check if it works
    desc_bunny.mutable_operator_()->set_operatorname("delete");
    desc_bunny.set_params(
                "{"
                "  \"target\"   : \"/deltest/bunny\""
                "}"
                );
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    std::string TransientEntityFolder    = (std::string)repoName + "/.mapit/checkouts/test-delete-checkout/root/deltest/bunny/";
    std::string TransientDeltestTreeName = (std::string)repoName + "/.mapit/checkouts/test-delete-checkout/root/deltest/.generic_entry";
    std::string TransientRootTreeName    = (std::string)repoName + "/.mapit/checkouts/test-delete-checkout/root/.generic_entry";

    QVERIFY( fs::is_empty(TransientEntityFolder) );
    QVERIFY( fs::exists(TransientDeltestTreeName) );
    QVERIFY( fs::exists(TransientRootTreeName) );
    // TODO one could load the protobufs and check the path to its childen

    mapit::CommitId commit2ID = repo->commit(checkout, "CommitTest: Remove bunny", "the mapit system", "mapit@mascor.fh-aachen.de");

    std::string NewRootTreeName    = (std::string)repoName + "/.mapit/trees/3bcee7d728fd2eabf4a149e612f90ab8289a4c16ca1670598df3a02db511dd01";
    std::string NewDeltestTreeName = (std::string)repoName + "/.mapit/trees/e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"; // this is the empty tree
    QVERIFY( fs::exists(NewDeltestTreeName) );
    QVERIFY( fs::exists(NewRootTreeName) );

    // check if checkout (which would be the 3. commit) only points to 2. commit
    std::vector<mapit::CommitId> wsParentIDs = checkout->getParentCommitIds();
    for (mapit::CommitId wsParentID : wsParentIDs) {
        QVERIFY( 0 == commit2ID.compare( wsParentID ) );
    }

    // check if 2. commit point to 1. commit
    std::shared_ptr<Commit> co2 = repo->getCommit(commit2ID);
    for (auto co2ParentID : co2->parentcommitids()) {
        QVERIFY( 0 == commit1ID.compare( co2ParentID ) );
    }
}

void CommitTest::test_branching_data() { createTestdata(false, false); }

void CommitTest::test_branching()
{
    // actual branches are not yet supported, here we test if we can create a branch (without a name)
    QFETCH(std::shared_ptr<mapit::Repository>, repo);
    QFETCH(std::shared_ptr<mapit::Checkout>, checkout);

    mapit::CommitId parentForNewWs;
    std::vector<mapit::CommitId> wsParents = checkout->getParentCommitIds();
    // get the second last commit (this is ugly, but fastly written ;))
    for (mapit::CommitId wsParent : wsParents) {
        std::shared_ptr<Commit> coP1 = repo->getCommit(wsParent);
        assert(coP1);
        for (mapit::CommitId coP1Parent : coP1->parentcommitids()) {
            std::shared_ptr<Commit> coP2 = repo->getCommit(coP1Parent);
            assert(coP2);
            parentForNewWs = coP1Parent;
        }
    }

    std::shared_ptr<mapit::Checkout> ws = repo->createCheckout(parentForNewWs, "wsBranched");

    OperationDescription desc_bunny;
    mapit::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\","
                "  \"sec\"      : 6, "
                "  \"nsec\"     : 5 "
                "}"
                );
    ret = ws->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    mapit::CommitId commitID = repo->commit(ws, "BranchTest: Test to commit a branched workspace\n\na few commits have been done in a different direction allready", "the mapit system", "mapit@mascor.fh-aachen.de");

    // this names depends on variables in RepositoryCommon::initTestdata() and bunny.pcd
    // TODO how to test if it worked?
//    mapit::depthFirstSearchHistory(repo
//                                   , commitID
//                                   , [](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return true;}
//                                   , [](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return true;});

}

DECLARE_TEST(CommitTest)
