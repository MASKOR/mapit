#include "delete_test.h"
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

#include <typeinfo>
#include <iostream>

void DeleteTest::init()
{
    fileSystemName_ = std::string("delete_test.mapit");
    cleanup();
    repo_ = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(fileSystemName_));
    checkout_ = std::shared_ptr<upns::Checkout>(repo_->createCheckout("master", "del_test"));
}

void DeleteTest::cleanup()
{
    QDir dir(fileSystemName_.c_str());
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        QVERIFY( result );
    }
}

void DeleteTest::initTestCase()
{
}

void DeleteTest::cleanupTestCase()
{
}

void DeleteTest::add_bunny(std::string path)
{
    OperationDescription desc_bunny;
    upns::OperationResult ret;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"" + path + "\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = checkout_->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );
}

void DeleteTest::test_delete_entity()
{
    //add bunny
    add_bunny("bunny1");

    QVERIFY( checkout_->getEntity( "bunny1" ) != nullptr);

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunny1\""
                "}"
                );
    upns::OperationResult ret_del = checkout_->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout_->getEntity( "bunny1" ) == nullptr);
}

void DeleteTest::test_delete_tree()
{
    //add bunnys
    add_bunny("bunnys/bun1");
    add_bunny("bunnys/bun2");
    add_bunny("bunnys/bun3");
    add_bunny("keep1/bun1");
    add_bunny("keep2/bun1");
    add_bunny("keep_bun1");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys\""
                "}"
                );
    upns::OperationResult ret_del = checkout_->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout_->getTree("bunnys" ) == nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun1" ) == nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun3" ) == nullptr);
    QVERIFY( checkout_->getEntity( "keep1/bun1" ) != nullptr);
    QVERIFY( checkout_->getEntity( "keep2/bun1" ) != nullptr);
    QVERIFY( checkout_->getEntity( "keep_bun1" ) != nullptr);
}

void DeleteTest::test_delete_sub_entity()
{
    //add bunnys
    add_bunny("bunnys/bun1");
    add_bunny("bunnys/bun2");
    add_bunny("bunnys/bun3");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys/bun2\""
                "}"
                );
    upns::OperationResult ret_del = checkout_->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout_->getTree("bunnys" ) != nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun1" ) != nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( checkout_->getEntity( "bunnys/bun3" ) != nullptr);
}

void DeleteTest::test_delete_sub_tree()
{
    //add bunnys
    add_bunny("suuub/bunnys/bun1");
    add_bunny("suuub/bunnys/bun2");
    add_bunny("suuub/bunnys/bun3");
    add_bunny("suuub/keep/bun");
    add_bunny("suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"suuub/bunnys\""
                "}"
                );
    upns::OperationResult ret_del = checkout_->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout_->getTree("suuub" ) != nullptr);
    QVERIFY( checkout_->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( checkout_->getTree("suuub/keep" ) != nullptr);
    QVERIFY( checkout_->getEntity( "suuub/keep_bun" ) != nullptr);
}

void DeleteTest::test_delete_entities_and_trees_mixed()
{
    //add bunnys
    add_bunny("suuub/bunnys/del1");
    add_bunny("suuub/bunnys/del2");
    add_bunny("suuub/bunnys2/keep1");
    add_bunny("suuub/bunnys2/del2");
    add_bunny("suuub/keep/bun");
    add_bunny("suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : [ \"suuub/bunnys\", \"suuub/bunnys2/del2\" ]"
                "}"
                );
    upns::OperationResult ret_del = checkout_->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout_->getTree("suuub" ) != nullptr);
    QVERIFY( checkout_->getTree("suuub/keep" ) != nullptr);
    QVERIFY( checkout_->getEntity( "suuub/keep_bun" ) != nullptr);
    QVERIFY( checkout_->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( checkout_->getTree("suuub/bunnys2" ) != nullptr);
    QVERIFY( checkout_->getEntity( "suuub/bunnys2/keep1" ) != nullptr);
    QVERIFY( checkout_->getEntity( "suuub/bunnys2/del2" ) == nullptr);
}

DECLARE_TEST(DeleteTest)
