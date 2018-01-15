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

Q_DECLARE_METATYPE(std::shared_ptr<upns::Repository>)
Q_DECLARE_METATYPE(std::shared_ptr<upns::Checkout>)
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

void DeleteTest::add_bunny(std::shared_ptr<upns::Checkout> checkout, std::string path)
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
    ret = checkout->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );
}

void DeleteTest::test_delete_entity_data() { createTestdata(true, true); }

void DeleteTest::test_delete_entity()
{
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);
    //add bunny
    add_bunny(checkout, "bunny1");

    QVERIFY( checkout->getEntity( "bunny1" ) != nullptr);

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunny1\""
                "}"
                );
    upns::OperationResult ret_del = checkout->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout->getEntity( "bunny1" ) == nullptr);
}

void DeleteTest::test_delete_tree_data() { createTestdata(true, true); }

void DeleteTest::test_delete_tree()
{
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    //add bunnys
    add_bunny(checkout, "bunnys/bun1");
    add_bunny(checkout, "bunnys/bun2");
    add_bunny(checkout, "bunnys/bun3");
    add_bunny(checkout, "keep1/bun1");
    add_bunny(checkout, "keep2/bun1");
    add_bunny(checkout, "keep_bun1");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys\""
                "}"
                );
    upns::OperationResult ret_del = checkout->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout->getTree("bunnys" ) == nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun1" ) == nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun3" ) == nullptr);
    QVERIFY( checkout->getEntity( "keep1/bun1" ) != nullptr);
    QVERIFY( checkout->getEntity( "keep2/bun1" ) != nullptr);
    QVERIFY( checkout->getEntity( "keep_bun1" ) != nullptr);
}

void DeleteTest::test_delete_sub_entity_data() { createTestdata(true, true); }

void DeleteTest::test_delete_sub_entity()
{
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    //add bunnys
    add_bunny(checkout, "bunnys/bun1");
    add_bunny(checkout, "bunnys/bun2");
    add_bunny(checkout, "bunnys/bun3");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"bunnys/bun2\""
                "}"
                );
    upns::OperationResult ret_del = checkout->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout->getTree("bunnys" ) != nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun1" ) != nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun2" ) == nullptr);
    QVERIFY( checkout->getEntity( "bunnys/bun3" ) != nullptr);
}

void DeleteTest::test_delete_sub_tree_data() { createTestdata(true, true); }

void DeleteTest::test_delete_sub_tree()
{
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    //add bunnys
    add_bunny(checkout, "suuub/bunnys/bun1");
    add_bunny(checkout, "suuub/bunnys/bun2");
    add_bunny(checkout, "suuub/bunnys/bun3");
    add_bunny(checkout, "suuub/keep/bun");
    add_bunny(checkout, "suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : \"suuub/bunnys\""
                "}"
                );
    upns::OperationResult ret_del = checkout->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout->getTree("suuub" ) != nullptr);
    QVERIFY( checkout->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( checkout->getTree("suuub/keep" ) != nullptr);
    QVERIFY( checkout->getEntity( "suuub/keep_bun" ) != nullptr);
}

void DeleteTest::test_delete_entities_and_trees_mixed_data() { createTestdata(true, true); }

void DeleteTest::test_delete_entities_and_trees_mixed()
{
    QFETCH(std::shared_ptr<upns::Checkout>, checkout);

    //add bunnys
    add_bunny(checkout, "suuub/bunnys/del1");
    add_bunny(checkout, "suuub/bunnys/del2");
    add_bunny(checkout, "suuub/bunnys2/keep1");
    add_bunny(checkout, "suuub/bunnys2/del2");
    add_bunny(checkout, "suuub/keep/bun");
    add_bunny(checkout, "suuub/keep_bun");

    OperationDescription desc_del;
    desc_del.mutable_operator_()->set_operatorname("delete");
    desc_del.set_params(
                "{"
                "  \"target\"   : [ \"suuub/bunnys\", \"suuub/bunnys2/del2\" ]"
                "}"
                );
    upns::OperationResult ret_del = checkout->doOperation( desc_del );
    QVERIFY( upnsIsOk(ret_del.first) );

    QVERIFY( checkout->getTree("suuub" ) != nullptr);
    QVERIFY( checkout->getTree("suuub/keep" ) != nullptr);
    QVERIFY( checkout->getEntity( "suuub/keep_bun" ) != nullptr);
    QVERIFY( checkout->getTree("suuub/bunnys" ) == nullptr);
    QVERIFY( checkout->getTree("suuub/bunnys2" ) != nullptr);
    QVERIFY( checkout->getEntity( "suuub/bunnys2/keep1" ) != nullptr);
    QVERIFY( checkout->getEntity( "suuub/bunnys2/del2" ) == nullptr);
}

DECLARE_TEST(DeleteTest)
