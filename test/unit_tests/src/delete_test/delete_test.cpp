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

void DeleteTest::test_delete_entity()
{
    //add bunny
    OperationDescription desc_bunny;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny1\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    upns::OperationResult ret = checkout_->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );

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

}

void DeleteTest::test_delete_sub_entity()
{

}

void DeleteTest::test_delete_sub_tree()
{

}

void DeleteTest::test_delete_entities_and_trees_mixed()
{

}

DECLARE_TEST(DeleteTest)
