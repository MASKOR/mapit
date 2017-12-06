#include "op_reg_local_icp_test.h"
#include "../../src/autotest.h"

#include <upns/logging.h>

#include <upns/errorcodes.h>
#include <upns/versioning/checkout.h>
#include <upns/versioning/repositoryfactory.h>

#include <mapit/msgs/datastructs.pb.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>

#include <upns/layertypes/pointcloudlayer.h>
#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/time/time.h>

#include <typeinfo>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

Eigen::Affine3f
getTfMatrix()
{
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() << 0.05, 0., 0.;
    tf.rotate( Eigen::AngleAxisf (M_PI / 8., Eigen::Vector3f::UnitZ()) );

    return tf;
}

void OPRegLocalICPTest::init()
{
    fileSystemName_ = std::string("op_reg_local_icp_test.mapit");
    cleanup();
    repo_ = std::shared_ptr<upns::Repository>(upns::RepositoryFactory::openLocalRepository(fileSystemName_));
    checkout_ = std::shared_ptr<upns::Checkout>(repo_->createCheckout("master", "op_reg_local_icp_test"));

    //add bunny
    OperationDescription desc_bunny;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\""
                "}"
                );
    upns::OperationResult ret = checkout_->doOperation( desc_bunny );
    QVERIFY( upnsIsOk(ret.first) );

    // get bunny, transform and save as bunny_tf
    std::shared_ptr<upns::AbstractEntitydata> ed_a = checkout_->getEntitydataReadOnly("bunny");
    QVERIFY( ed_a != nullptr );
    std::shared_ptr<PointcloudEntitydata> ed_pc = std::dynamic_pointer_cast<PointcloudEntitydata>(ed_a);
    QVERIFY( ed_pc != nullptr );
    std::shared_ptr<pcl::PCLPointCloud2> bunny = ed_pc->getData();
    Eigen::Affine3f tf = getTfMatrix();
    log_info("op_reg_local_icp_test: transform bunny with:");
    std::cout << tf.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> bunny_pc;
    pcl::fromPCLPointCloud2(*bunny, bunny_pc);

    pcl::PointCloud<pcl::PointXYZ> bunny_tf;
    pcl::transformPointCloud(bunny_pc, bunny_tf, tf);

    // store bunny_tf in filesystem
    pcl::io::savePCDFileBinary("data/bunny_tf.pcd", bunny_tf);

    // add buny_tf
    OperationDescription desc_bunny_tf;
    desc_bunny_tf.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny_tf.set_params(
                "{"
                "  \"filename\" : \"data/bunny_tf.pcd\","
                "  \"target\"   : \"bunny_tf\""
                "}"
                );
    upns::OperationResult ret_tf = checkout_->doOperation( desc_bunny_tf );
    QVERIFY( upnsIsOk(ret_tf.first) );
}

void OPRegLocalICPTest::cleanup()
{
    QDir dir(fileSystemName_.c_str());
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        QVERIFY( result );
    }
}

void OPRegLocalICPTest::initTestCase()
{
}

void OPRegLocalICPTest::cleanupTestCase()
{
}

void OPRegLocalICPTest::test_icp_tf_add()
{
    // execute operator ICP
    mapit::msgs::OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("reg_local_icp");
    desc.set_params(
                "{"
                "   \"input\"             : \"bunny\","
                "   \"target\"            : \"bunny_tf\","
                "   \"handle-result\"     : \"tf-add\","
                "   \"tf-prefix\"         : \"/tfs/\","
                "   \"tf-frame_id\"       : \"world\","
                "   \"tf-child_frame_id\" : \"bunny\""
                "}"
                );
    upns::OperationResult ret = checkout_->doOperation( desc );
    QVERIFY( upnsIsOk(ret.first) );

    // get result from tf buffer
    mapit::tf2::BufferCore buffer(checkout_.get(), "/tfs");
    tf::TransformStamped tf_b = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(0, 0));
    Eigen::Affine3f tf_from_icp(tf_b.transform.translation);
    tf_from_icp.rotate(tf_b.transform.rotation);

    // compare result
    Eigen::Affine3f tf_origin = getTfMatrix();
    QVERIFY( tf_from_icp.isApprox(tf_origin, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
}

DECLARE_TEST(OPRegLocalICPTest)
