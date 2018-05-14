/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "op_reg_local_test.h"
#include "../../src/autotest.h"

#include <mapit/logging.h>

#include <mapit/errorcodes.h>
#include <mapit/versioning/workspace.h>
#include <mapit/versioning/repositoryfactory.h>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>

#include <mapit/layertypes/pointcloudlayer.h>
#include <mapit/layertypes/tflayer.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
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
    fileSystemName_ = std::string("op_reg_local_test.mapit");
    cleanup();
    repo_ = std::shared_ptr<mapit::Repository>(mapit::RepositoryFactory::openLocalRepository(fileSystemName_));
    workspace_ = std::shared_ptr<mapit::Workspace>(repo_->createWorkspace("master", "op_reg_local_icp_test"));

    log_warn("\n\nop_reg_local_icp_test: this test is based on the performance of ICP on 1 example pointcloud\n"
                 "                       When this failes, it might just mean that the ICP does not work as\n"
                 "                       good as before for only the example pointcloud while it might would\n"
                 "                       work better overall"
             " \n\n");

    //add bunny
    OperationDescription desc_bunny;
    desc_bunny.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    mapit::OperationResult ret = workspace_->doOperation( desc_bunny );
    QVERIFY( mapitIsOk(ret.first) );

    //add 2. bunny
    OperationDescription desc_bunny2;
    desc_bunny2.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny2.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny2\","
                "  \"sec\"      : 2, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = workspace_->doOperation( desc_bunny2 );
    QVERIFY( mapitIsOk(ret.first) );

    //add 3. bunny
    OperationDescription desc_bunny3;
    desc_bunny3.mutable_operator_()->set_operatorname("load_pointcloud");
    desc_bunny3.set_params(
                "{"
                "  \"filename\" : \"data/bunny.pcd\","
                "  \"target\"   : \"bunny3\","
                "  \"sec\"      : 3, "
                "  \"nsec\"     : 0 "
                "}"
                );
    ret = workspace_->doOperation( desc_bunny3 );
    QVERIFY( mapitIsOk(ret.first) );

    // get bunny, transform and save as bunny_tf
    std::shared_ptr<mapit::AbstractEntitydata> ed_a = workspace_->getEntitydataReadOnly("bunny");
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
                "  \"target\"   : \"bunny_tf\","
                "  \"sec\"      : 1, "
                "  \"nsec\"     : 0 "
                "}"
                );
    mapit::OperationResult ret_tf = workspace_->doOperation( desc_bunny_tf );
    QVERIFY( mapitIsOk(ret_tf.first) );

    // add tfs for tf-combine test
    OperationDescription desc_add_tfs;
    desc_add_tfs.mutable_operator_()->set_operatorname("load_tfs");
    desc_add_tfs.set_params(
                "{"
                "   \"prefix\" : \"tf-combine\","
                "   \"transforms\" : "
                "   ["
                "       {"
                "           \"static\" : false,"
                "           \"header\" : {"
                "               \"frame_id\" : \"world\","
                "               \"stamp\" : {"
                "                   \"sec\" : 0,"
                "                   \"nsec\" : 0"
                "               }"
                "           },"
                "           \"transform\" : {"
                "               \"child_frame_id\" : \"bunny\","
                "               \"translation\" : {"
                "                   \"x\" : 1.0,"
                "                   \"y\" : 0.0,"
                "                   \"z\" : 0.5"
                "               },"
                "               \"rotation\" : {"
                "                   \"w\" : 1.0,"
                "                   \"x\" : 0.0,"
                "                   \"y\" : 0.0,"
                "                   \"z\" : 0.0"
                "               }"
                "           }"
                "       },"
                "       {"
                "           \"static\" : false,"
                "           \"header\" : {"
                "               \"frame_id\" : \"world\","
                "               \"stamp\" : {"
                "                   \"sec\" : 1,"
                "                   \"nsec\" : 0"
                "               }"
                "           },"
                "           \"transform\" : {"
                "               \"child_frame_id\" : \"bunny\","
                "               \"translation\" : {"
                "                   \"x\" : 1.0,"
                "                   \"y\" : 1.0,"
                "                   \"z\" : 0.5"
                "               },"
                "               \"rotation\" : {"
                "                   \"w\" : 1.0,"
                "                   \"x\" : 0.0,"
                "                   \"y\" : 0.0,"
                "                   \"z\" : 0.0"
                "               }"
                "           }"
                "       },"
                "       {"
                "           \"static\" : false,"
                "           \"header\" : {"
                "               \"frame_id\" : \"world\","
                "               \"stamp\" : {"
                "                   \"sec\" : 2,"
                "                   \"nsec\" : 0"
                "               }"
                "           },"
                "           \"transform\" : {"
                "               \"child_frame_id\" : \"bunny\","
                "               \"translation\" : {"
                "                   \"x\" : 1.0,"
                "                   \"y\" : 1.0,"
                "                   \"z\" : 1.0"
                "               },"
                "               \"rotation\" : {"
                "                   \"w\" : 1.0,"
                "                   \"x\" : 0.0,"
                "                   \"y\" : 0.0,"
                "                   \"z\" : 0.0"
                "               }"
                "           }"
                "       },"
                "       {"
                "           \"static\" : false,"
                "           \"header\" : {"
                "               \"frame_id\" : \"world\","
                "               \"stamp\" : {"
                "                   \"sec\" : 3,"
                "                   \"nsec\" : 0"
                "               }"
                "           },"
                "           \"transform\" : {"
                "               \"child_frame_id\" : \"bunny\","
                "               \"translation\" : {"
                "                   \"x\" : 2.0,"
                "                   \"y\" : 1.0,"
                "                   \"z\" : 1.0"
                "               },"
                "               \"rotation\" : {"
                "                   \"w\" : 1.0,"
                "                   \"x\" : 0.0,"
                "                   \"y\" : 0.0,"
                "                   \"z\" : 0.0"
                "               }"
                "           }"
                "       }"
                "   ]"
                "}"
                );
    mapit::OperationResult ret_tfs = workspace_->doOperation( desc_add_tfs );
    QVERIFY( mapitIsOk(ret_tfs.first) );
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
                "   \"tf-child_frame_id\" : \"bunny\","
                "   \"tf-is_static\"      : true,"
                "   \"matching-algorithm\" : \"icp\""
                "}"
                );
    mapit::OperationResult ret = workspace_->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );

    // get result from tf buffer
    mapit::tf2::BufferCore buffer(workspace_.get(), "/tfs");
    mapit::tf::TransformStamped tf_b = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(0, 0));
    Eigen::Affine3f tf_from_icp(tf_b.transform.translation);
    tf_from_icp.rotate(tf_b.transform.rotation);

    // compare result
    Eigen::Affine3f tf_origin = getTfMatrix();
    QVERIFY( tf_from_icp.isApprox(tf_origin, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
}

void OPRegLocalICPTest::test_icp_for_more_than_one_input()
{
    // execute operator ICP
    mapit::msgs::OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("reg_local_icp");
    desc.set_params(
                "{"
                "   \"input\"             : [ \"bunny\", \"bunny2\", \"bunny3\" ],"
                "   \"target\"            : \"bunny_tf\","
                "   \"handle-result\"     : \"tf-add\","
                "   \"tf-prefix\"         : \"/tfs/\","
                "   \"tf-frame_id\"       : \"world\","
                "   \"tf-child_frame_id\" : \"bunny\","
                "   \"tf-is_static\"      : false,"
                "   \"matching-algorithm\" : \"icp\""
                "}"
                );
    mapit::OperationResult ret = workspace_->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );
}

void OPRegLocalICPTest::test_icp_tf_combine()
{
    // execute operator ICP
    mapit::msgs::OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("reg_local_icp");
    desc.set_params(
                "{"
                "   \"input\"             : [ \"bunny\", \"bunny2\" ],"
                "   \"target\"            : \"bunny_tf\","
                "   \"handle-result\"     : \"tf-combine\","
                "   \"tf-prefix\"         : \"/tf-combine/\","
                "   \"tf-frame_id\"       : \"world\","
                "   \"tf-child_frame_id\" : \"bunny\","
                "   \"tf-is_static\"      : false,"
                "   \"matching-algorithm\" : \"icp\""
                "}"
                );
    mapit::OperationResult ret = workspace_->doOperation( desc );
    QVERIFY( mapitIsOk(ret.first) );

    // get result from tf buffer
    mapit::tf2::BufferCore buffer(workspace_.get(), "/tf-combine");
    mapit::tf::TransformStamped tf_b0 = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(0, 1)); // 0.0 would get the "newest" tf but at 1ns it works and is close enough
    Eigen::Affine3f tf_b0_m(tf_b0.transform.translation);
    tf_b0_m.rotate(tf_b0.transform.rotation);
    mapit::tf::TransformStamped tf_b1 = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(1, 0));
    Eigen::Affine3f tf_b1_m(tf_b1.transform.translation);
    tf_b1_m.rotate(tf_b1.transform.rotation);
    mapit::tf::TransformStamped tf_b2 = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(2, 0));
    Eigen::Affine3f tf_b2_m(tf_b2.transform.translation);
    tf_b2_m.rotate(tf_b2.transform.rotation);
    mapit::tf::TransformStamped tf_b3 = buffer.lookupTransform("world", "bunny", mapit::time::from_sec_and_nsec(3, 0));
    Eigen::Affine3f tf_b3_m(tf_b3.transform.translation);
    tf_b3_m.rotate(tf_b3.transform.rotation);

    // get transforms to compare with
    Eigen::Affine3f tf_origin = getTfMatrix();
    Eigen::Affine3f tf_old_buffer_0 = Eigen::Affine3f::Identity();
    tf_old_buffer_0.translation() << 1., 0., 0.5;
    Eigen::Affine3f tf_old_buffer_1 = Eigen::Affine3f::Identity();
    tf_old_buffer_1.translation() << 1., 1., 0.5;
    Eigen::Affine3f tf_old_buffer_2 = Eigen::Affine3f::Identity();
    tf_old_buffer_2.translation() << 1., 1., 1.;
    Eigen::Affine3f tf_old_buffer_3 = Eigen::Affine3f::Identity();
    tf_old_buffer_3.translation() << 2., 1., 1.;

    // compare result
    QVERIFY( tf_b0_m.isApprox(tf_old_buffer_0, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
    QVERIFY( tf_b1_m.isApprox(tf_old_buffer_1 * tf_origin, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
    QVERIFY( tf_b2_m.isApprox(tf_old_buffer_2 * tf_origin, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
    QVERIFY( tf_b3_m.isApprox(tf_old_buffer_3, 0.06) ); // what does the epsilon mean (unit???)? 0.06 works, 0.055 does not
}

DECLARE_TEST(OPRegLocalICPTest)
