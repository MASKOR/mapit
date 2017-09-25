#include "tf_test.h"
#include "../../src/autotest.h"

#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/time/time.h>

#include <typeinfo>
#include <iostream>

void TFTest::init()
{
}

void TFTest::cleanup()
{
}

void TFTest::initTestCase()
{
}

void TFTest::cleanupTestCase()
{
}

void TFTest::compareTfs(upns::tf::TransformStamped a, upns::tf::TransformStamped b)
{
    QVERIFY(a.frame_id == b.frame_id);
    QVERIFY(a.stamp == b.stamp);
    QVERIFY(a.transform.child_frame_id == b.transform.child_frame_id);
    QVERIFY( fabs(a.transform.translation.x() - b.transform.translation.x()) < 1e-6);
    QVERIFY( fabs(a.transform.translation.y() - b.transform.translation.y()) < 1e-6);
    QVERIFY( fabs(a.transform.translation.z() - b.transform.translation.z()) < 1e-6);

    QVERIFY( fabs((a.transform.rotation * b.transform.rotation.inverse()).w()) - 1 < 1e-6);
    QVERIFY( fabs((a.transform.rotation * b.transform.rotation.inverse()).x()) < 1e-6);
    QVERIFY( fabs((a.transform.rotation * b.transform.rotation.inverse()).y()) < 1e-6);
    QVERIFY( fabs((a.transform.rotation * b.transform.rotation.inverse()).z()) < 1e-6);
}

void TFTest::test_input_output()
{
    std::shared_ptr<mapit::tf2::BufferCore> buffer;
    buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(mapit::time::seconds(10)));

    upns::tf::TransformStamped in_out;
    in_out.frame_id  = "in";
    in_out.transform.child_frame_id = "out";
    in_out.stamp = mapit::time::from_sec_and_nsec(1001, 10002);
    in_out.transform.translation = Eigen::Translation3f(1, 2, 3);
    in_out.transform.rotation = Eigen::Quaternionf::Identity();

    buffer->setTransform(in_out, "layername", false);

    compareTfs(in_out, buffer->lookupTransform("in", "out", mapit::time::from_sec_and_nsec(1001, 10002)));

    // test input = output rotation + translation
    buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(mapit::time::seconds(10)));
    upns::tf::TransformStamped in_out_rot;
    in_out_rot.frame_id  = "in";
    in_out_rot.transform.child_frame_id = "out";
    in_out_rot.stamp = mapit::time::from_sec_and_nsec(1001, 10002);
    in_out_rot.transform.translation = Eigen::Translation3f(1, 2, 3);
    in_out_rot.transform.rotation =
              Eigen::AngleAxisf(-1, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ());

    buffer->setTransform(in_out_rot, "layername", false);

    compareTfs(in_out_rot, buffer->lookupTransform("in", "out", mapit::time::from_sec_and_nsec(1001, 10002)));
}

void TFTest::test_chain_of_2_tfs()
{
    std::shared_ptr<mapit::tf2::BufferCore> buffer;
    buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(mapit::time::seconds(10)));

    upns::tf::TransformStamped chain_1;
    chain_1.frame_id  = "cb";
    chain_1.transform.child_frame_id = "cm";
    chain_1.stamp = mapit::time::from_sec_and_nsec(1001, 10002);
    chain_1.transform.translation = Eigen::Translation3f(1, 2, 3);
    chain_1.transform.rotation =
              Eigen::AngleAxisf(-1, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ());

    buffer->setTransform(chain_1, "layername", false);

    upns::tf::TransformStamped chain_2;
    chain_2.frame_id  = "cm";
    chain_2.transform.child_frame_id = "ce";
    chain_2.stamp = mapit::time::from_sec_and_nsec(1001, 10002);
    chain_2.transform.translation = Eigen::Translation3f(1, 2, 3);
    chain_2.transform.rotation =
              Eigen::AngleAxisf(-1, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ());

    buffer->setTransform(chain_2, "layername", false);

    upns::tf::TransformStamped chain_together;
    chain_together.frame_id = chain_1.frame_id;
    chain_together.stamp = chain_1.stamp;
    chain_together.transform.child_frame_id = chain_2.transform.child_frame_id;
    Eigen::Affine3f c1m = chain_1.transform.translation * chain_1.transform.rotation;
    Eigen::Affine3f c2m = chain_2.transform.translation * chain_2.transform.rotation;
    Eigen::Affine3f chain_tf = c1m * c2m;
    chain_together.transform.rotation = chain_tf.rotation();
    chain_together.transform.translation = Eigen::Translation3f(chain_tf.translation());

    compareTfs(chain_together, buffer->lookupTransform("cb", "ce", mapit::time::from_sec_and_nsec(1001, 10002)));
}

void TFTest::test_interpolation()
{
    std::shared_ptr<mapit::tf2::BufferCore> buffer;
    buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(mapit::time::seconds(10)));

    upns::tf::TransformStamped inter_1;
    inter_1.frame_id  = "a";
    inter_1.transform.child_frame_id = "b";
    inter_1.stamp = mapit::time::from_sec_and_nsec(1000, 0);
    inter_1.transform.translation = Eigen::Translation3f(1, 2, 3);
    inter_1.transform.rotation =
              Eigen::AngleAxisf(-1, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ());

    buffer->setTransform(inter_1, "layername", false);

    upns::tf::TransformStamped inter_2;
    inter_2.frame_id  = "a";
    inter_2.transform.child_frame_id = "b";
    inter_2.stamp = mapit::time::from_sec_and_nsec(1002, 0);
    inter_2.transform.translation = Eigen::Translation3f(5, 7, 11);
    inter_2.transform.rotation =
              Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

    buffer->setTransform(inter_2, "layername", false);

    // calculate by food
    upns::tf::TransformStamped inter_together;
    inter_together.frame_id = inter_1.frame_id;
    inter_together.stamp    = mapit::time::from_sec_and_nsec(1001, 0);
    inter_together.transform.child_frame_id = inter_1.transform.child_frame_id;
    float time_1 = (inter_together.stamp - inter_1.stamp).count();
    float time_2 = (inter_2.stamp - inter_together.stamp).count();
    float factor_1 = time_1 / (time_1 + time_2);
    float factor_2 = time_2 / (time_1 + time_2);

    inter_together.transform.translation = Eigen::Translation3f(
                  factor_1 * inter_1.transform.translation.x() + factor_2 * inter_2.transform.translation.x()
                , factor_1 * inter_1.transform.translation.y() + factor_2 * inter_2.transform.translation.y()
                , factor_1 * inter_1.transform.translation.z() + factor_2 * inter_2.transform.translation.z()
                );

    inter_together.transform.rotation = inter_1.transform.rotation.slerp(
                  factor_1
                , inter_2.transform.rotation
                );

    compareTfs(inter_together, buffer->lookupTransform("a", "b", inter_together.stamp));
}

void TFTest::test_tf()
{
    test_input_output();
    test_chain_of_2_tfs();
    test_interpolation();
}

DECLARE_TEST(TFTest)
