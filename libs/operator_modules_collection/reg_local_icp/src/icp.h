#ifndef ICP_H
#define ICP_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <list>
#include <string>

#include <Eigen/Geometry>

#include <upns/typedefs.h>
#include <mapit/time/time.h>

class QJsonDocument;

class PointcloudEntitydata;
namespace pcl {
template <typename PointT>
class PointCloud;
struct PointXYZ;
struct PCLHeader;
}

namespace upns {
class OperationEnvironment;
class CheckoutRaw;
}

namespace mapit {
namespace tf2 {
class BufferCore;
}

class ICP
{
public:
    ICP(upns::OperationEnvironment* env, upns::StatusCode& status);

    upns::StatusCode operate();
private:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> get_pointcloud(  std::string path
                                                                     , upns::StatusCode& status
                                                                     , mapit::time::Stamp &stamp
                                                                     , pcl::PCLHeader& header
                                                                     , std::shared_ptr<PointcloudEntitydata> entitydata
                                                                    );

    upns::StatusCode mapit_add_tf(  const time::Stamp &input_stamp
                                  , const Eigen::Affine3f &transform
                                 );
    upns::StatusCode mapit_remove_tfs(  const time::Stamp &stamp_start
                                      , const time::Stamp &stamp_end
                                     );

    // general setup
    upns::CheckoutRaw* checkout_;
    std::shared_ptr<mapit::tf2::BufferCore> tf_buffer_;

    std::string cfg_tf_prefix_;

    std::list<std::string> cfg_input_;
    std::string cfg_target_;

    bool cfg_use_frame_id_;
    std::string cfg_frame_id_;
    bool cfg_use_metascan_;

    enum class HandleResult {tf_add, tf_combine, data_change};
    HandleResult cfg_handle_result_;
    std::string cfg_tf_frame_id_;
    std::string cfg_tf_child_frame_id_;
    bool cfg_tf_is_static_;

    //--- algorithm configs ---
    enum class MatchingAlgorithm {ICP};
    MatchingAlgorithm cfg_matching_algorithm_;
    // ICP
    upns::StatusCode get_cfg_icp(upns::OperationEnvironment* env);
    void icp_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                     , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target
                     , pcl::PointCloud<pcl::PointXYZ>& result_pc
                     , Eigen::Affine3f& result_transform
                     , bool& has_converged
                     , double& fitness_score);

};

}
#endif // ICP_H
