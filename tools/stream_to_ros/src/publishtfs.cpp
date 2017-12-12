#include <tf2_msgs/TFMessage.h>
#include <upns/layertypes/tflayer.h>

#include "publishtfs.h"

void
PublishTFs::publish_entity(std::shared_ptr<mapit::Entity> entity)
{
  // get data
  std::shared_ptr<upns::AbstractEntitydata> entity_data_abstract = checkout_->getEntityDataReadOnly(entity);
  std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> tfs =
      std::static_pointer_cast<TfEntitydata>(
        entity_data_abstract
        )->getData()->dispose();

  type_data new_tf_map;
  for (std::unique_ptr<upns::tf::TransformStamped>& tf : *tfs) {
    double time = mapit::time::to_sec( tf->stamp );
    new_tf_map.insert(std::pair<double, std::unique_ptr<upns::tf::TransformStamped>>( time, std::move(tf) ));
  }

  std::pair<type_data::iterator, type_data> tmp;
  tmp.second = std::move( new_tf_map );
  tmp.first = tmp.second.begin();
  tf_entitys_.push_back( std::move( tmp ) );

  log_info("start to publish tf entities");
  tf_timer_ = std::make_shared<ros::Timer>( node_handle_->createTimer(ros::Rate(100), &PublishTFs::tf_timer_callback, this) );
}

void
PublishTFs::publish_one_tf_entity(std::unique_ptr<upns::tf::TransformStamped> tf_stamped)
{
  // convert data
  tf2_msgs::TFMessage entity_data_publishable;
  geometry_msgs::TransformStamped ros_tf;
  ros_tf.child_frame_id = tf_stamped->child_frame_id;
  ros_tf.transform.rotation.w = tf_stamped->transform.rotation.w();
  ros_tf.transform.rotation.x = tf_stamped->transform.rotation.x();
  ros_tf.transform.rotation.y = tf_stamped->transform.rotation.y();
  ros_tf.transform.rotation.z = tf_stamped->transform.rotation.z();
  ros_tf.transform.translation.x = tf_stamped->transform.translation.x();
  ros_tf.transform.translation.y = tf_stamped->transform.translation.y();
  ros_tf.transform.translation.z = tf_stamped->transform.translation.z();

  // don't forget the header
  ros_tf.header.frame_id = tf_stamped->frame_id;
  ros::Time et;
  et.fromSec( mapit::time::to_sec( tf_stamped->stamp ) );
  ros_tf.header.stamp = et;

  entity_data_publishable.transforms.push_back(ros_tf);

  log_info("Publish tf to topic \"" + publisher_->getTopic() + "\" in ROS");

  // publish
  publisher_->publish( entity_data_publishable );
}

void
PublishTFs::tf_timer_callback(const ros::TimerEvent&)
{
  // check for end
  for (std::pair<type_data::iterator, type_data>& tf_entity : tf_entitys_) {
    if (tf_entity.first == tf_entity.second.end()) {
      log_warn("reached end of tf entity");
      tf_timer_ = nullptr;
      return;
    }

    log_warn("reached end of tf entity");
    // for all where the time is over
    double time_ros = ros::Time::now().toSec();
    for (
         ;    tf_entity.first != tf_entity.second.end()
           && tf_entity.first->first <= time_ros - offset_
         ; ++tf_entity.first) {
      std::unique_ptr<upns::tf::TransformStamped> tmp;
      if (tf_entity.first->second == nullptr) {
        tmp = nullptr;
      } else {
        tmp = std::move( tf_entity.first->second );
      }
  //    std::unique_ptr<upns::tf::TransformStamped> tmp = std::move( tf_entity_next_->second );
      publish_one_tf_entity( std::move( tmp ) );
    }
  }
}
