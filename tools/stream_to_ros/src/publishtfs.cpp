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

#include <tf2_msgs/TFMessage.h>
#include <mapit/layertypes/tflayer.h>

#include "publishtfs.h"

void
PublishTFs::publish_entity(const std::string& entity_name, const std::shared_ptr<::Entity>& entity)
{
  // get data
  std::shared_ptr<mapit::AbstractEntitydata> entity_data_abstract = checkout_->getEntitydataReadOnly(entity_name);
  if ( entity_data_abstract == nullptr || 0 != std::strcmp(entity_data_abstract->type(), TfEntitydata::TYPENAME()) ) {
    log_error("stream_to_ros: can't publish " + entity_name + " since the type is wrongly given\n"
              "type is: " + entity_data_abstract->type() + " but we need: " + TfEntitydata::TYPENAME());
    return;
  }
  std::unique_ptr<std::list<std::unique_ptr<mapit::tf::TransformStamped>>> tfs =
      std::static_pointer_cast<TfEntitydata>(
        entity_data_abstract
        )->getData()->dispose();

  type_data new_tf_map;
  for (std::unique_ptr<mapit::tf::TransformStamped>& tf : *tfs) {
    double time = mapit::time::to_sec( tf->stamp );
    new_tf_map.insert(std::pair<double, std::unique_ptr<mapit::tf::TransformStamped>>( time, std::move(tf) ));
  }

  std::pair<type_data::iterator, type_data> tmp;
  tmp.second = std::move( new_tf_map );
  tmp.first = tmp.second.begin();
  tf_entitys_.push_back( std::move( tmp ) );

  log_info("start to publish tf entities");
  // TODO: there should be a special case when a tf entity should be published to ROS, but not in playback mode
  //       its not implemented yet, since I can't imagen the result or the usecase
  tf_timer_ = std::make_shared<ros::Timer>( node_handle_->createTimer(ros::Rate(100), &PublishTFs::tf_timer_callback, this) );
}

void
PublishTFs::publish_one_tf_entity(std::unique_ptr<mapit::tf::TransformStamped> tf_stamped)
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
      std::unique_ptr<mapit::tf::TransformStamped> tmp;
      if (tf_entity.first->second == nullptr) {
        tmp = nullptr;
      } else {
        tmp = std::move( tf_entity.first->second );
      }
  //    std::unique_ptr<mapit::tf::TransformStamped> tmp = std::move( tf_entity_next_->second );
      publish_one_tf_entity( std::move( tmp ) );
    }
  }
}
