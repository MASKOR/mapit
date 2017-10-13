#include <tf2_msgs/TFMessage.h>
#include <upns/layertypes/tflayer.h>

#include "publishtfs.h"

void
PublishTFs::publish_entity(std::shared_ptr<mapit::Entity> entity)
{
  // get data
  std::shared_ptr<upns::AbstractEntitydata> entity_data_abstract = checkout_->getEntityDataReadOnly(entity);
  std::shared_ptr<upns::tf::Transform> entity_data =
      std::dynamic_pointer_cast<TfEntitydata>(
        entity_data_abstract
        )->getData();

  // convert data
  tf2_msgs::TFMessage entity_data_publishable;
  geometry_msgs::TransformStamped ros_tf;
  ros_tf.child_frame_id = entity_data->child_frame_id;
  ros_tf.transform.rotation.w = entity_data->rotation.w();
  ros_tf.transform.rotation.x = entity_data->rotation.x();
  ros_tf.transform.rotation.y = entity_data->rotation.y();
  ros_tf.transform.rotation.z = entity_data->rotation.z();
  ros_tf.transform.translation.x = entity_data->translation.x();
  ros_tf.transform.translation.y = entity_data->translation.y();
  ros_tf.transform.translation.z = entity_data->translation.z();

  // don't forget the header
  ros_tf.header = get_header(entity);

  entity_data_publishable.transforms.push_back(ros_tf);

  log_info("Publish entity \"" + entity->getName() + "\" to topic \"" + publisher_->getTopic() + "\" in ROS");

  // publish
  publisher_->publish( entity_data_publishable );
}
