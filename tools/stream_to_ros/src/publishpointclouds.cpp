#include <sensor_msgs/PointCloud2.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "publishpointclouds.h"

void
PublishPointClouds::publish_entity(const std::string& entity_name, const std::shared_ptr<::Entity>& entity)
{
  // get data
  std::shared_ptr<upns::AbstractEntitydata> entity_data_abstract = checkout_->getEntitydataReadOnly(entity_name);
  if ( entity_data_abstract == nullptr || 0 != std::strcmp(entity_data_abstract->type(), PointcloudEntitydata::TYPENAME()) ) {
    log_error("stream_to_ros: can't publish " + entity_name + " since the type is wrongly given\n"
              "type is: " + entity_data_abstract->type() + " but we need: " + PointcloudEntitydata::TYPENAME());
    return;
  }
  std::shared_ptr<pcl::PCLPointCloud2> entity_data =
      std::static_pointer_cast<PointcloudEntitydata>(
        entity_data_abstract
        )->getData();

  // convert data
  sensor_msgs::PointCloud2 entity_data_publishable;
  pcl_conversions::fromPCL(*entity_data, entity_data_publishable);
  // don't forget the header
  entity_data_publishable.header = get_header(entity->entity);

  log_info("Publish entity \"" + entity_name + "\" to topic \"" + publisher_->getTopic() + "\" in ROS");

  // publish
  publisher_->publish( entity_data_publishable );
}
