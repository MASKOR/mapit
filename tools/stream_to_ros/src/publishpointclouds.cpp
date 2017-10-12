#include <sensor_msgs/PointCloud2.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "publishpointclouds.h"

void
PublishPointClouds::publish_entity(std::shared_ptr<mapit::Entity> entity)
{
  // get data
  std::shared_ptr<upns::AbstractEntitydata> entity_data_abstract = checkout_->getEntityDataReadOnly(entity);
  std::shared_ptr<pcl::PCLPointCloud2> entity_data =
      std::dynamic_pointer_cast<PointcloudEntitydata>(
        entity_data_abstract
        )->getData();

  // convert data
  sensor_msgs::PointCloud2 entity_data_publishable;
  pcl_conversions::fromPCL(*entity_data, entity_data_publishable);
  // don't forget the header
  entity_data_publishable.header = get_header(entity);

  // publish
  publisher_->publish( entity_data_publishable );
}
