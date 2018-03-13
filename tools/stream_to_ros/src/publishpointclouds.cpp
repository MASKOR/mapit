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
