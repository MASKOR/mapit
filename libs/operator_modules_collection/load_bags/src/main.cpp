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

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <mapit/time/time.h>

#include <tf2_msgs/TFMessage.h>
#include <mapit/layertypes/tflayer.h>

#include <sensor_msgs/PointCloud2.h>
#include <mapit/layertypes/pointcloudlayer.h>
#include <pcl_conversions/pcl_conversions.h>

mapit::StatusCode get_or_create_entity(  CheckoutRaw* checkout
                                      , const std::string& entity_name
                                      , const std::string& entity_type
                                      , std::shared_ptr<mapit::msgs::Entity>& entity)
{
    entity = checkout->getEntity(entity_name);
    if (entity == nullptr) {
        entity = std::make_shared<mapit::msgs::Entity>();
        entity->set_type( entity_type );
        checkout->storeEntity(entity_name, entity);
    }
    if ( 0 == entity->type().compare( entity_type ) ) {
        return MAPIT_STATUS_OK;
    } else {
        return MAPIT_STATUS_ERROR;
    }
}

std::string escape_slashes(const std::string& in)
{
   std::string out = in;
   std::replace( out.begin(), out.end(), '/', '_');
   return out;
}

mapit::StatusCode operate_load_bags(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "bags" : ["name_of_bag_1", ...],
     *  "translation_pairs" :
     *  [                       [repeated]
     *      {
     *          "topic" : ...,
     *          "prefix" : ...,
     *          "type" : ...,   [pointcloud, tf, TODO]
     *      },
     *      {
     *          ...
     *      }
     *  ]
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    CheckoutRaw* checkout = env->getCheckout();

    QJsonArray json_bag_names( params["bags"].toArray() );
    for (auto json_bag_name : json_bag_names) {
      std::string bag_name = json_bag_name.toString().toStdString();

      log_info("load_bags: Open bag " + bag_name);
      rosbag::Bag bag;
      bag.open(bag_name, rosbag::bagmode::Read);

      QJsonArray json_translation_pairs( params["translation_pairs"].toArray() );
      for (auto json_translation_pair : json_translation_pairs) {
        QJsonObject json_translation_pair_obj( json_translation_pair.toObject() );
        std::string topic_name = json_translation_pair_obj["topic"].toString().toStdString();
        std::string prefix_name = json_translation_pair_obj["prefix"].toString().toStdString() + "/";
        std::string type_name = json_translation_pair_obj["type"].toString().toStdString();

        std::string mapit_type = "";
        if (        0 == type_name.compare("pointcloud") ) {
          mapit_type = PointcloudEntitydata::TYPENAME();
        } else if ( 0 == type_name.compare("tf") ) {
          mapit_type = TfEntitydata::TYPENAME();
        } else {
          log_error("load_bags: unknown type specified");
          return MAPIT_STATUS_ERROR;
        }

        log_info("load_bags: Import [" + bag_name + "] : \"" + topic_name + "\"\t=> \"" + prefix_name + "/*\"\t(type: " + mapit_type + ")");

        std::vector<std::string> topics;
        topics.push_back( topic_name );
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // for tf
        mapit::tf::store::TransformStampedListGatherer tfs_map;

        // for all msgs
        for (const rosbag::MessageInstance msg : view) {
          if (        0 == mapit_type.compare( PointcloudEntitydata::TYPENAME() ) ) {
            sensor_msgs::PointCloud2::ConstPtr pc2 = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pc2 != NULL) {
              std::string entity_name = prefix_name + "/" + escape_slashes(pc2->header.frame_id + std::to_string(pc2->header.stamp.sec) + "." + std::to_string(pc2->header.stamp.nsec));

              std::shared_ptr<mapit::msgs::Entity> entity;
              mapit::StatusCode get_entity_status = get_or_create_entity(checkout, entity_name, PointcloudEntitydata::TYPENAME(), entity);
              if ( ! upnsIsOk(get_entity_status) ) {
                  log_error("load_bags: Can't get entity \"" + entity_name + "\" of type \"" + PointcloudEntitydata::TYPENAME() + "\"");
                  return get_entity_status;
              }
              entity->set_frame_id( pc2->header.frame_id );
              entity->mutable_stamp()->set_sec(pc2->header.stamp.sec);
              entity->mutable_stamp()->set_nsec(pc2->header.stamp.nsec);

              std::shared_ptr<pcl::PCLPointCloud2> entity_data = std::make_shared<pcl::PCLPointCloud2>();
              pcl_conversions::toPCL(*pc2, *entity_data);

              log_info("load_bags:   entity: " + entity_name);
              checkout->storeEntity(entity_name, entity);
              std::static_pointer_cast<PointcloudEntitydata>(checkout->getEntitydataForReadWrite(entity_name))->setData(entity_data);
            } else {
              log_error("load_bags: Data [" + bag_name + "] : \"" + topic_name + "\" is not of type \"sensor_msgs::PointCloud2\"");
              return MAPIT_STATUS_ERROR;
            }
          } else if ( 0 == mapit_type.compare( TfEntitydata::TYPENAME() ) ) {
            tf2_msgs::TFMessage::ConstPtr tf2 = msg.instantiate<tf2_msgs::TFMessage>();
            if (tf2 != NULL) {

              for (geometry_msgs::TransformStamped ros_tf : tf2->transforms) {
                std::unique_ptr<tf::TransformStamped> tf_loaded = std::make_unique<tf::TransformStamped>();

                tf_loaded->frame_id = ros_tf.header.frame_id;
                tf_loaded->stamp = mapit::time::from_sec_and_nsec( ros_tf.header.stamp.sec, ros_tf.header.stamp.nsec );
                tf_loaded->child_frame_id = ros_tf.child_frame_id;

                tf_loaded->transform.translation.x() = ros_tf.transform.translation.x;
                tf_loaded->transform.translation.y() = ros_tf.transform.translation.y;
                tf_loaded->transform.translation.z() = ros_tf.transform.translation.z;
                tf_loaded->transform.rotation.w() = ros_tf.transform.rotation.w;
                tf_loaded->transform.rotation.x() = ros_tf.transform.rotation.x;
                tf_loaded->transform.rotation.y() = ros_tf.transform.rotation.y;
                tf_loaded->transform.rotation.z() = ros_tf.transform.rotation.z;

                bool is_static = false;
                if (0 == msg.getTopic().compare("/tf_static")) {
                    is_static = true;
                }

                tfs_map.add_transform( std::move( tf_loaded ), is_static );
              }
            } else {
              log_error("load_bags: Data [" + bag_name + "] : \"" + topic_name + "\" is not of type \"tf2_msgs::TFMessage\"");
              return MAPIT_STATUS_ERROR;
            }
          }
        }
        tfs_map.store_entities( checkout, prefix_name );
      }
    }

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "load bagfiles in mapit", "fhac", OPERATOR_VERSION, "any", &operate_load_bags)
