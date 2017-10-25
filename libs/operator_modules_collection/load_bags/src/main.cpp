#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>
#include <upns/layertypes/tflayer.h>

#include <sensor_msgs/PointCloud2.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl_conversions/pcl_conversions.h>

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "map" : ...,
     *  "bags" : ["name_of_bag_1", ...],
     *  "translation_pairs" :
     *  [                       [repeated]
     *      {
     *          "topic" : ...,
     *          "layer" : ...,
     *          "type" : ...,   [pointcloud, tf, TODO]
//     *          "is_static" : ..., [only for tf]
     *      },
     *      {
     *          ...
     *      }
     *  ]
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string map_name = params["map"].toString().toStdString();

    CheckoutRaw* checkout = env->getCheckout();
    std::shared_ptr<mapit::Map> map = checkout->getExistingOrNewMap(map_name);

    QJsonArray json_bag_names( params["bags"].toArray() );
    for (auto json_bag_name : json_bag_names) {
      std::string bag_name = json_bag_name.toString().toStdString();

      log_info("Open bag " + bag_name);
      rosbag::Bag bag;
      bag.open(bag_name, rosbag::bagmode::Read);

      QJsonArray json_translation_pairs( params["translation_pairs"].toArray() );
      for (auto json_translation_pair : json_translation_pairs) {
        QJsonObject json_translation_pair_obj( json_translation_pair.toObject() );
        std::string topic_name = json_translation_pair_obj["topic"].toString().toStdString();
        std::string layer_name = json_translation_pair_obj["layer"].toString().toStdString();
        std::string type_name = json_translation_pair_obj["type"].toString().toStdString();

        log_info("Import [" + bag_name + "] : \"" + topic_name + "\"\t=> \"" + map_name + "/" + layer_name + "\"\t(type: " + type_name + ")");

        std::string mapit_type = "";
        if (        0 == type_name.compare("pointcloud") ) {
          mapit_type = PointcloudEntitydata::TYPENAME();
        } else if ( 0 == type_name.compare("tf") ) {
          mapit_type = TfEntitydata::TYPENAME();
        } else {
          log_error("unknown type specified");
          return UPNS_STATUS_ERROR;
        }
        std::shared_ptr<mapit::Layer> layer = checkout->getExistingOrNewLayer(map, layer_name, mapit_type);

        std::vector<std::string> topics;
        topics.push_back( topic_name );
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // for tf
        upns::tf::store::TransformStampedListGatherer tfs_map;

        // for all msgs
        for (const rosbag::MessageInstance msg : view) {
          if (        0 == type_name.compare("pointcloud") ) {
            sensor_msgs::PointCloud2::ConstPtr pc2 = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pc2 != NULL) {
              std::string entity_name = pc2->header.frame_id + std::to_string(pc2->header.stamp.sec) + "." + std::to_string(pc2->header.stamp.nsec);

              std::shared_ptr<mapit::Entity> entity = checkout->getExistingOrNewEntity(layer, entity_name);
              entity->set_frame_id( pc2->header.frame_id );
              entity->set_stamp( mapit::time::from_sec_and_nsec( pc2->header.stamp.sec, pc2->header.stamp.nsec ));

              std::shared_ptr<pcl::PCLPointCloud2> entity_data = std::make_shared<pcl::PCLPointCloud2>();
              pcl_conversions::toPCL(*pc2, *entity_data);

              log_info("  entity: " + entity_name);
              checkout->storeEntity(entity, entity_data);
            } else {
              log_error("Data [" + bag_name + "] : \"" + topic_name + "\" is not of type \"sensor_msgs::PointCloud2\"");
              return UPNS_STATUS_ERROR;
            }
          } else if ( 0 == type_name.compare("tf") ) {
            tf2_msgs::TFMessage::ConstPtr tf2 = msg.instantiate<tf2_msgs::TFMessage>();
            if (tf2 != NULL) {

              for (geometry_msgs::TransformStamped ros_tf : tf2->transforms) {
                std::unique_ptr<tf::TransformStamped> tf_loaded = std::unique_ptr<tf::TransformStamped>(new tf::TransformStamped);

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

                tfs_map.add_transform( std::move( tf_loaded ) );
              }
            } else {
              log_error("Data [" + bag_name + "] : \"" + topic_name + "\" is not of type \"tf2_msgs::TFMessage\"");
              return UPNS_STATUS_ERROR;
            }
          }
        }
        tfs_map.store_entities( checkout, layer );
      }
    }

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "store transforms in mapit", "fhac", OPERATOR_VERSION, "any", &operate)
