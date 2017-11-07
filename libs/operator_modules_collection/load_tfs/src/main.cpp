#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

upns::StatusCode
save_data(  CheckoutRaw* checkout
          , std::shared_ptr<mapit::Layer> layer
          , std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>& tfs_map
         )
{
  for (auto tf_list : tfs_map) {
    std::shared_ptr<mapit::Entity> entity = checkout->getExistingOrNewEntity(
                                              layer
                                              , tf_list.first
                                              );
    std::shared_ptr<AbstractEntitydata> ed_a = checkout->getEntityDataReadWrite(entity);
    std::shared_ptr<upns::tf::store::TransformStampedList> ed;
    if (ed_a == nullptr) {
      // create new
      ed = tf_list.second;
    } else {
      // appand
      ed = std::dynamic_pointer_cast<upns::tf::store::TransformStampedList>( ed_a );
      if (ed == nullptr) {
        log_error("load_tf: internal error, entity for tf with name " + entity->getName() + " has entity date of other type then tf");
        return UPNS_STATUS_ERROR;
      }

      for (auto &tf : *tf_list.second->dispose()) {
        ed->add_TransformStamped(std::move( tf ));
      }
    }
    checkout->storeEntity<upns::tf::store::TransformStampedList>(entity, ed);
  }

  return UPNS_STATUS_OK;
}

upns::StatusCode operate_load_tfs(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "map" : ...,
     *  "dynamic_layer" : ...,  [optional]
     *  "static_layer" : ...,   [optional]
     *  "transforms" :
     *  [                       [repeated]
     *      {
     *          "static" : ...,
     *          "header" : {
     *              "frame_id" : ...,
     *              "stamp" : {
     *                  "sec" : ...,
     *                  "nsec" : ...
     *              }
     *          },
     *
     *          "transform" : {
     *              "child_frame_id" : ...,
     *              "translation" : {
     *                  "x" : ...,
     *                  "y" : ...,
     *                  "z" : ...
     *              },
     *              "rotation" : {
     *                  "w" : ...,
     *                  "x" : ...,
     *                  "y" : ...,
     *                  "z" : ...
     *              }
     *          }
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
    map_name = CheckoutCommon::getMapPathOfEntry(map_name);

    CheckoutRaw* checkout = env->getCheckout();
    std::shared_ptr<mapit::Map> map = checkout->getExistingOrNewMap(map_name);
    std::shared_ptr<mapit::Layer> layer_static, layer_dynamic;
    layer_static = checkout->getExistingOrNewLayer(map, upns::tf::_DEFAULT_LAYER_NAME_STATIC_, TfEntitydata::TYPENAME());
    layer_dynamic = checkout->getExistingOrNewLayer(map, upns::tf::_DEFAULT_LAYER_NAME_DYNAMIC_, TfEntitydata::TYPENAME());

    std::shared_ptr<std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>> tfs_map_static
        = std::make_shared<std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>>();
    std::shared_ptr<std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>> tfs_map_dynamic
        = std::make_shared<std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>>();

    // TODO check if data is available
    QJsonArray json_transforms( params["transforms"].toArray() );
    for ( QJsonValue json_value_tf : json_transforms ) {
        QJsonObject json_tf( json_value_tf.toObject() );

        bool layer_is_static = json_tf["static"].toBool();

        // get data from json
        std::unique_ptr<tf::TransformStamped> tf_loaded = std::unique_ptr<tf::TransformStamped>(new tf::TransformStamped);

        // get header
        QJsonObject header( json_tf["header"].toObject() );
        tf_loaded->frame_id = header["frame_id"].toString().toStdString();
        QJsonObject stamp( header["stamp"].toObject() );
        // TODO when int is not enough we can use string and cast this to long (jason does not support long)
        tf_loaded->stamp = mapit::time::from_sec_and_nsec(stamp["sec"].toInt(), stamp["nsec"].toInt());

        // get data
        QJsonObject json_transform( json_tf["transform"].toObject() );
        tf_loaded->child_frame_id = json_transform["child_frame_id"].toString().toStdString();
        QJsonObject json_translation( json_transform["translation"].toObject() );
        QJsonObject json_rotation( json_transform["rotation"].toObject() );
        tf_loaded->transform.translation = Eigen::Translation3f(
                    (float)json_translation["x"].toDouble()
                ,   (float)json_translation["y"].toDouble()
                ,   (float)json_translation["z"].toDouble()
                    );

        tf_loaded->transform.rotation = Eigen::Quaternionf(
                    (float)json_rotation["w"].toDouble()
                ,   (float)json_rotation["x"].toDouble()
                ,   (float)json_rotation["y"].toDouble()
                ,   (float)json_rotation["z"].toDouble()
                    );

        tf_loaded->transform.translation = Eigen::Translation3f(
                    (float)json_translation["x"].toDouble()
                ,   (float)json_translation["y"].toDouble()
                ,   (float)json_translation["z"].toDouble()
                    );
        tf_loaded->transform.rotation.normalize();

        bool valid = std::abs((tf_loaded->transform.rotation.w() * tf_loaded->transform.rotation.w()
                             + tf_loaded->transform.rotation.x() * tf_loaded->transform.rotation.x()
                             + tf_loaded->transform.rotation.y() * tf_loaded->transform.rotation.y()
                             + tf_loaded->transform.rotation.z() * tf_loaded->transform.rotation.z()) - 1.0f) < 10e-6;
        if(!valid)
        {
            log_warn("Invalid Quaternion 0.");
            return UPNS_STATUS_ERROR;
        }

        // add data to map
        std::shared_ptr<std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>> tfs_map;
        if (layer_is_static) {
            tfs_map = tfs_map_static;
        } else {
            tfs_map = tfs_map_dynamic;
        }

        std::string entity_name = upns::tf::store::TransformStampedList::get_entity_name(tf_loaded->frame_id, tf_loaded->child_frame_id);
        // if not yet in list
        if (tfs_map->find(entity_name) == tfs_map->end()) {
          // create empty and add to map
          std::shared_ptr<upns::tf::store::TransformStampedList> tfs_store = std::make_shared<upns::tf::store::TransformStampedList>(tf_loaded->frame_id, tf_loaded->child_frame_id);
          (*tfs_map)[ entity_name ] = tfs_store;
        }

        std::map<std::string, std::shared_ptr<upns::tf::store::TransformStampedList>>::iterator tfs_store_it = tfs_map->find(entity_name);
        tfs_store_it->second->add_TransformStamped(std::move(tf_loaded));
    }

    upns::StatusCode usc_static = save_data(checkout, layer_static, *tfs_map_static);
    upns::StatusCode usc_dynamic = save_data(checkout, layer_dynamic, *tfs_map_dynamic);

    if (   usc_static == UPNS_STATUS_OK
           && usc_dynamic == UPNS_STATUS_OK) {
      return UPNS_STATUS_OK;
    } else {
      return UPNS_STATUS_ERROR;
    }

}

UPNS_MODULE(OPERATOR_NAME, "store transforms in mapit", "fhac", OPERATOR_VERSION, "any", &operate_load_tfs)
