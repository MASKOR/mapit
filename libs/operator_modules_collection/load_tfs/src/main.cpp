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

upns::StatusCode operate(upns::OperationEnvironment* env)
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

    CheckoutRaw* checkout = env->getCheckout();
    std::shared_ptr<mapit::Map> map = checkout->getExistingOrNewMap(map_name);
    std::shared_ptr<mapit::Layer> layer_static, layer_dynamic;
    layer_static = checkout->getExistingOrNewLayer(map, upns::tf::_DEFAULT_LAYER_NAME_STATIC_, TfEntitydata::TYPENAME());
    layer_dynamic = checkout->getExistingOrNewLayer(map, upns::tf::_DEFAULT_LAYER_NAME_DYNAMIC_, TfEntitydata::TYPENAME());

    // TODO check if data is available
    QJsonArray json_transforms( params["transforms"].toArray() );
    for ( QJsonValue json_value_tf : json_transforms ) {
        QJsonObject json_tf( json_value_tf.toObject() );

        bool layer_is_static = json_tf["static"].toBool();

        // get data from json
        tf::TransformStamped tf_loaded;

        // get header
        QJsonObject header( json_tf["header"].toObject() );
        tf_loaded.frame_id = header["frame_id"].toString().toStdString();
        QJsonObject stamp( header["stamp"].toObject() );
        // TODO when int is not enough we can use string and cast this to long (jason does not support long)
        tf_loaded.stamp = mapit::time::from_sec_and_nsec(stamp["sec"].toInt(), stamp["nsec"].toInt());

        // get data
        QJsonObject json_transform( json_tf["transform"].toObject() );
        tf_loaded.transform.child_frame_id = json_transform["child_frame_id"].toString().toStdString();
        QJsonObject json_translation( json_transform["translation"].toObject() );
        QJsonObject json_rotation( json_transform["rotation"].toObject() );
        tf_loaded.transform.translation = Eigen::Translation3f(
                    (float)json_translation["x"].toDouble()
                ,   (float)json_translation["y"].toDouble()
                ,   (float)json_translation["z"].toDouble()
                    );

        tf_loaded.transform.rotation = Eigen::Quaternionf(
                    (float)json_rotation["w"].toDouble()
                ,   (float)json_rotation["x"].toDouble()
                ,   (float)json_rotation["y"].toDouble()
                ,   (float)json_rotation["z"].toDouble()
                    );

        // write data
        std::shared_ptr<mapit::Layer> layer;
        if (layer_is_static) {
            layer = layer_static;
        } else {
            layer = layer_dynamic;
        }
        unsigned long sec, nsec;
        mapit::time::to_sec_and_nsec(tf_loaded.stamp, sec, nsec);
        std::shared_ptr<mapit::Entity> entity = checkout->getExistingOrNewEntity(
                      layer
                    , tf_loaded.frame_id + std::to_string( sec ) + std::to_string( nsec ) + tf_loaded.transform.child_frame_id);

        std::shared_ptr<tf::Transform> entity_data = std::shared_ptr<tf::Transform>(new tf::Transform(tf_loaded.transform));
        entity->set_frame_id( tf_loaded.frame_id );
        entity->set_stamp( tf_loaded.stamp );
        checkout->storeEntity<tf::Transform>(entity, entity_data);
    }

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "store transforms in mapit", "fhac", OPERATOR_VERSION, "any", &operate)
