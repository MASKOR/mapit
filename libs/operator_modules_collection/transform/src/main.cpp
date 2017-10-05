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

// Apply array of transforms.
// {
//    target: "path/to/target.tf"
//    mode: "relative"|"absolute",
//    tf: [
//      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
//      1: {mat: [0: m00_2, ...]},
//      2: ...
//    ]
// }
// relative:
// parent may be a tf or a tree
// if parent is a tree, timestamp is used to choose a appropiate coordinate system (lerp between two tfs).
// if parent is empty, target is used.
// absolute:
// only one tf (index:0) may be given, parent/timestamp must not be used.

upns::StatusCode getEntity(upns::OperationEnvironment* env, std::string entity_name, std::shared_ptr<mapit::msgs::Entity> &entity, std::shared_ptr<TfEntitydata> &entityData)
{
    // Get Target
    entity = env->getCheckout()->getEntity(entity_name);
    if(entity == NULL)
    {
        // If target could not be received, create new entity
        entity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        entity->set_type(TfEntitydata::TYPENAME());
        StatusCode s = env->getCheckout()->storeEntity(entity_name, entity);
        if(!upnsIsOk(s))
        {
            log_error("Failed to create entity.");
            return UPNS_STATUS_ERR_UNKNOWN;
        }
    }
    else
    {
        log_warn("Overwrite existing tf " + entity_name);
    }
    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( entity_name );
    if(abstractEntitydata == NULL)
    {
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    entityData = std::dynamic_pointer_cast<TfEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
        log_error("Entity is not of type TfEntitydata.");
        return UPNS_STATUS_ERR_UNKNOWN;
    }

    return UPNS_STATUS_OK;
}

void saveEntity(upns::OperationEnvironment* env, std::string entity_name, std::shared_ptr<mapit::msgs::Entity> entity, std::shared_ptr<TfEntitydata> entityData, tf::TransformStamped tf)
{
    // safe header to entity
    long sec, nsec;
    mapit::time::to_sec_and_nsec( tf.stamp, sec, nsec);
    mapit::msgs::Time *entity_stamp = new mapit::msgs::Time(); //entity->mutable_stamp();
    entity_stamp->set_sec( sec );
    entity_stamp->set_nsec( nsec );
    entity->set_allocated_stamp( entity_stamp );

    entity->set_frame_id(tf.frame_id);
    env->getCheckout()->storeEntity(entity_name, entity);

    // safe data to entitydata
    upns::tf::TransformPtr tf_transform( new upns::tf::Transform(tf.transform) );
    entityData->setData( tf_transform );
}

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

    std::string layername_dynamic("tf_dynamic");
    std::string layername_static("tf_static");
    std::string map = params["map"].toString().toStdString();

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

        // generate entity name for TF
        std::string entity_name = "";
        if (layer_is_static) {
            entity_name = map + "/" + layername_static + "/" + tf_loaded.frame_id + std::to_string( mapit::time::to_sec(tf_loaded.stamp) ) + tf_loaded.transform.child_frame_id;
        } else {
            entity_name = map + "/" + layername_dynamic + "/" + tf_loaded.frame_id + std::to_string( mapit::time::to_sec(tf_loaded.stamp) ) + tf_loaded.transform.child_frame_id;
        }

        std::shared_ptr<mapit::msgs::Entity> entity;
        std::shared_ptr<TfEntitydata> entityData;
        upns::StatusCode status = getEntity(env, entity_name, entity, entityData);
        if ( status != UPNS_STATUS_OK ) {
            return status;
        }

        saveEntity(env, entity_name, entity, entityData, tf_loaded);
    }

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "store transforms in mapit", "fhac", OPERATOR_VERSION, "any", &operate)
