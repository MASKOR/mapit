/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/layertypes/tflayer.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

mapit::StatusCode operate_load_tfs(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "prefix" : ...,
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

    std::string prefix = params["prefix"].toString().toStdString();

    mapit::operators::WorkspaceWritable* workspace = env->getWorkspace();

    std::shared_ptr<mapit::tf::store::TransformStampedListGatherer> tfs_map
        = std::make_shared<mapit::tf::store::TransformStampedListGatherer>();

    // TODO check if data is available
    QJsonArray json_transforms( params["transforms"].toArray() );
    for ( QJsonValue json_value_tf : json_transforms ) {
        QJsonObject json_tf( json_value_tf.toObject() );

        bool is_static = json_tf["static"].toBool();

        // get data from json
        std::unique_ptr<mapit::tf::TransformStamped> tf_loaded = std::unique_ptr<mapit::tf::TransformStamped>(new mapit::tf::TransformStamped);

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
            return MAPIT_STATUS_ERROR;
        }

        // add data to map
        tfs_map->add_transform( std::move( tf_loaded ), is_static );
    }

    mapit::StatusCode usc_static = tfs_map->store_entities(workspace, prefix);

    if (   usc_static == MAPIT_STATUS_OK) {
      return MAPIT_STATUS_OK;
    } else {
      return usc_static;
    }

}

MAPIT_MODULE(OPERATOR_NAME, "store transforms in mapit", "fhac", OPERATOR_VERSION, "any", &operate_load_tfs)
