#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <upns/depthfirstsearch.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <upns/layertypes/pointcloudlayer.h>

double factorX_ = 0.0;
double factorY_ = 0.0;
double factorZ_ = 0.0;

void scalePoint(std::vector<pcl::uint8_t>& data, const size_t& offset, const ::pcl::PCLPointField& type, const double& factor)
{
    switch (type.datatype) {
        case ::pcl::PCLPointField::FLOAT32: {
            float* p = reinterpret_cast<float*>( &(data[offset + type.offset]) );
            (*p) *= factor;
        }
        break;
        case ::pcl::PCLPointField::FLOAT64: {
            double* p = reinterpret_cast<double*>( &(data[offset + type.offset]) );
            (*p) *= factor;
        }
        break;
        default:
            log_error("scale_pointcloud: pointtype is not implemented");
        break;
    }
}

upns::StatusCode scalePointcloud(CheckoutRaw* checkout, std::string path, std::shared_ptr<mapit::msgs::Entity> entity)
{
    std::shared_ptr<AbstractEntitydata> edA = checkout->getEntitydataForReadWrite(path);
    if ( 0 != std::strcmp(edA->type(), PointcloudEntitydata::TYPENAME()) ) {
        log_error("scale_pointcloud: entity \"" + path
                + "\" is of type \"" + edA->type()
                + "\", but type \"" + PointcloudEntitydata::TYPENAME() + "\" is needed for this operator");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> ed = std::static_pointer_cast<PointcloudEntitydata>(edA);
    std::shared_ptr<pcl::PCLPointCloud2> edPointcloud = ed->getData();

    uint32_t pointSize = edPointcloud->point_step;
    uint32_t offsetX = -1;
    uint32_t offsetY = -1;
    uint32_t offsetZ = -1;
    ::pcl::PCLPointField typeX;
    ::pcl::PCLPointField typeY;
    ::pcl::PCLPointField typeZ;

    for (::pcl::PCLPointField field : edPointcloud->fields) {
        if (        0 == field.name.compare("x") ) {
            offsetX = field.offset;
            typeX = field;
        } else if ( 0 == field.name.compare("y") ) {
            offsetY = field.offset;
            typeY = field;
        } else if ( 0 == field.name.compare("z") ) {
            offsetZ = field.offset;
            typeZ = field;
        }
    }
    if (offsetX == -1 || offsetY == -1 || offsetZ == -1) {
        log_error("scale_pointcloud: can not extract offset of x, y, or z of pointcloud \"" + path + "\"");
        return UPNS_STATUS_ERROR;
    }
    for (  int pointBegin = 0
         ; pointBegin < edPointcloud->data.size()
         ; pointBegin += pointSize) {
        scalePoint(edPointcloud->data, pointBegin, typeX, factorX_);
        scalePoint(edPointcloud->data, pointBegin, typeY, factorY_);
        scalePoint(edPointcloud->data, pointBegin, typeZ, factorZ_);
    }

    ed->setData(edPointcloud);

    return UPNS_STATUS_OK;
}

upns::StatusCode operateScalePointclouds(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"target" : ...,
     *  <float>"factor-x" : ...,
     *  <float>"factor-y" : ...,
     *  <float>"factor-z" : ...
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    CheckoutRaw* checkout = env->getCheckout();

    if ( ! params.contains("target") || ! params["target"].isString() ) {
        log_error("scale_pointcloud: parameter \"target\" is not set or not a string");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string target = params["target"].toString().toStdString();
    if ( ! params.contains("factor-x") || ! params["factor-x"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-x\" is not set or not a float");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorX_ = params["factor-x"].toDouble();
    if ( ! params.contains("factor-y") || ! params["factor-y"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-y\" is not set or not a float");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorY_ = params["factor-y"].toDouble();
    if ( ! params.contains("factor-z") || ! params["factor-z"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-z\" is not set or not a float");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorZ_ = params["factor-z"].toDouble();

    log_info("scale_pointcloud: executing on \"" + target + "\" with factor ("
             + std::to_string(factorX_) + ", " + std::to_string(factorY_) + ", " + std::to_string(factorZ_) + ")");

    std::shared_ptr<mapit::msgs::Entity> entity = checkout->getEntity(target);
    if (entity != nullptr) {
        return scalePointcloud(checkout, target, entity);
    } else {
        std::shared_ptr<mapit::msgs::Tree> tree = checkout->getTree(target);
        if (tree != nullptr) {
            upns::StatusCode statusSearch = UPNS_STATUS_OK;
            checkout->depthFirstSearch(
                        target
                        , depthFirstSearchAll(mapit::msgs::Tree)
                        , depthFirstSearchAll(mapit::msgs::Tree)
                        , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const upns::Path &path)
                          {
                              upns::StatusCode s = scalePointcloud(checkout, path, obj);
                              if ( upnsIsOk(s) ) {
                                  return true;
                              } else {
                                  statusSearch = s;
                                  return false;
                              }
                          }
                        , depthFirstSearchAll(mapit::msgs::Entity)
                        );
        } else {
            log_error("scale_pointcloud: \"target\" is neither a entity nor a tree");
            return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        }
    }

    return UPNS_STATUS_ERROR; // this shouldn't be reached
}

UPNS_MODULE(OPERATOR_NAME, "scale pointclouds by a factor in x, y and z", "fhac", OPERATOR_VERSION, "any", &operateScalePointclouds)
