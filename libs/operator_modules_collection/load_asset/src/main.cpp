#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/assettype.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>

using namespace mapit::msgs;

upns::StatusCode operate_load_asset(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();


    std::string filename = params["filename"].toString().toStdString();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::shared_ptr<Entity> entity(new Entity);
    entity->set_type(AssetEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, entity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if(abstractEntitydata == NULL)
    {
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<AssetEntitydata> entityData = std::dynamic_pointer_cast<AssetEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
         // Because the Entity is stored (above) with the correct entity type, this should never happen!
        log_error("Asset has wrong type. (This should never happen)");
        return UPNS_STATUS_ERR_UNKNOWN;
    }

    std::ifstream is(filename, std::ios::binary );
    if(!is.is_open())
    {
        log_error("could not open file");
        return UPNS_STATUS_ERR_DB_NOT_FOUND;
    }
    AssetPtr plyDat(new AssetDataPair(tinyply::PlyFile(is), nullptr));
    tinyply::PlyFile *ply(&plyDat->first);
    std::vector<std::vector<int8_t> >  vecInt8;
    std::vector<std::vector<uint8_t> > vecUInt8;
    std::vector<std::vector<int16_t> >  vecInt16;
    std::vector<std::vector<uint16_t> > vecUInt16;
    std::vector<std::vector<int32_t> >  vecInt32;
    std::vector<std::vector<uint32_t> > vecUInt32;
    std::vector<std::vector<float> >  vecFloat32;
    std::vector<std::vector<double> > vecFloat64;
    for (tinyply::PlyElement &e : ply->get_elements())
    {
        std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
        if(e.properties.size() > 0)
        {
            for (auto p : e.properties)
            {
                std::vector<std::string> propertyKeys;
                propertyKeys.push_back(p.name);
                switch(p.propertyType)
                {
                case tinyply::PlyProperty::Type::INT8:
                    vecInt8.push_back(std::vector<int8_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecInt8.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::UINT8:
                    vecUInt8.push_back(std::vector<uint8_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecUInt8.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::INT16:
                    vecInt16.push_back(std::vector<int16_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecInt16.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::UINT16:
                    vecUInt16.push_back(std::vector<uint16_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecUInt16.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::INT32:
                    vecInt32.push_back(std::vector<int32_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecInt32.back(), p.isList?3:1); //TODO: Hardcoded 3 here. Only supports triangles. May be incompatible with other int-lists
                break;
                case tinyply::PlyProperty::Type::UINT32:
                    vecUInt32.push_back(std::vector<uint32_t>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecUInt32.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::FLOAT32:
                    vecFloat32.push_back(std::vector<float>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecFloat32.back(), p.isList?p.listCount:1);
                break;
                case tinyply::PlyProperty::Type::FLOAT64:
                    vecFloat64.push_back(std::vector<double>());
                    ply->request_properties_from_element(e.name, propertyKeys, vecFloat64.back(), p.isList?p.listCount:1);
                break;
                }
            }
        }
    }
    ply->read(is);
    entityData->setData(plyDat);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Asset from JSON File", "fhac", OPERATOR_VERSION, AssetEntitydata_TYPENAME, &operate_load_asset)
