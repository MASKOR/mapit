#include "module.h"
#include "libs/layertypes/pointcloud2/include/pointcloudlayer.h"
#include "libs/mapmanager/src/mapmanager.h" //< TODO: use interface (something in include folder)!
#include "operationenvironment.h"
#include "modules/versioning/checkoutraw.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include "error.h"

bool field_present(const std::string& name, const  std::vector<pcl::PCLPointField>& flist){
    for(int i=0; i<flist.size(); i++){
        if (flist[i].name == name) return true;
    }
    return false;
}

void chooseDefaultRepresentation ( const std::vector<pcl::PCLPointField>& flist )
{
    if ( field_present("rgba", flist)){
        //return new PointCloudRGBFactory<pcl::PointXYZRGBA, pcl::RGB>();
    } else if ( field_present("rgb", flist)){
            //return new PointCloudRGBFactory<pcl::PointXYZRGB, pcl::RGB>();
    } else if ( field_present("intensity", flist)){
        //return NULL; //PointCloudIFactory<pcl::PointXYZI, pcl::Intensity>;
    } else if ( field_present("label", flist)){
        //return new PointCloudLabelFactory<pcl::PointXYZ, pcl::Label>();
    }
    //return new PointCloudCRangeFactory<>("z");
}

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    const upns::OperationParameter *paramFilename = env->getParameter("filename");
    if(paramFilename == NULL)
    {
        log_error("No filename of PCD file given.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::string filename = paramFilename->strval();

    upnsPointcloud2Ptr pc2( new pcl::PCLPointCloud2);

    pcl::PCDReader reader;
    if ( reader.read(filename, *pc2) < 0 )
    {
        log_error("Couldn't read file" + filename);
        return UPNS_STATUS_FILE_NOT_FOUND;
    }

    const OperationParameter* target = env->getParameter("target");

    if(target &&
            (  target->mapval()   == 0 && target->layerval()  != 0
            || target->mapval()   == 0 && target->entityval() != 0
            || target->layerval() == 0 && target->entityval() != 0)) {
        log_error("wrong combination of target ids was set. Valid: map, layer and entity; map and layer; only map.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    upnsSharedPointer<Tree> map;
    if(!target || target->mapval() == 0)
    {
        const OperationParameter* mapnameParam = env->getParameter("mapname");
        std::string mapname;
        if(mapnameParam != NULL && !mapnameParam->strval().empty())
        {
            mapname = mapnameParam->strval();
        }
        else
        {
            std::string basename(filename);
            const size_t last_slash_idx = basename.find_last_of("\\/");
            basename.erase(0, last_slash_idx + 1);
            const size_t period_idx = basename.rfind('.');
            if (std::string::npos != period_idx)
            {
                basename.erase(period_idx);
            }
            std::stringstream strm;
            strm << "pcd_" << basename;
            mapname = strm.str();
        }
        map = upnsSharedPointer<Tree>(new Tree);
        env->getCheckout()->storeTree("//TODO/PATH",map);
        // TODO: update parent
    }
    else
    {
        map = env->getCheckout()->getTree(target->objectid());
    }
    if(map == NULL)
    {
        std::stringstream strm;
        strm << "Map not found: " << target->mapval();
        log_error(strm.str());
        return UPNS_STATUS_MAP_NOT_FOUND;
    }
//    Tree* layer = NULL;
//    if(!target || target->layerval() == 0)
//    {
//        layer = map->();
//        layer->set_type(LayerType::POINTCLOUD2);
//        const OperationParameter* usageParam = env->getParameter("usage");
//        std::string usage( usageParam?usageParam->strval():"" );
//        std::transform(usage.begin(), usage.end(), usage.begin(), ::tolower);
//        //TODO: this does not really work. Use strict protobuf?!
//        // e.g. Custom layer usages not supported
//        if(usage == "laser")
//        {
//            layer->set_usagetype(LayerUsageType::LASER);
//        }
//        else if(usage == "radar")
//        {
//            layer->set_usagetype(LayerUsageType::RADAR);
//        }
//        else if(usage == "navigation")
//        {
//            layer->set_usagetype(LayerUsageType::NAVIGATION);
//        }
//        else if(usage == "annotation")
//        {
//            layer->set_usagetype(LayerUsageType::ANNOTATION);
//        }
//        else if(usage.empty())
//        {
//            // supress info log
//        }
//        else
//        {
//            log_warn("Unknown Usagetype for layer: " + usage);
//        }
//    }
//    else
//    {
//        for(int i=0; i < map->layers_size() ; ++i)
//        {
//            Layer *l = map->mutable_layers(i);
//            if(l->id() == target->layerval())
//            {
//                layer = l;
//                break;
//            }
//        }
//        if( layer == NULL )
//        {
//            log_error("layer was not found.");
//            return UPNS_STATUS_LAYER_NOT_FOUND;
//        }
//        if(layer->type() != LayerType::POINTCLOUD2)
//        {
//            log_error("not a pointcloud layer. Can not load pointcloud into this layer.");
//            return UPNS_STATUS_LAYER_TYPE_MISMATCH;
//        }
//    }
//    const Entity *entity = NULL;
//    if(!target || target->entityval() == 0)
//    {
//        entity = layer->add_entities();

//        // New Ids are generated here...
//        StatusCode result = env->mapServiceVersioned()->storeMap( map );
//        if( !upnsIsOk(result) )
//        {
//            std::stringstream err;
//            err << "could not store map. " << result;
//            log_error( err.str() );
//            return result;
//        }
//    }
//    else
//    {
//        for(int i=0; i < layer->entities_size() ; ++i)
//        {
//            const Entity &e = layer->entities(i);
//            if(e.id() == target->entityval())
//            {
//                entity = &e;
//                break;
//            }
//        }
//        if( entity == NULL )
//        {
//            log_error("entity was not found.");
//            return UPNS_STATUS_ENTITY_NOT_FOUND;
//        }
//    }
//    assert( map->id() != 0 );
//    assert( layer->id() != 0 );
//    assert( entity->id() != 0 );

    upnsSharedPointer<AbstractEntityData> abstractEntityData = env->getCheckout()->getEntityDataForReadWrite( "//TODO:THE/path" );
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(abstractEntityData);
    entityData->setData( pc2 );

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    OperationParameter *outTarget = out.add_params();
    outTarget->set_key("target");
//    outTarget->set_mapval( map->id() );
//    outTarget->set_layerval( layer->id() );
//    outTarget->set_entityval( entity->id() );
    OperationParameter *outMapname = out.add_params();
    outMapname->set_key("mapname");
//    outMapname->set_strval( map->name() );
    env->setOutputDescription( out );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate)
