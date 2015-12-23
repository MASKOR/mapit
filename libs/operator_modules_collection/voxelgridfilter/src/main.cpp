#include "module.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include "libs/mapmanager/src/mapmanager.h" //< TODO: use interface (something in include folder)!
#include "operationenvironment.h"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <boost/weak_ptr.hpp>

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    const OperationParameter* leafsizeParam = env->getParameter("leafsize");
    float leafSize;
    if(leafsizeParam != NULL && leafsizeParam->realval() != 0.0f)
    {
        leafSize = leafsizeParam->realval();
    }
    else
    {
        leafSize = 0.01f;
    }

    const OperationParameter* target = env->getParameter("target");

    if(     target == NULL
            || target->mapval()   == 0 && target->layerval()  != 0)
            //|| target->layerval()  == 0
    {
        log_error("wrong combination of target ids was set. Valid: map and layer; map, layer and entity");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    upnsSharedPointer<Map> map = env->mapServiceVersioned()->getMap(target->mapval());
    if(map == NULL)
    {
        std::stringstream strm;
        strm << "Map not found: " << target->mapval();
        log_error(strm.str());
        return UPNS_STATUS_MAP_NOT_FOUND;
    }
    Layer* layer = NULL;
    for(int i=0; i < map->layers_size() ; ++i)
    {
        Layer *l = map->mutable_layers(i);
        if(l->id() == target->layerval())
        {
            layer = l;
            break;
        }
        else if(target->layerval() == 0 && l->type() == upns::POINTCLOUD2)
        {
            //TODO: this branch is TEMP. Do not 'randomly' choose first pcd layer but throw error. For testing only
            layer = l;
            break;
        }
    }
    if( layer == NULL )
    {
        log_error("layer was not found.");
        return UPNS_STATUS_LAYER_NOT_FOUND;
    }
    if(layer->type() != LayerType::POINTCLOUD2)
    {
        log_error("not a pointcloud layer. Can not load pointcloud into this layer.");
        return UPNS_STATUS_LAYER_TYPE_MISMATCH;
    }

    const Entity *entity = NULL;
    if(target->entityval() == 0)
    {
        // TODO: use all entities, not just first
        if( layer->entities_size() == 1)
        entity = &layer->entities(0);
    }
    else
    {
        for(int i=0; i < layer->entities_size() ; ++i)
        {
            const Entity &e = layer->entities(i);
            if(e.id() == target->entityval())
            {
                entity = &e;
                break;
            }
        }
        if( entity == NULL )
        {
            log_error("entity was not found.");
            return UPNS_STATUS_ENTITY_NOT_FOUND;
        }
    }
    assert( map->id() != 0 );
    assert( layer->id() != 0 );
    assert( entity->id() != 0 );

    upnsSharedPointer<AbstractEntityData> abstractEntityData = env->mapServiceVersioned()->getEntityData(map->id(), map->layers(0).id(), map->layers(0).entities(0).id() );
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(abstractEntityData);
    upnsPointcloud2Ptr pc2 = entityData->getData();

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2ConstPtr stdPc2( pc2.get(), [](pcl::PCLPointCloud2*){});
    sor.setInputCloud(stdPc2);
    sor.setLeafSize (leafSize, leafSize, leafSize);

    upnsPointcloud2Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    sor.filter (*cloud_filtered);
    std::stringstream strstr;
    strstr << "new pointcloudsize " << cloud_filtered->width;
    log_info( strstr.str() );

    entityData->setData(cloud_filtered);

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    OperationParameter *outTarget = out.add_params();
    outTarget->set_key("target");
    outTarget->set_mapval( map->id() );
    outTarget->set_layerval( layer->id() );
    outTarget->set_entityval( entity->id() );
    OperationParameter *outMapname = out.add_params();
    outMapname->set_key("leafsize");
    outMapname->set_realval( leafSize );
    env->setOutputDescription( out );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl voxelgrid filter on a pointcloud", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate)
