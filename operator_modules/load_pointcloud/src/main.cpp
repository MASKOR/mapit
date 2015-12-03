#include "module.h"
#include "layertypes/pointcloud2/src/pointcloudlayer.h"
#include "mapmanager/src/mapmanager.h" //< TODO: use interface (something in include folder)!
#include "operationenvironment.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>

int operate(upns::OperationEnvironment* env)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/bunny.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    const OperationParameter* targetMap = env->getParameter("targetMap");
    if(targetMap == NULL)
    {
        upnsSharedPointer<Map> map = env->mapServiceVersioned()->createMap("load_pointcloud_outputmap");
        Layer* layer = map->add_layers();
        layer->set_name("layer");
        layer->set_type(LayerType::POINTCLOUD2);
        layer->set_usagetype(LayerUsageType::LASER);
        Entity *entity = layer->add_entities();
        StatusCode result = env->mapServiceVersioned()->storeMap( map );
        assert(upnsIsOk(result));
        assert( map->id() != 0 );
        assert( map->layers(0).id() != 0);
        assert( map->layers(0).entities(0).id() != 0 );
        upnsSharedPointer<AbstractEntityData> abstractEntityData = env->mapManager()->getEntityData(map->id(), map->layers(0).id(), map->layers(0).entities(0).id() );
        upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(abstractEntityData);
        upnsPointcloud2Ptr pc2( new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud, *pc2);
        entityData->setData( pc2 );
    }

    return 0;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate)
