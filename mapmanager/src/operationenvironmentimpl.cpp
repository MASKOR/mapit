#include "operationenvironmentimpl.h"

namespace upns
{

OperationEnvironmentImpl::OperationEnvironmentImpl(const OperationDescription &desc)
    :m_operationDesc( desc )
{

}

void OperationEnvironmentImpl::setMapManager(MapManager *mapManager)
{
    m_mapManager = mapManager;
}

void OperationEnvironmentImpl::setMapService(MapService *mapService)
{
    m_mapService = mapService;
}

MapManager *OperationEnvironmentImpl::mapManager() const
{
    return m_mapManager;
}

MapService *OperationEnvironmentImpl::mapServiceVersioned() const
{
    return m_mapService;
}

const OperationDescription *OperationEnvironmentImpl::getDescription() const
{
    return &m_operationDesc;
}

const OperationParameter *OperationEnvironmentImpl::getParameter(std::string key) const
{
    for(int p=0; p < m_operationDesc.params_size() ; ++p)
    {
        const OperationParameter *param = &m_operationDesc.params(p);
        if(param->key() == key)
        {
            return param;
        }
    }
    return NULL;
}

}
