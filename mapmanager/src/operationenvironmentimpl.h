#ifndef __OPERATIONENVIRONMENTIMPL_H
#define __OPERATIONENVIRONMENTIMPL_H
#include "operationenvironment.h"
#include "../upns_interface/services.pb.h"

namespace upns
{

class OperationEnvironmentImpl : public OperationEnvironment
{
public:
    OperationEnvironmentImpl(const OperationDescription& desc);
    void setMapManager(MapManager *mapManager);
    void setMapService(MapService *mapService);
    MapManager *mapManager() const;
    MapService *mapServiceVersioned() const;
    const OperationDescription *getDescription() const;
    const OperationParameter *getParameter(std::string key) const;
    void setOutputDescription(const OperationDescription out);
    const OperationDescription& outputDescription() const;
private:
    MapManager *m_mapManager;
    MapService *m_mapService;
    const OperationDescription m_operationDesc;
    OperationDescription m_outDesc;
};

}

#endif
