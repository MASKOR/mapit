#ifndef __OPERATIONENVIRONMENT_H
#define __OPERATIONENVIRONMENT_H

#include "upns.h"

namespace upns
{
class MapManager;
class MapService;
class OperationDescription;
class OperationParameter;

class OperationEnvironment
{
public:
    /**
     * @brief mapManager TODO: finally remove from here. Use single interface for operations!
     * @return
     */
    virtual MapManager *mapManager() const = 0;
    /**
     * @brief mapServiceVersioned
     * This might be able to do a snapshot before the operation. afterwards it can see, what the operation did change.
     * @return
     */
    virtual MapService *mapServiceVersioned() const = 0;
    virtual const OperationDescription *getDescription() const = 0;
    virtual const OperationParameter *getParameter(std::string key) const = 0;
    virtual void setOutputDescription(OperationDescription) = 0;
    virtual const OperationDescription& outputDescription() const = 0;
};

}
#endif
