#ifndef __OPERATION_H
#define __OPERATION_H

#include "upns.h"
#include "layerdata.h"
#include "upns_interface/services.pb.h"

namespace upns
{

class OperationEnvironment
{
    const OperationDescription m_operationDesc;
public:
    OperationEnvironment( const OperationDescription& desc);
};

OperationEnvironment::OperationEnvironment(const OperationDescription &desc)
    :m_operationDesc( desc )
{
}

}
#endif
