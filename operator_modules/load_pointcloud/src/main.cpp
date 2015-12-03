#include "module.h"
#include <iostream>

int operate(upns::OperationEnvironment* layer)
{
    std::cout << "Works";
    return 0;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate)
