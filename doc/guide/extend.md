---
layout: page
title: Extending
permalink: /extend/
order: 600
---

In architecture an overview is given which parts of the software can be extended.

## Writing new operators

Start by including

```cpp
#include <upns/operators/module.h>
```

in you main cpp, you can define a method with a name of your choice and this signature:

```cpp
upns::StatusCode <name_of_entry_function>(upns::OperationEnvironment* env)
```

At the end of you code, use this macro to make the library usable for mapit_core/tools:

```cpp
UPNS_MODULE(<OPERATOR_NAME>, "description of your operator",
                             "author",
                             <OPERATOR_VERSION>,
                             "reserved",
                             &<name_of_entry_function>)
```

The parameter of the function is upns::OperationEnvironment, which exposes all data you need.

- upns::OperationEnvironment::getParameters(): Returns a string that is the data given as paramters. This may be a string, the user entered in the command line. Currently all our operators use Json (json11 library) for encoding parameters.
- upns::OperationEnvironment::getCheckout(): Gives you an interface for manipulating data (CheckoutRaw).

A simple voxelgridfilter using pcl looks like this:

```cpp
#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include "json11.hpp"

upns::StatusCode operate_voxelgrid(upns::OperationEnvironment* env)
{
    //// Read Input ////
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        log_error("could not parse operator parameters as json");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    double leafSize = params["leafsize"].number_value();
    std::string target = params["target"].string_value();

    //// Validate Input ////
    if(leafSize == 0.0)
    {
        leafSize = 0.01f;
    }
    if(target.empty())
    {
        log_error("target not set");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if( ! abstractEntitydata)
    {
        log_error("target does not exist");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> entityData = std::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if( ! entityData)
    {
        log_error("target is not a pcl pointcloud");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::shared_ptr<pcl::PCLPointCloud2> pc2 = entityData->getData();
    if( ! pc2)
    {
        log_error("target pointcloud is empty");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    //// execute voxelgrid filter ////
    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    // workaround because pcl uses boost shared pointer.
    pcl::PCLPointCloud2ConstPtr stdPc2( pc2.get(), [](pcl::PCLPointCloud2*){});
    filter.setInputCloud(stdPc2);
    filter.setLeafSize (leafSize, leafSize, leafSize);

    upnsPointcloud2Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    sor.filter (*cloud_filtered);
    log_info( "new pointcloudsize " + std::to_string( cloud_filtered->width ) );

    //// set output ////
    entityData->setData(cloud_filtered);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME,
            "use pcl voxelgrid filter on a pointcloud",
            "fhac",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operate_voxelgrid)
```

We use CMake to declare the macros OPERATOR_NAME and OPERATOR_VERSION. Currently they contain "voxelgridfilter" and 0001 (integer).

Configure the linker to create an shared library. Here is an CMake example:

```cmake
project(voxelgridfilter)
set(OPERATOR_VERSION 1)
add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")
add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
find_package(json11 REQUIRED)

include_directories(${UPNS_INTERFACE_INCLUDE})

find_package(PCL REQUIRED COMPONENTS common io filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED main.cpp)
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES}
                                       ${PROTOBUF_LIBRARIES}
                                       json11
                                       mapit-msgs
                                       upns_core
                                       layertype_pointcloud2)

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)
# for 'make install' to create .../lib/libupns_operators_voxelgridfilter.so
# mapit installation searches in LD_LIBRARY_PATH for a lib with name "libupns_operators_<operatorname>.so"
# TODO: tested only for linux.
if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_OPERATORS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
```

## Writing new layertypes

Layertypes consist of a header, that can be used to develop *operators* with the layertype and a shared library that can be loaded at runtime.

The code needs to publish a method for creating an object of the layertype out of a datastream of typeless bytes.

```cpp
extern "C"
{
    MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out,
                                        std::shared_ptr<AbstractEntitydataProvider> streamProvider);
}
```

To prevent memory leaks on some systems, create a shared pointer with a custom deleter that is executed as part of the library. This is how it looks for PCL pointclouds:

```cpp
void deleteEntitydata(AbstractEntitydata *ld)
{
    PointcloudEntitydata *p = static_cast<PointcloudEntitydata*>(ld);
    delete p;
}
void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<AbstractEntitydata>(new PointcloudEntitydata( streamProvider ),
                                               deleteEntitydata);
}
```

The pointer returned by the library is of the type AbstractEntitydata, so we need to create a class deriving from AbstractEntitydatato make use of polymorphism/subtyping and casting to a concrete layertype in e.g. operators.
For convinience we do not derive from AbstractEntitydata directly, but from a templated class Entitydata. This removes some common pitfalls in the implementation and makes implementation easier.

The hard part of this task is generating the third party interface to data from an arbritary datastream. A datastream may come over the network or is read from a file, which should be transparent (meaning not important, not having an influence) to our layertype.


In the getData()-Method you can choose three forms, how the *datastream* is represented.

- file
- memory
- std::stream

This is needed, because different thridparty libraries can only interpret some of these forms. In the case of PCL there is a lack of functionality when reading PCD Format from pointers to a chunk of memory. Thus, PCL Pointcloud layertype can only use *files*.

**AbstractEntitydataProvider** can be used to obtain a filename, a pointer or a std::stream. These can then be used with the loading/storing of the thirdparty library. Entitydataprovider needs to know when a reading or writing process starts and when it ends.

Use

* **startRead** + **endRead**, **startWrite** + **endWrite**: to obtain an std::stream to read from/write to
* **startReadPointer** + **endReadPointer**, **startWritePointer** + **endWritePointer**: to obtain a chunk of memory
* **startReadFile** + **endReadFile**, **startWriteFile** + **endWriteFile**: to obtain a filename to read from / write to

There are two methods to let the layertype know, which serialization is preferred for the current operation. E.g. when reading/writing from/to network, the streaming methods would be preferred. If there is a local file, the appropriate methods for file loading would be faster.

- preferredReadType() and preferredWriteType() will give an hint, which method should ideally used. Possible return values are
    - ReadWriteStream,
    - ReadWritePointer,
    - ReadWriteFile

However, you may completely ignore this preference and only implement one of the serialization types.

## Writing tools

For writing tools use the **Repository** and **Checkout** class. Moreover GUIs can be implemented easily in Qml

### GUIs

There are fast C++ implementations for Qml Components to display an Checkout. Moreover there is a Qt3D GeometryRenderer to visualize common layertypes (like pointclouds).


To import the C++ classes and use them with the namespace ```Mapit``` the following line is needed.

```qml
import fhac.upns 1.0 as Mapit
```


There is a binding for **Repository** and **Checkout** to easyly access and manipulate both. They have no visual representation but can be used to show a list of checkouts or in combination with **RootTreeModel** to create a tree view for a checkout,

```qml
Mapit.Repository {
  id: repo
  url: "tcp://localhost:5555"
}

Mapit.Checkout {
  id: checkout
  repository: globalRepository
  name: "testcheckout"
}
```

Mapit comes with it's own TreeModel for checkouts:

```qml
Mapit.RootTreeModel {
  id: rootTreeModel
  root: checkout
}
```

This can be used to create a TreeView

```qml
TreeView {
  id: treeViewCheckout
  model: rootTreeModel
  TableViewColumn {
    role: "displayRole"
    title: "Name"
  }
  TableViewColumn {
    id: pathColumn
    role: "path"
    title: "Path"
  }
}
```

When using the new Qt3D, data from the repository can be visualized by using **Mapit.EntitydataRenderer** and **Mapit.Entitydata**.
The renderer implementes **QGeometryRenderer** and can be used as a **Component** to add geometry to an **Entity**. E.g. it can be used just like a simple **SphereMesh** in Qt3D.

```qml
Mapit.EntitydataRenderer {
  entitydata: Mapit.Entitydata {
  id: currentEntitydata
  checkout: checkout
  path: treeViewCheckout.currentIndex
        && treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole)
        === UPNS.RootTreeModel.EntityNode
          ? treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole)
          : ""
}
```
