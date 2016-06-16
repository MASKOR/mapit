#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QResource>
#include <QDebug>
#include "bindings/qmlmapsrenderviewport.h"
#include "bindings/qmlentitydata.h"
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>
#include "stubs/qmlstubentitydatapointcloud2.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_pointcloud.h>
#include <math.h>
#include <pcl/octree/octree_impl.h>
#include <QDir>
#include "controls/xboxcontroller.h"
#include "bindings/renderdata.h"
#include "bindings/qmlrepository.h"
#include "bindings/qmlcheckout.h"
#include "bindings/qmlcommit.h"
#include "bindings/qmltree.h"
#include "bindings/qmlentity.h"
#include "bindings/qmlentitydata.h"
#include "bindings/qmlbranch.h"
#include "qpointcloud.h"
#include "qpointcloudgeometry.h"
#include "qpointfield.h"
#include "bindings/qmlentitydatarenderer.h"
#include "models/qmlroottreemodel.h"

int main(int argc, char *argv[])
{
    log4cplus::PropertyConfigurator config("logging.properties");
    config.configure();
    //TODO: Use QGuiApplication when this bug is fixed: https://bugreports.qt.io/browse/QTBUG-39437
    QApplication app(argc, argv);

    qmlRegisterType<QmlMapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterUncreatableType<Renderdata>("fhac.upns", 1, 0, "Renderdata", "Can not create Renderdata");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "EntityData");
    //qmlRegisterType<QmlEntitydataPointcloud2>("fhac.upns", 1, 0, "EntityDataPointcloud2");
    qmlRegisterType<XBoxController>("fhac.upns", 1, 0, "XBoxController");
    //qmlRegisterUncreatableType<QmlEntity>("fhac.upns", 1, 0, "UpnsEntity", "Please add entities by using layer.addEntity()");

    qmlRegisterType<QmlRepository>("fhac.upns", 1, 0, "Repository");
    qmlRegisterType<QmlCheckout>("fhac.upns", 1, 0, "Checkout");
    qmlRegisterType<QmlCommit>("fhac.upns", 1, 0, "Commit");
    qmlRegisterType<QmlTree>("fhac.upns", 1, 0, "Tree");
    qmlRegisterType<QmlEntity>("fhac.upns", 1, 0, "Entity");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    qmlRegisterType<QmlBranch>("fhac.upns", 1, 0, "Branch");

    qmlRegisterType<QmlRootTreeModel>("fhac.upns", 1, 0, "RootTreeModel");

    qmlRegisterType<QmlEntitydataRenderer>("fhac.upns", 1, 0, "EntitydataRenderer");
    qmlRegisterUncreatableType<QPointcloud>("pcl", 1, 0, "Pointcloud", "Please use factory method (not yet available).");
    qmlRegisterType<QPointcloudGeometry>("pcl", 1, 0, "PointcloudGeometry");
    qmlRegisterUncreatableType<QPointfield>("pcl", 1, 0, "Pointfield", "Please use factory method (not yet available).");
    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:///qml/main.qml")));

    int result = app.exec();
    return result;
}

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hinst, HINSTANCE, LPSTR argv, int argc)
{
//    int argc=1;
//    char *argv[] = {"temp"};
    return main(argc, &argv);
}
#endif
