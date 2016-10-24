#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QQmlContext>
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
#include "bindings/qmlentitydatatransform.h"
#include "bindings/qmlbranch.h"
#include "qpointcloud.h"
#include "qpointcloudgeometry.h"
#include "qpointfield.h"
#include "bindings/qmlentitydatarenderer.h"
#include "models/qmlroottreemodel.h"
#include "versioning/repositoryfactory.h"

#include "services.pb.h"
#include "upns_errorcodes.h"

void createExampleRepo()
{
//    YAML::Node config = YAML::Load("mapsource:"
//                                   " name: FileSystem"
//                                   " filename: ../test.db");
    YAML::Node config = YAML::LoadFile("./repo.yaml");
    upns::Repository *repo( upns::RepositoryFactory::openLocalRepository( config ) );

    upns::upnsSharedPointer<upns::Checkout> co = repo->createCheckout("master", "testcheckout");
    if(co == NULL)
    {
        co = repo->getCheckout("testcheckout");
        if(co == NULL)
        {
            std::cout << "could not create examole checkout." << std::endl;
            return;
        }
    }
    upns::OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    desc.set_params("{\"filename\":\"data/1465223257087387.pcd\", \"target\":\"corridor/laser/eins\"}");
    log_info("Executing load_pointcloud");
    upns::OperationResult res = co->doOperation(desc);
    if(upnsIsOk(res.first))
    {
        std::cout << "success." << std::endl;
    }
    else
    {
        std::cout << "failed to execute operator." << std::endl;
    }
    desc.set_operatorname("load_pointcloud");
    desc.set_params("{\"filename\":\"data/bunny.pcd\", \"target\":\"bunny/laser/eins\"}");
    log_info("Executing load_pointcloud");
    upns::OperationResult res2 = co->doOperation(desc);
    if(upnsIsOk(res2.first))
    {
        std::cout << "success." << std::endl;
    }
    else
    {
        std::cout << "failed to execute operator." << std::endl;
    }
}

int main(int argc, char *argv[])
{
    log4cplus::PropertyConfigurator config("logging.properties");
    config.configure();
    //TODO: Use QGuiApplication when this bug is fixed: https://bugreports.qt.io/browse/QTBUG-39437
    QApplication app(argc, argv);

    createExampleRepo();

    qmlRegisterType<QmlMapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterUncreatableType<Renderdata>("fhac.upns", 1, 0, "Renderdata", "Can not create Renderdata");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    //qmlRegisterType<QmlEntitydataPointcloud2>("fhac.upns", 1, 0, "EntitydataPointcloud2");
    qmlRegisterType<XBoxController>("fhac.upns", 1, 0, "XBoxController");
    //qmlRegisterUncreatableType<QmlEntity>("fhac.upns", 1, 0, "UpnsEntity", "Please add entities by using layer.addEntity()");

    qmlRegisterType<QmlRepository>("fhac.upns", 1, 0, "Repository");
    qmlRegisterType<QmlCheckout>("fhac.upns", 1, 0, "Checkout");
    qmlRegisterType<QmlCommit>("fhac.upns", 1, 0, "Commit");
    qmlRegisterType<QmlTree>("fhac.upns", 1, 0, "Tree");
    qmlRegisterType<QmlEntity>("fhac.upns", 1, 0, "Entity");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    qmlRegisterType<QmlBranch>("fhac.upns", 1, 0, "Branch");

    qmlRegisterType<QmlEntitydataTransform>("fhac.upns", 1, 0, "EntitydataTransform");

    qmlRegisterType<QmlRootTreeModel>("fhac.upns", 1, 0, "RootTreeModel");

    qmlRegisterType<QmlEntitydataRenderer>("fhac.upns", 1, 0, "EntitydataRenderer");
    qmlRegisterUncreatableType<QPointcloud>("pcl", 1, 0, "Pointcloud", "Please use factory method (not yet available).");
    qmlRegisterType<QPointcloudGeometry>("pcl", 1, 0, "PointcloudGeometry");
    qmlRegisterUncreatableType<QPointfield>("pcl", 1, 0, "Pointfield", "Please use factory method (not yet available).");
    QQmlApplicationEngine engine;
    QmlRepository *exampleRepo = new QmlRepository(engine.rootContext());
    exampleRepo->setConf("./repo.yaml");
    engine.rootContext()->setContextProperty("exampleRepo", exampleRepo);
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
