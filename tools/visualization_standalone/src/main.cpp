#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QQmlContext>
#include <QResource>
#include <QDebug>
#include <upns/ui/bindings/qmlmapsrenderviewport.h>
#include <upns/ui/bindings/qmlentitydata.h>
#include <upns/logging.h>
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
#include <upns/ui/bindings/renderdata.h>
#include <upns/ui/bindings/qmlrepository.h>
#include <upns/ui/bindings/qmlcheckout.h>
#include <upns/ui/bindings/qmlcommit.h>
#include <upns/ui/bindings/qmltree.h>
#include <upns/ui/bindings/qmlentity.h>
#include <upns/ui/bindings/qmlentitydata.h>
#include <upns/ui/bindings/qmltransform.h>
#include <upns/ui/bindings/qmlbranch.h>
#include "qpointcloud.h"
#include "qpointcloudgeometry.h"
#include "qpointfield.h"
#include <upns/ui/bindings/qmlentitydatarenderer.h>
#include <upns/ui/models/qmlroottreemodel.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/ui/bindings/qmlpointcloudcoordinatesystem.h>
#include <qmlraycast.h>

#include <mapit/msgs/services.pb.h>
#include <upns/errorcodes.h>

#include "inputcontrols/editorcameracontroller.h"

#include <boost/program_options.hpp>
#include "iconimageprovider.h"
#include "fileio.h"

#include <Qt3DQuick/Qt3DQuick>
#include <Qt3DInput/QInputSettings>

#include "imageupdater.h"

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    upns_init_logging();
    //TODO: Use QGuiApplication when this bug is fixed: https://bugreports.qt.io/browse/QTBUG-39437
    QApplication app(argc, argv);
    app.setOrganizationName("Fachhochschule Aachen");
    app.setOrganizationDomain("fh-aachen.de");
    app.setApplicationName("Mapit Viewer");

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + "");
    program_options_desc.add_options()
            ("help,h", "print usage");

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);

    std::shared_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    if(repo == nullptr)
    {
        log_error("Could not load Repository.");
        return 1;
    }
    qmlRegisterType<QmlRayCast>("fhac.upns", 1, 0, "RayCast");
    qmlRegisterType<QmlMapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterUncreatableType<Renderdata>("fhac.upns", 1, 0, "Renderdata", "Can not create Renderdata");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    //qmlRegisterType<QmlEntitydataPointcloud2>("fhac.upns", 1, 0, "EntitydataPointcloud2");
    qmlRegisterType<XBoxController>("fhac.upns", 1, 0, "XBoxController");
    //qmlRegisterUncreatableType<QmlEntity>("fhac.upns", 1, 0, "UpnsEntity", "Please add entities by using layer.addEntity()");

    //qmlRegisterUncreatableType<QmlRepository>("fhac.upns", 1, 0, "Repository", "Not yet implemented. Uses RAII and must be wrapped to set all program_options for repo. Use global \"globalRepository\" for now.");
    qmlRegisterType<QmlRepository>("fhac.upns", 1, 0, "Repository");
    qmlRegisterType<QmlCheckout>("fhac.upns", 1, 0, "Checkout");
    qmlRegisterType<QmlCommit>("fhac.upns", 1, 0, "Commit");
    qmlRegisterType<QmlTree>("fhac.upns", 1, 0, "Tree");
    qmlRegisterType<QmlEntity>("fhac.upns", 1, 0, "Entity");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    qmlRegisterType<QmlBranch>("fhac.upns", 1, 0, "Branch");

    qmlRegisterType<QmlTransform>("fhac.upns", 1, 0, "TfTransform");

    qmlRegisterType<QmlRootTreeModel>("fhac.upns", 1, 0, "RootTreeModel");

    qmlRegisterType<QmlEntitydataRenderer>("fhac.upns", 1, 0, "EntitydataRenderer");
    qmlRegisterUncreatableType<QPointcloud>("pcl", 1, 0, "Pointcloud", "Please use factory method (not yet available).");
    qmlRegisterType<QPointcloudGeometry>("pcl", 1, 0, "PointcloudGeometry");
    qmlRegisterUncreatableType<QPointfield>("pcl", 1, 0, "Pointfield", "Please use factory method (not yet available).");

    qmlRegisterType<EditorCameraController>("qt3deditorlib", 1, 0, "EditorCameraController");

    qmlRegisterType<QmlPointcloudCoordinatesystem>("fhac.upns", 1, 0, "PointcloudCoordinatesystem");
    qmlRegisterType<FileIO, 1>("FileIO", 1, 0, "FileIO");

    QQmlApplicationEngine engine;
    QmlRepository *exampleRepo = new QmlRepository(repo, engine.rootContext());
    engine.rootContext()->setContextProperty("globalRepository", exampleRepo);
    //IconImageProvider *imgProviderDummy = new IconImageProvider("");
    IconImageProvider *imgProviderIcon = new IconImageProvider(":/icon/");
    IconImageProvider *imgProviderMaterialDesign = new IconImageProvider(":/icon/material");
    IconImageProvider *imgProviderOperator = new IconImageProvider(":/qml/operators", false, true);
    IconImageProvider *imgProviderPrimitive = new IconImageProvider(":/icon/", false, false);
    //engine.addImageProvider("", imgProviderDummy);
    engine.addImageProvider("icon", imgProviderIcon);
    engine.addImageProvider("material", imgProviderMaterialDesign);
    engine.addImageProvider("operator", imgProviderOperator);
    engine.addImageProvider("primitive", imgProviderPrimitive);
    engine.load(QUrl(QStringLiteral("qrc:///qml/main.qml")));

    if(!engine.rootObjects().empty())
    {
        QObject *appStyle = engine.rootObjects().first()->findChild<QObject *>("appStyle");
        if(appStyle)
        {
            ImageUpdater *updater(new ImageUpdater(appStyle, &engine, appStyle));
            const QMetaObject *appStyleMeta = appStyle->metaObject();
            int isDarkIndex = appStyleMeta->indexOfProperty("isDark");
            QMetaMethod isDarkChanged = appStyleMeta->property(isDarkIndex).notifySignal();

    //        const QMetaObject *imgProvMeta = &IconImageProvider::staticMetaObject;
    //        int setDarkIconsIndex = imgProvMeta->indexOfSlot("setDarkIcons(bool)");
    //        QMetaMethod setDarkIcons = imgProvMeta->method(setDarkIconsIndex);

    //        QObject::connect(appStyle, isDarkChanged, imgProviderIcon, setDarkIcons);
    //        QObject::connect(appStyle, isDarkChanged, imgProviderMaterialDesign, setDarkIcons);
    //        QObject::connect(appStyle, isDarkChanged, imgProviderOperator, setDarkIcons);

            const QMetaObject *imgUpdMeta = &ImageUpdater::staticMetaObject;
            int updateAllIdx = imgUpdMeta->indexOfSlot("updateAllImages()");
            QMetaMethod updateAll = imgUpdMeta->method(updateAllIdx);

            QObject::connect(appStyle, isDarkChanged, updater, updateAll);
            updater->updateAllImages();
        }
        //Qt3DCore::Quick::QQmlAspectEngine * test = engine.findChild<Qt3DCore::Quick::QQmlAspectEngine *>();
        QObject *mainWindow = engine.rootObjects().first()->findChild<QObject *>("mainWindow");
        Qt3DInput::QInputSettings *inputSettings = engine.rootObjects().first()->findChild<Qt3DInput::QInputSettings *>();
        if (inputSettings) {
            inputSettings->setEventSource(mainWindow);
        } else {
            qDebug() << "No Input Settings found, keyboard and mouse events won't be handled";
        }
    }

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
