#include <Qt3DQuickExtras/qt3dquickwindow.h>
#include <Qt3DQuick/QQmlAspectEngine>
#include <QGuiApplication>
#include <QQmlEngine>
#include <QQmlContext>
#include <QQmlEngine>
#include <qqml.h>

#include "qvirtualrealityapi.h"
#include "qheadmounteddisplay.h"

#include <upns/logging.h>
#include <upns/errorcodes.h>
#include <mapit/msgs/services.pb.h>

#include <boost/program_options.hpp>

#include <upns/ui/bindings/qmlrepository.h>
#include <upns/ui/bindings/qmlcheckout.h>
#include <upns/ui/bindings/qmlcommit.h>
#include <upns/ui/bindings/qmltree.h>
#include <upns/ui/bindings/qmlentity.h>
#include <upns/ui/bindings/qmlentitydata.h>
#include <upns/ui/bindings/qmltransform.h>
#include <upns/ui/bindings/qmlbranch.h>
#include <upns/ui/bindings/qmlrepositoryserver.h>
#include <upns/ui/bindings/qmlentitydatarenderer.h>
#include <upns/ui/bindings/qmlpointcloudcoordinatesystem.h>
#include <upns/ui/models/qmlroottreemodel.h>
#include <upns/versioning/repositoryfactorystandard.h>

#include "qpointcloud.h"
#include "qpointcloudgeometry.h"
#include "qpointfield.h"
//#include <qmlraycast.h>
#include <QQmlApplicationEngine> // For Network Dialog

#include <Qt3DRender/QCameraLens> // TO DO: Temporary

#include <qmlraycast.h> // TODO: put this to visualization

namespace po = boost::program_options;



void onSceneCreated(QObject *rootItem)
{
    QQmlApplicationEngine *engine(new QQmlApplicationEngine(rootItem));

    QObject *mapitClient = rootItem->findChild<QObject *>("mapitClient");
    if(!mapitClient)
    {
        qDebug() << "Dialog has no Network Object for Visualization";
    }
    else
    {
        engine->rootContext()->setContextProperty("globalMapitClient", mapitClient);
    }
    QObject *activator = rootItem->findChild<QObject *>("activator");
    if(!mapitClient)
    {
        qDebug() << "Dialog has no debug Activator to start deferred connection";
    }
    else
    {
        engine->rootContext()->setContextProperty("activator", activator);
    }
    engine->load(QUrl(QStringLiteral("qrc:///qml/NetworkDialog.qml")));
}

int main(int argc, char* argv[])
{
    Q_INIT_RESOURCE(mapit_visualization);
    Q_INIT_RESOURCE(mapit_visualization_vr);
    upns_init_logging();
    //TODO: Use QGuiApplication when this bug is fixed: https://bugreports.qt.io/browse/QTBUG-39437
    QGuiApplication app(argc, argv);
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
    bool specified;
    std::shared_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars, &specified ) );

    Qt3DVirtualReality::QVirtualRealityApi::Type requestedVrApi(Qt3DVirtualReality::QVirtualRealityApi::OpenVR);
    bool apiAvialable = Qt3DVirtualReality::QVirtualRealityApi::isRuntimeInstalled(requestedVrApi);
    if(!apiAvialable)
    {
        qDebug() << "Vr API not available. Try to choose another VR SDK or recompile with support for other VR SDK.";
        return 1;
    }
    qDebug() << "Starting...";
    Qt3DVirtualReality::QVirtualRealityApi vrapi(requestedVrApi);
    Qt3DVirtualReality::QHeadMountedDisplayFormat fmt;
    Qt3DVirtualReality::QHeadMountedDisplay *hmd(vrapi.getHmd(0, fmt));
    if( hmd == nullptr ) {
        qDebug() << "Head Mounted disply could not be initialized";
        return 1;
    }
    // Expose the head mounted display as a context property so we can set the aspect ratio
    hmd->engine()->qmlEngine()->rootContext()->setContextProperty("_hmd", hmd);



    qmlRegisterType<QmlPointcloudCoordinatesystem>("fhac.upns", 1, 0, "PointcloudCoordinatesystem");
//    qmlRegisterType<FileIO, 1>("FileIO", 1, 0, "FileIO");

    QmlRepository *exampleRepo = new QmlRepository(repo, hmd->engine()->qmlEngine()->rootContext());
    hmd->engine()->qmlEngine()->rootContext()->setContextProperty("globalRepository", exampleRepo);
    hmd->engine()->qmlEngine()->rootContext()->setContextProperty("globalRepositoryExplicitlySpecifiedCommandline", specified);

    qmlRegisterType<Qt3DRender::QCameraLens>("qt3d.render_", 1, 0, "CameraLens_");

    qmlRegisterType<QmlEntitydataRenderer>("fhac.upns", 1, 0, "EntitydataRenderer");
    qmlRegisterUncreatableType<QPointcloud>("pcl", 1, 0, "Pointcloud", "Please use factory method (not yet available).");
    qmlRegisterType<QPointcloudGeometry>("pcl", 1, 0, "PointcloudGeometry");
    qmlRegisterUncreatableType<QPointfield>("pcl", 1, 0, "Pointfield", "Please use factory method (not yet available).");


    qmlRegisterType<QmlTransform>("fhac.upns", 1, 0, "TfTransform");
    qmlRegisterUncreatableType<QmlStamp>("fhac.upns", 1, 0, "MapitStamp", "Can not use MapitTime in Qml because it uses bytes of long unsigned int which are not available in script.");

    qmlRegisterType<QmlRaycast>("fhac.upns", 1, 0, "Raycast");

    qmlRegisterType<QmlRepository>("fhac.upns", 1, 0, "Repository");
    qmlRegisterType<QmlCheckout>("fhac.upns", 1, 0, "Checkout");
    qmlRegisterType<QmlCommit>("fhac.upns", 1, 0, "Commit");
    qmlRegisterType<QmlTree>("fhac.upns", 1, 0, "Tree");
    qmlRegisterType<QmlEntity>("fhac.upns", 1, 0, "Entity");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "Entitydata");
    qmlRegisterType<QmlBranch>("fhac.upns", 1, 0, "Branch");
    qmlRegisterType<QmlRepositoryServer>("fhac.upns", 1, 0, "RepositoryServer");

    QObject::connect(hmd, &Qt3DVirtualReality::QHeadMountedDisplay::sceneCreated, &onSceneCreated);
    hmd->setSource(QUrl("qrc:///qml/main.qml"));

    hmd->run();
    return app.exec();
}
