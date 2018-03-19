/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Qt3DQuickExtras/qt3dquickwindow.h>
#include <Qt3DQuick/QQmlAspectEngine>
#include <QGuiApplication>
#include <QQmlEngine>
#include <QQmlContext>
#include <QQmlEngine>
#include <qqml.h>

#include "qvirtualrealityapi.h"
#include "qheadmounteddisplay.h"

#include <mapit/logging.h>
#include <mapit/errorcodes.h>
#include <mapit/msgs/services.pb.h>

#include <boost/program_options.hpp>

#include <mapit/ui/bindings/qmlrepository.h>
#include <mapit/ui/bindings/qmlworkspace.h>
#include <mapit/ui/bindings/qmlcommit.h>
#include <mapit/ui/bindings/qmltree.h>
#include <mapit/ui/bindings/qmlentity.h>
#include <mapit/ui/bindings/qmlentitydata.h>
#include <mapit/ui/bindings/qmltransform.h>
#include <mapit/ui/bindings/qmlbranch.h>
#include <mapit/ui/bindings/qmlrepositoryserver.h>
#include <mapit/ui/bindings/qmlentitydatarenderer.h>
#include <mapit/ui/bindings/qmlpointcloudcoordinatesystem.h>
#include <mapit/ui/models/qmlroottreemodel.h>
#include <mapit/versioning/repositoryfactorystandard.h>

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
    mapit_init_logging();
    //TODO: Use QGuiApplication when this bug is fixed: https://bugreports.qt.io/browse/QTBUG-39437
    QGuiApplication app(argc, argv);
    app.setOrganizationName("Fachhochschule Aachen");
    app.setOrganizationDomain("fh-aachen.de");
    app.setApplicationName("Mapit Viewer");

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + "");
    program_options_desc.add_options()
            ("help,h", "print usage");

    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);
    bool specified;
    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars, &specified ) );

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



    qmlRegisterType<QmlPointcloudCoordinatesystem>("fhac.mapit", 1, 0, "PointcloudCoordinatesystem");
//    qmlRegisterType<FileIO, 1>("FileIO", 1, 0, "FileIO");

    QmlRepository *exampleRepo = new QmlRepository(repo, hmd->engine()->qmlEngine()->rootContext());
    hmd->engine()->qmlEngine()->rootContext()->setContextProperty("globalRepository", exampleRepo);
    hmd->engine()->qmlEngine()->rootContext()->setContextProperty("globalRepositoryExplicitlySpecifiedCommandline", specified);

    qmlRegisterType<Qt3DRender::QCameraLens>("qt3d.render_", 1, 0, "CameraLens_");

    qmlRegisterType<QmlEntitydataRenderer>("fhac.mapit", 1, 0, "EntitydataRenderer");
    qmlRegisterUncreatableType<QPointcloud>("pcl", 1, 0, "Pointcloud", "Please use factory method (not yet available).");
    qmlRegisterType<QPointcloudGeometry>("pcl", 1, 0, "PointcloudGeometry");
    qmlRegisterUncreatableType<QPointfield>("pcl", 1, 0, "Pointfield", "Please use factory method (not yet available).");


    qmlRegisterType<QmlTransform>("fhac.mapit", 1, 0, "TfTransform");
    qmlRegisterUncreatableType<QmlStamp>("fhac.mapit", 1, 0, "MapitStamp", "Can not use MapitTime in Qml because it uses bytes of long unsigned int which are not available in script.");

    qmlRegisterType<QmlRaycast>("fhac.mapit", 1, 0, "Raycast");

    qmlRegisterType<QmlRepository>("fhac.mapit", 1, 0, "Repository");
    qmlRegisterType<QmlWorkspace>("fhac.mapit", 1, 0, "Workspace");
    qmlRegisterType<QmlCommit>("fhac.mapit", 1, 0, "Commit");
    qmlRegisterType<QmlTree>("fhac.mapit", 1, 0, "Tree");
    qmlRegisterType<QmlEntity>("fhac.mapit", 1, 0, "Entity");
    qmlRegisterType<QmlEntitydata>("fhac.mapit", 1, 0, "Entitydata");
    qmlRegisterType<QmlBranch>("fhac.mapit", 1, 0, "Branch");
    qmlRegisterType<QmlRepositoryServer>("fhac.mapit", 1, 0, "RepositoryServer");

    QObject::connect(hmd, &Qt3DVirtualReality::QHeadMountedDisplay::sceneCreated, &onSceneCreated);
    hmd->setSource(QUrl("qrc:///qml/main.qml"));

    hmd->run();
    return app.exec();
}
