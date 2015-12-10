#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QResource>
#include <QDebug>
#include "mapsrenderviewport.h"
#include "qmlmapmanager.h"
#include "qmlmap.h"
#include "qmllayer.h"
#include "qmlentity.h"
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>

int main(int argc, char *argv[])
{
    log4cplus::PropertyConfigurator config("logging.properties");
    config.configure();
    //TODO: Use QGuiApplication when this bug iss fixed: https://bugreports.qt.io/browse/QTBUG-39437
    //QGuiApplication app(argc, argv);
    QApplication app(argc, argv);

    qmlRegisterType<MapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterType<QmlMapManager>("fhac.upns", 1, 0, "MapManager");
    qmlRegisterUncreatableType<QmlMap>("fhac.upns", 1, 0, "UpnsMap", "Please use mapmanager to create/retrieve maps");
    qmlRegisterUncreatableType<QmlLayer>("fhac.upns", 1, 0, "UpnsLayer", "Please add layers using map.addLayer()");
    qmlRegisterUncreatableType<QmlEntity>("fhac.upns", 1, 0, "UpnsEntity", "Please add entities by using layer.addEntity()");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:///qml/main.qml")));

    int result = app.exec();
    return result;
}
