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

int main(int argc, char *argv[])
{
    log4cplus::PropertyConfigurator config("logging.properties");
    config.configure();
    //TODO: Use QGuiApplication when this bug iss fixed: https://bugreports.qt.io/browse/QTBUG-39437
    //QGuiApplication app(argc, argv);
    QApplication app(argc, argv);

    qmlRegisterType<QmlMapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterType<QmlEntitydata>("fhac.upns", 1, 0, "EntityData");
    qmlRegisterType<QmlEntitydataPointcloud2>("fhac.upns", 1, 0, "EntityDataPointcloud2");
    //qmlRegisterUncreatableType<QmlEntity>("fhac.upns", 1, 0, "UpnsEntity", "Please add entities by using layer.addEntity()");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:///qml/main.qml")));

    int result = app.exec();
    return result;
}
