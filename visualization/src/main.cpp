#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QResource>
#include <QDebug>
#include "mapsrenderviewport.h"
#include "qmlmapmanager.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    qmlRegisterType<MapsRenderViewport>("fhac.upns", 1, 0, "MapsRenderViewport");
    qmlRegisterType<QmlMapManager>("fhac.upns", 1, 0, "MapManager");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:///qml/main.qml")));

    int result = app.exec();
    return result;
}

//int main(int argc, char **argv)
//{
//    QGuiApplication app(argc, argv);

//    qmlRegisterType<MapsRenderer>("fhac.upns", 1, 0, "MapsRenderer");

//    QQuickView view;
//    view.setResizeMode(QQuickView::SizeRootObjectToView);
//    view.setSource(QUrl("qrc:///qml/main.qml"));
//    view.show();

//    return app.exec();
//}
