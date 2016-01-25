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
#include <Windows.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

pcl::PointCloud<pcl::PointXYZRGBNormal> convert(std::string fn, float x, float y, float z)
{
    pcl::PCLPointCloud2 pc2;
    pcl::PLYReader reader;
    if ( reader.read(fn, pc2) < 0 )
    {
        log_error("Couldn't read file" + fn);
        return pcl::PointCloud<pcl::PointXYZRGBNormal>();
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal> c1;
    pcl::PointXYZRGBNormal p;
    pcl::fromPCLPointCloud2(pc2, c1);
    std::vector<pcl::PointXYZRGBNormal, Eigen::aligned_allocator<pcl::PointXYZRGBNormal> >::iterator iter(c1.points.begin());
    while(iter != c1.points.end())
    {
        iter->x -= x;
        iter->y -= y;
        iter->z -= z;
        ++iter;
    }
    //pcl::toPCLPointCloud2(c1, pc2);
    pcl::PLYWriter writer;
    if ( writer.write(std::string(fn + "_moved"), c1) < 0 )
    {
        std::cout << "temp";
    }
    return c1;
}
int main(int argc, char *argv[])
{
//    std::string folder("data/");
//    pcl::PointCloud<pcl::PointXYZRGBNormal> all;
//    all = convert(folder + "Aligned_FARO_Scan_072.ply", 54.81013, -121.84754, 156.20739);
//    std::cout << "done1";
//    all += convert(folder + "Aligned_FARO_Scan_074.ply", 63.65034, -79.05722, 154.72884);
//    std::cout << "done2";
//    all += convert(folder + "Aligned_FARO_Scan_075.ply", 120.12115, -74.3882, 155.28055);
//    std::cout << "done3";
//    all += convert(folder + "Aligned_FARO_Scan_077.ply", 110.1273, -16.47103, 154.30708);
//    convert(folder + "Aligned_FARO_Scan_079.ply",77.8984, 5.86538, 153.58345);
//    convert(folder + "Aligned_FARO_Scan_082.ply",26.83254, 16.58681, 152.95122);
//    convert(folder + "Aligned_FARO_Scan_084.ply",-1.73885, -35.49625, 155.65961);
//    convert(folder + "Aligned_FARO_Scan_085.ply",-13.05307, -66.50669, 157.4628);
//    convert(folder + "Aligned_FARO_Scan_086.ply",-0.12532, -68.45763, 159.28508);
//    convert(folder + "Aligned_FARO_Scan_087.ply",-7.15781, -74.05119, 157.84985);
//    convert(folder + "Aligned_FARO_Scan_088.ply",-3.37326, -104.19804, 158.26798);
//    convert(folder + "Aligned_FARO_Scan_089.ply",15.84941, -116.74751, 158.86091);


//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloud);
//    sor.setLeafSize (0.01f, 0.01f, 0.01f);
//    sor.filter (*cloud_filtered);

//    pcl::PLYWriter writer;
//    if ( writer.write(folder + "all_pointclouds.ply", all) < 0 )
//    {
//        std::cout << "temp";
//    }

    std::cout << "done";

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

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hinst, HINSTANCE, LPSTR argv, int argc)
{
//    int argc=1;
//    char *argv[] = {"temp"};
    return main(argc, &argv);
}
#endif
