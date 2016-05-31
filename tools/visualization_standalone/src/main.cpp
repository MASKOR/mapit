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
//    std::string prefix = "data/fh/";
//    const char* files[]={"000000.pcd", "000001.pcd", "000002.pcd", "000003.pcd", "000004.pcd", "000005.pcd",
//                       "000006.pcd", "000007.pcd", "000008.pcd", "000009.pcd", "000010.pcd", "000011.pcd"};
//    pcl::PCLPointCloud2 pc2;
//    pcl::PCDReader reader;
//    pcl::PointCloud<pcl::PointXYZRGB> c1all;
//    pcl::PointCloud<pcl::PointXYZRGB> c1;
//    for(int i=0 ; i<sizeof(files)/sizeof(files[0]) ; ++i)
//    {
//        if ( reader.read(prefix + files[i], pc2) < 0 )
//        {
//            log_error("Couldn't read file0");
//            return 1;
//        }
//        pcl::fromPCLPointCloud2(pc2, c1);
//        c1all += c1;
//        std::cout << files[i] << " added." << std::endl;
//    }

//    pcl::PCDWriter writer;
//    if ( writer.writeBinary(std::string("data/fh/all2.pcd"), c1all) < 0 )
//    {
//        std::cout << "temp";
//    }
//    std::cout << "done" << std::endl;
//    return 0;

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
//////////
//    std::string fn("data/fh/all_pointclouds20_norm_flipped.pcd");
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    pcl::PCLPointCloud2 pc2;
//    pcl::PCDReader reader;
//    std::cout << "start reading " << fn << std::endl;
//    reader.read(fn, pc2);
//    std::cout << "read " << std::endl;
//    pcl::fromPCLPointCloud2(pc2, *c1);
//    std::cout << "pc1 " << std::endl;
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid (*c1, centroid);
//    centroid[0] = 0.0f;
//    centroid[1] = 0.0f;
//    centroid[2] = 0.0f;
//    std::vector<pcl::PointXYZRGBNormal, Eigen::aligned_allocator<pcl::PointXYZRGBNormal> >::iterator iter(c1->points.begin());
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    float dist=0;
//    int max = 500000;
//    while(iter != c1->points.end())
//    {
//        max--;
//        if(max == 0) break;
//        pcl::PointXYZRGBNormal p;
//        p.x = iter->x - centroid[0];
//        p.y = iter->z - centroid[2]; // swap y z
//        p.z = iter->y - centroid[1];
//        p.normal_x = iter->normal_x;
//        p.normal_y = iter->normal_z;
//        p.normal_z = iter->normal_y;
////        float newdist = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
////        if(newdist > 200.0f)
////        {
////            ++iter;
////            continue;
////        }
////        dist = std::max(dist, newdist);
//        p.rgba = iter->rgba;
//        c2->push_back(p);
//        ++iter;
//    }
//    std::cout << "dist: " << dist;
//    std::cout << "moved " << std::endl;
//    pcl::PCDWriter writer;
//    if ( writer.writeBinary(std::string(fn + "_flipped_mini"), *c2) < 0 )
//    {
//        std::cout << "temp";
//    }
//    std::cout << "written " << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_part(new pcl::PointCloud<pcl::PointXYZRGB>);

//    pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB> pOctree (20.0);
//    pOctree.setInputCloud(c2);
//    pOctree.addPointsFromInputCloud();

//    pcl::octree::OctreePointCloud< pcl::PointXYZRGB >::LeafNodeIterator pOctreeIterator(&pOctree);

//    pcl::IndicesPtr indexVector (new std::vector<int>);

//    while (pOctreeIterator != pOctree.leaf_end())
//    {
//        std::cout << ".";
//        std::cout.flush();
//        indexVector->clear();
//        pOctreeIterator.getLeafContainer().getPointIndices(*indexVector);
//        //pOctreeIterator.getData(*indexVector);

//        pcl::VoxelGrid<pcl::PointXYZRGB> pGrid;
//        //pGrid.setLeafSize(0.007, 0.007, 0.007); // 0.007 is possible and leads to 1.8gig cache file. When loaded, right eye blinks
//        pGrid.setLeafSize(0.02, 0.02, 0.02);

//        pGrid.setInputCloud(c2);
//        pGrid.setIndices(indexVector);

//        pGrid.filter(*filtered_part);
//        *filtered += *filtered_part;
//        ++pOctreeIterator;
//    }

////    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
////    sor.setInputCloud (c1);
////    sor.setLeafSize (0.20f, 0.20f, 0.20f);
////    sor.filter (*filtered);

//    std::cout << "voxed" << std::endl;
//    if ( writer.writeBinary(std::string(fn + "_voxed20"), *filtered) < 0 )
//    {
//        std::cout << "temp";
//    }
//    std::cout << "2" << std::endl;
//    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//    ne.setInputCloud (filtered);

//    // Create an empty kdtree representation, and pass it to the normal estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//    ne.setSearchMethod (tree);

//    // Output datasets
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

//    // Use all neighbors in a sphere of radius 3cm
//    ne.setRadiusSearch (1.1);

//    // Compute the features
//    ne.compute (*cloud_normals);
//    std::cout << "got normals " << std::endl;
//    pcl::PointCloud<pcl::PointXYZRGBNormal> out;
//    pcl::concatenateFields (*filtered, *cloud_normals, out);
//    std::cout << "concated " << std::endl;
//    std::cout << "done writing 1.";
//    if ( writer.writeBinary("data/fh/all_pointclouds20_norm2.pcd", out) < 0 )
//    {
//        std::cout << "temp";
//    }

//    std::cout << "fin";
//    return 0;
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
