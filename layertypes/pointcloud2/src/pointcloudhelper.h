#ifndef __POINTCLOUDHELPER_H__
#define __POINTCLOUDHELPER_H__

#include <pcl/PCLPointCloud2.h>

namespace upns
{

int readPointcloud2(std::istream &fs, pcl::PCLPointCloud2 &cloud);

}
#endif


