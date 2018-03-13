---
layout: page
permalink: /operator/load_pointcloud
---

## operator/load_pointcloud

### Requires
- PCL

### Parameters (JSON)
- filename: filename from the machine, the operator is running on
- target: output entity

### Effect
loads a pointcloud

#### Remark
Algorithm: pcl::Pointcloud2 is used internally. This can represent all Pointtypes. However, operators will only work on specific point types (e.g. PointXYZNormal, PointXY). The type of Pointcloud that was read by the operator is not known by the system.
