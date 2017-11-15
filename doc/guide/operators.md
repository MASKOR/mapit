---
layout: page
title: Operators
permalink: /ops/
order: 300
---

## [load_pointcloud](../operator/load_pointcloud)

## [load_tfs](../operator/load_tfs)

## [load_rosbags](../operator/load_rosbags)

#### centroid_to_origin
- Requires: PCL
- Parameters: JSON
    - target: input and output
- Effect: Removes offset from a pointcloud and tries to reposition it at the center.
- Algorithm: Calculates a axisaligned boundingbox and demeans the pointcloud using the BBs center.

#### copy

- Requires: -
- Parameters: JSON
    - target: input and output
- Effect:
- Algorithm:

#### grid (nyi)

- Requires: -
- Parameters: JSON
    - target: input and output
    - leafsize: length of the side of each grid cell
    - cells: overrides leafsize; number of cells to create
- Effect: Splitting the pointcloud into qubes, where each qube results in an new pointcloud containing the points of the original pointcloud that are within this qube.
- Algorithm: Not yet implemented

#### levelset_to_mesh

- Requires: PLY Asset, OpenVDB
- Parameters: JSON
    - target: input and output; overrides input/output
    - input: ovdb levelset entitiy
    - output: surface as PLY model
    - detail: adaptivity of the mesh; between 0.0 and 1.0 with 0.0 beeing the highest detail level.
- Effect: Creates a Polygon model out of an OpenVDB Levelset
- Algorithm: uses OpenVDB Algorithm

#### normalestimation

- Requires: PCL
- Parameters: JSON
    - target: input and output
    - radius: radius to search for neighbours in
- Effect: Generates normals for a pointcloud
- Algorithm: Uses RadiusSearch algorithm of PCL.

#### ovdb_smooth

- Requires: OpenVDB
- Parameters: JSON
    - target: input and output
    - radius: distance to blow the surface up and shrink back
    - smoothness: radius for gaussian blur
- Effect: Inflates the surface (might be usefull to close holes). Then the surface is smoothed using gaussian filter. At last the surface is deflated again to it's original volume.
- Algorithm: OpenVDB LevelSetFilter. 1) filter.offset(-radius), 2) filter.gaussian(smoothness), 3) filter.offset(radius)

#### surfrecon_openvdb

- Requires: PCL, OpenVDB
- Parameters: JSON
    - target: input and output; overrides input/output
    - input: pcl pointcloud
    - output: surface as ovdb levelset
    - radius: radius of the created spheres. should be larger than the avg. distance between points and as small as possible.
    - voxelsize:
- Effect: Reconstructs a surface from a pointcloud
- Algorithm: Creates level-set-spheres for each point

#### voxelgridfilter

- Requires:
- Parameters: JSON
    - target: input and output
    - leafsize: size of each cell
- Effect:  Thin out a pointcloud. Each voxel of size <leafsize\> will contain one or zero points.
- Algorithm:

Operators are versioned, the actually executed version is stored in metadata.
