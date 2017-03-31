#ifndef TFLAYER_H
#define TFLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>

#include <Eigen/Geometry>

// Transform or Path (Array of transforms) in combination with a timestamp.
// {
//    tags: [tag1, tag2, ...]
//    parent: "/path/to/parent"
//    parenttimestamp: unixts
//    timestamp: unixts
//   One of the following:
//       1) reference
//    <empty, only parent and parenttimestamp are used>
//   XOR 2) tf
//    tf: [0: m00, 1: m01, m02...]
//   XOR 3) path
//    path: [
//      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
//      1: {mat: [0: m00_2, ...]},
//      2: ...
//    ]
// }
// Transforms are managed as a graph. Each transform represents a relative transform in the coordinate system of it's parent.
// To transform an entity, the graph needs to be traversed to the root and all transforms must be applied (use tf2 for that).
// Reference-Mode:
// At least a transform needs parent. If no other property is set, the transform is a reference to "parent".
// If "parent" is a path, "parenttimestamp" is used to calculate an interpolated transform.
// Transform-Mode:
// If "tf" contains a matrix, this is used as a transform. If "parent" is set, the matrix is interpreted in the
// coordinatesystem of parent.
// Path-Mode:
// The "path" property contains multiple transforms and "timestamps". Moreover "timestamp" of the transform itself is set
// and lays in the interval of all path[*].transform s. In this case the final transformation is calculated by interpolation
// from the path.
// relative:
// parent is the path to another transform
// timestamp is used to choose an appropiate coordinate system (lerp between two tfs).
// if parent is empty, target is used.
// absolute:
// only one tf (index:0) is given, parent/timestamp must not be used.
// tags: example: ICP is executed multiple times. The first time it will create a new transform, that corrects the laserscans.
// The second time it is executed, the first estimation should be approximated better. This time, ICP-operator recognices,
// there is a transform with the tag "ICP" in the tree of transforms, so this transform is changed by the operator instead
// of instering a new one.
// Tags are used to identify transforms for operators. Sometimes you want to add something to the transform tree, sometimes
// you want to set a new version of an existing transform in the tree.
// Note for operators: A reference to an transform, should always be a "tf" and a "timestamp".


using namespace upns;

// TODO: Is import case needed? (dynmic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);
MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(std::shared_ptr<AbstractEntitydata> streamProvider);
}

namespace tf {
  struct Transform {
    std::string child_frame_id;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
  };
  typedef std::shared_ptr<Transform> TransformPtr;
}

class TfEntitydata : public Entitydata<tf::Transform>
{
public:
  static const char* TYPENAME();

  TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

  const char*      type() const;
  bool             hasFixedGrid() const;
  bool             canSaveRegions() const;

  int              setData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           tf::TransformPtr &data,
                           int lod = 0);
  int              setData(tf::TransformPtr &data, int lod = 0);

  tf::TransformPtr getData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           bool clipMode,
                           int lod = 0);
  tf::TransformPtr getData(int lod = 0);

  void gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                  upnsReal &x1, upnsReal &y1, upnsReal &z1,
                  upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

  int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                           upnsReal &x2, upnsReal &y2, upnsReal &z2);

    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *&strm);

    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *&strm);

  size_t size() const;

private:
  std::shared_ptr<AbstractEntitydataProvider> m_streamProvider;
  tf::TransformPtr m_transform;

};

#endif
