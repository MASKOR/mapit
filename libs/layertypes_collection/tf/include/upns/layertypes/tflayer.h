#ifndef TFLAYER_H
#define TFLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>

#include <Eigen/Geometry>
#include <mapit/time/time.h>
#include <list>
#include <upns/logging.h>

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

namespace upns {
namespace tf {
  const std::string _DEFAULT_LAYER_NAME_STATIC_ = "tf_static";
  const std::string _DEFAULT_LAYER_NAME_DYNAMIC_ = "tf_dynamic";
  struct Transform {
    Eigen::Translation3f translation;
    Eigen::Quaternionf rotation;

    static Transform Identity()
    {
      Transform t;
      t.translation = t.translation.Identity();
      t.rotation.setIdentity();
      return t;
    }
  };
  typedef std::shared_ptr<Transform> TransformPtr;

  struct TransformStamped {
    std::string frame_id;
    std::string child_frame_id;
    mapit::time::Stamp stamp;

    Transform transform;
  };
  typedef std::shared_ptr<TransformStamped> TransformStampedPtr;

  namespace store {
    /**
     * @brief The TransformStampedList class
     * the class to group several transforms in one entity
     * each transform in this group has to have the same frame_id and child_frame_id
     */
    class TransformStampedList {
    public:
      /**
       * @brief TransformStampedList creates a container for several transforms to be stored in one entity
       * @param frame_id       the frame_id for all transforms that are stored within this list
       * @param child_frame_id this child_fram_id for all transforms that are stored within this list
       */
      TransformStampedList(std::string frame_id, std::string child_frame_id)
        : frame_id_(frame_id)
        , child_frame_id_(child_frame_id)
      {
        transforms_ = std::move( std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>>( new std::list<std::unique_ptr<upns::tf::TransformStamped>>() ) );
      }

      /**
       * @brief get_entity_name ONLY USE THIS WHEN YOU DON'T HAVE AND CAN'T GET AN INSTANCE OF THIS CLASS!
       *                        why do you need this without an instance anyway?
       * @param frame_id
       * @param child_frame_id
       * @return
       */
      static std::string get_entity_name(std::string frame_id, std::string child_frame_id)
      {
        std::string out = frame_id + "_____" + child_frame_id;
        std::replace( out.begin(), out.end(), '/', '_');
        return out;
      }

      /**
       * @brief get_entity_name generates the name to be used following the naming convention
       *                        "frame_id + _____ + child_frame_id"
       * @return
       */
      std::string get_entity_name()
      {
        return get_entity_name(frame_id_, child_frame_id_);
      }

      /**
       * @brief add_TransformStamped adds a new transform to this container
       *                             frame_id and child_frame_id need to match the values of this container
       * @param in
       * @return error code (1 fault, 0 ok)
       */
      int add_TransformStamped(std::unique_ptr<upns::tf::TransformStamped> in)
      {
        if (transforms_ == nullptr) { return 1; }
        if (frame_id_ != in->frame_id) {
          log_error("tflayer: can't add transform, frame_id is not equal as of container");
          return 1;
        }
        if (child_frame_id_ != in->child_frame_id) {
          log_error("tflayer: can't add transform, child_frame_id is not equal as of container");
          return 1;
        }

        transforms_->push_back( std::move( in ) );

        return 0;
      }

      /**
       * @brief dispose returns the list of the container, this can not be used afterwards
       * @return
       */
      std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>>
      dispose()
      {
        if (transforms_ == nullptr) { return nullptr; }
        std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> tmp
            = std::move( transforms_ );
        transforms_ = nullptr;
        return std::move( tmp );
      }

    private:
      std::string frame_id_;
      std::string child_frame_id_;
      std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> transforms_;
    };
  }
}
}

class TfEntitydata : public Entitydata<tf::store::TransformStampedList>
{
public:
  static const char* TYPENAME();

  TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

  const char*      type() const;
  bool             hasFixedGrid() const;
  bool             canSaveRegions() const;

  int              setData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           std::shared_ptr<tf::store::TransformStampedList> &data,
                           int lod = 0);
  int              setData(std::shared_ptr<tf::store::TransformStampedList> &data, int lod = 0);

  std::shared_ptr<tf::store::TransformStampedList> getData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           bool clipMode,
                           int lod = 0);
  std::shared_ptr<tf::store::TransformStampedList> getData(int lod = 0);

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
  std::shared_ptr<tf::store::TransformStampedList> m_transforms;

};

#endif
