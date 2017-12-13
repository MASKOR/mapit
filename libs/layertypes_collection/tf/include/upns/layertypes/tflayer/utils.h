#ifndef TFLAYER_UTILS_H
#define TFLAYER_UTILS_H

#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/errorcodes.h>

#include <Eigen/Geometry>

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
      TransformStampedList(std::string frame_id, std::string child_frame_id, bool is_static);

      /**
       * @brief get_entity_name ONLY USE THIS WHEN YOU DON'T HAVE AND CAN'T GET AN INSTANCE OF THIS CLASS!
       *                        why do you need this without an instance anyway?
       * @param frame_id
       * @param child_frame_id
       * @return
       */
      static std::string
      get_entity_name(std::string frame_id, std::string child_frame_id)
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
      std::string
      get_entity_name();

      /**
       * @brief get_frame_id returns the frame id of the transforms contained in the list
       * @return
       */
      std::string
      get_frame_id();

      /**
       * @brief get_child_frame_id returns the child frame id of the transforms contained in the list
       * @return
       */
      std::string
      get_child_frame_id();

      /**
       * @brief get_is_static returns whether the transforms contained in the list are static or not
       * @return
       */
      bool
      get_is_static();

      /**
       * @brief get_stamp_earliest returnes the stamp of the earliest data in the list
       * @return
       */
      mapit::time::Stamp
      get_stamp_earliest();

      /**
       * @brief add_TransformStamped adds a new transform to this container
       *                             frame_id and child_frame_id need to match the values of this container
       * @param in
       * @return error code (1 fault, 0 ok)
       */
      int
      add_TransformStamped(std::unique_ptr<upns::tf::TransformStamped> in, bool is_static);

      /**
       * @brief delete_TransformStamped delete the transforms inbetween the time from start to end
       * @param start
       * @param end
       * @return error code (1 fault, 0 ok)
       */
      int
      delete_TransformStamped(const mapit::time::Stamp& start, const mapit::time::Stamp& end);

      /**
       * @brief dispose returns the list of the container, this can not be used afterwards
       * @return
       */
      std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>>
      dispose();

    private:
      mapit::time::Stamp stamp_earliest_;
      std::string frame_id_;
      std::string child_frame_id_;
      bool is_static_;
      std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> transforms_;
    };

    /**
     * @brief The TransformStampedListGatherer class
     * a class to gather all transforms before written to the storage
     */
    class TransformStampedListGatherer {
    public:
      TransformStampedListGatherer();

      void
      add_transform(std::unique_ptr<TransformStamped> tf , bool is_static);

      upns::StatusCode
      store_entities(CheckoutRaw* checkout, const std::string& prefix);
    private:
      std::shared_ptr<std::map<std::string, std::shared_ptr<TransformStampedList>>> tfs_map_;
    };
  }
}
}
#endif
