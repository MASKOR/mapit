/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <upns/layertypes/tflayer/tf2/time_cache.h>
#include <upns/layertypes/tflayer/tf2/exceptions.h>
#include <upns/layertypes/tflayer/tf2/linear_math.h>

#include <upns/versioning/checkout.h>
#include <upns/operators/versioning/checkoutraw.h>

#include <assert.h>
//#include "tf2/LinearMath/Transform.h"
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>

Eigen::Translation3f
interpolate3(const Eigen::Translation3f& v0, const Eigen::Translation3f& v1, float rt);
Eigen::Vector3f
quatRotate(const Eigen::Quaternionf& rotation, const Eigen::Vector3f& v);

namespace mapit {
#if 0 // just to make autoindent happy
}
#endif
namespace tf2
{
#if 0 // just to make autoindent happy
}
#endif

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const ::tf::Transform& msg, Transform& tf2)
{

  tf2 = msg.translation * msg.rotation;
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const Transform& tf2, ::tf::Transform& msg)
{
  msg.translation = Eigen::Translation3f( tf2.translation() );
  msg.rotation = Eigen::Quaternionf( tf2.rotation() );
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const Transform& tf2, ::tf::TransformStamped& msg, mapit::time::Stamp stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(tf2, msg.transform);
  msg.stamp = stamp;
  msg.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void transformTF2ToMsg(const Eigen::Quaternionf& orient, const Eigen::Translation3f& pos, ::tf::Transform& msg)
{
  msg.translation = pos;
  msg.rotation = orient;
}

void transformTF2ToMsg(const Eigen::Quaternionf& orient, const Eigen::Translation3f& pos, ::tf::TransformStamped& msg, mapit::time::Stamp stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(orient, pos, msg.transform);
  msg.stamp = stamp;
  msg.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void setIdentity(::tf::Transform& tx)
{
  tx.translation = tx.translation.Identity();
  tx.rotation.setIdentity();
}

bool startsWithSlash(const std::string& frame_id)
{
  if (frame_id.size() > 0)
    if (frame_id[0] == '/')
      return true;
  return false;
}

std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if (startsWithSlash(in))
    out.erase(0,1);
  return out;
}


bool BufferCore::warnFrameId(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.size() == 0)
  {
    std::stringstream ss;
    ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
    log_warn( ss.str() );
    return true;
  }

  if (startsWithSlash(frame_id))
  {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
    log_warn( ss.str() );
    return true;
  }

  return false;
}

CompactFrameID BufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.empty())
  {
    std::stringstream ss;
    ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }

  if (startsWithSlash(frame_id))
  {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }

  CompactFrameID id = lookupFrameNumber(frame_id);
  if (id == 0)
  {
    std::stringstream ss;
    ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
    throw tf2::LookupException(ss.str().c_str());
  }
  
  return id;
}

BufferCore::BufferCore()
: cache_time_(mapit::time::seconds(std::numeric_limits<int>::max()))
, transformable_callbacks_counter_(0)
, transformable_requests_counter_(0)
, using_dedicated_thread_(false)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(TimeCacheInterfacePtr());
  frameIDs_reverse.push_back("NO_PARENT");
}

BufferCore::BufferCore(
          std::shared_ptr<upns::Checkout> checkout
        , std::string map_name
        , std::string layer_name_tf_static
        , std::string layer_name_tf_dynamic
        ) : BufferCore()
{
    std::shared_ptr<mapit::Map> map = checkout->getMap(map_name);
    if (map == nullptr) {
        //TODO error
        assert(false);
    }

    //TODO: Name decides on wether tfs are static or dynamic. This introduces several restrictions.
    // - if static/dynamic is a property of a tf, implementation must reside inside entitydata
    // - if it is a property of the current use-case, implementation must reside as part of "Entity"-Protobuf (next to stamp, frame_id).
    // restrictions:
    // - introduces fixed names for layertypes. These must be known to interpret the repository.
    // - making static layertypes dynamic/vice versa involves moving the entity (future: merging problems)
    // - the entity itself is not atomically defined, it always needs it's context to load it

    std::shared_ptr<mapit::Layer> layer_static = checkout->getLayer(map, layer_name_tf_static);
    if (layer_static == nullptr) {
        // TODO warning
    } else {
        setTransforms(checkout, layer_static, true);
    }
    std::shared_ptr<mapit::Layer> layer_dynamic = checkout->getLayer(map, layer_name_tf_dynamic);
    if (layer_dynamic == nullptr) {
        // TODO warning
    } else {
        setTransforms(checkout, layer_dynamic, false);
    }
}

void
BufferCore::setTransforms(std::shared_ptr<upns::Checkout> checkout, std::shared_ptr<mapit::Layer> layer, bool is_static)
{
    for (auto entity : checkout->getListOfEntities(layer)) {
        //TODO: nullpointer checks.
        std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> entity_data
            = std::dynamic_pointer_cast<TfEntitydata>(
                checkout->getEntityDataReadOnly( entity )
                )->getData()->dispose();

        for (const std::unique_ptr<upns::tf::TransformStamped>& tfs : *entity_data) {
          setTransform(*tfs, layer->getName(), is_static);
        }
    }
}

BufferCore::~BufferCore()
{

}

void BufferCore::clear()
{
  //old_tf_.clear();


  boost::mutex::scoped_lock lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector<TimeCacheInterfacePtr>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      if (*cache_it)
        (*cache_it)->clearList();
    }
  }
  
}

bool BufferCore::setTransform(const ::tf::TransformStamped& transform_in, const std::string& authority, bool is_static)
{

  /////BACKWARDS COMPATABILITY
  /* tf::StampedTransform tf_transform;
  tf::transformStampedMsgToTF(transform_in, tf_transform);
  if  (!old_tf_.setTransform(tf_transform, authority))
  {
    printf("Warning old setTransform Failed but was not caught\n");
    }*/

  /////// New implementation
  ::tf::TransformStamped stripped = transform_in;
  stripped.frame_id = stripSlash(stripped.frame_id);
  stripped.child_frame_id = stripSlash(stripped.child_frame_id);

  stripped.transform.rotation.normalize();

  bool error_exists = false;
  if (stripped.child_frame_id == stripped.frame_id)
  {
    log_error("TF_SELF_TRANSFORM: Ignoring transform from authority \"" + authority + "\" with frame_id and child_frame_id  \"" + stripped.child_frame_id + "\" because they are the same");
    error_exists = true;
  }

  if (stripped.child_frame_id == "")
  {
    log_error("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"" + authority + "\" because child_frame_id not set ");
    error_exists = true;
  }

  if (stripped.frame_id == "")
  {
    log_error("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"" + stripped.child_frame_id + "\"  from authority \"" + authority + "\" because frame_id not set");
    error_exists = true;
  }

  if (std::isnan(stripped.transform.translation.x()) || std::isnan(stripped.transform.translation.y()) || std::isnan(stripped.transform.translation.z()) ||
      std::isnan(stripped.transform.rotation.x()) ||       std::isnan(stripped.transform.rotation.y()) ||       std::isnan(stripped.transform.rotation.z()) ||       std::isnan(stripped.transform.rotation.w()))
  {
    log_error("TF_NAN_INPUT: Ignoring transform for child_frame_id \"" + stripped.child_frame_id + "\" from authority \"" + authority
           + "\" because of a nan value in the transform ("
           + std::to_string( stripped.transform.translation.x() ) + " " + std::to_string( stripped.transform.translation.y() ) + " " + std::to_string( stripped.transform.translation.z() ) + ") ("
           + std::to_string( stripped.transform.rotation.x() ) + " " + std::to_string( stripped.transform.rotation.y() ) + " " + std::to_string( stripped.transform.rotation.z() ) + " " + std::to_string( stripped.transform.rotation.w() ) + ")"
              );
    error_exists = true;
  }

  bool valid = std::abs((stripped.transform.rotation.w() * stripped.transform.rotation.w()
                       + stripped.transform.rotation.x() * stripped.transform.rotation.x()
                       + stripped.transform.rotation.y() * stripped.transform.rotation.y()
                       + stripped.transform.rotation.z() * stripped.transform.rotation.z()) - 1.0f) < 10e-6;

  if (!valid) 
  {
    log_error("TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id \"" + stripped.child_frame_id + "\" from authority \"" + authority
           + "\" because of an invalid quaternion in the transform (" + std::to_string( stripped.transform.rotation.x() ) + " " + std::to_string( stripped.transform.rotation.y() ) + " " + std::to_string( stripped.transform.rotation.z() ) + " " + std::to_string( stripped.transform.rotation.w() ) + ")"
             );
    error_exists = true;
  }

  if (error_exists)
    return false;
  
  {
    boost::mutex::scoped_lock lock(frame_mutex_);
    CompactFrameID frame_number = lookupOrInsertFrameNumber(stripped.child_frame_id);
    TimeCacheInterfacePtr frame = getFrame(frame_number);
    if (frame == NULL)
      frame = allocateFrame(frame_number, is_static);

    if (frame->insertData(TransformStorage(stripped, lookupOrInsertFrameNumber(stripped.frame_id), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    }
    else
    {
      log_warn( "TF_OLD_DATA ignoring data from the past for frame " + stripped.child_frame_id + " at time " + std::to_string( mapit::time::to_sec(stripped.stamp) ) + " according to authority " + authority + "\nPossible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained" );
      return false;
    }
  }

  testTransformableRequests();

  return true;
}

TimeCacheInterfacePtr BufferCore::allocateFrame(CompactFrameID cfid, bool is_static)
{
  TimeCacheInterfacePtr frame_ptr = frames_[cfid];
  if (is_static) {
    frames_[cfid] = TimeCacheInterfacePtr(new StaticCache());
  } else {
    frames_[cfid] = TimeCacheInterfacePtr(new TimeCache(cache_time_));
  }

  return frames_[cfid];
}

enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

// TODO for Jade: Merge walkToTopParent functions; this is now a stub to preserve ABI
template<typename F>
TF2Error BufferCore::walkToTopParent(F& f, mapit::time::Stamp time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
{
  return walkToTopParent(f, time, target_id, source_id, error_string, NULL);
}

template<typename F>
TF2Error BufferCore::walkToTopParent(F& f, mapit::time::Stamp time, CompactFrameID target_id,
    CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID>
    *frame_chain) const
{
  if (frame_chain)
    frame_chain->clear();

  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == mapit::time::Stamp())
  {
    TF2Error retval = getLatestCommonTime(target_id, source_id, time, error_string);
    if (retval != NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;

  std::string extrapolation_error_string;
  bool extrapolation_might_have_occurred = false;

  while (frame != 0)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain)
      frame_chain->push_back(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, &extrapolation_error_string);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      extrapolation_might_have_occurred = true;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  std::vector<CompactFrameID> reverse_frame_chain;

  while (frame != top_parent)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain)
      reverse_frame_chain.push_back(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      if (frame_chain)
      {
        frame_chain->swap(reverse_frame_chain);
      }
      return NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    if (extrapolation_might_have_occurred)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << extrapolation_error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
      
    }

    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);
  if (frame_chain)
  {
    // Pruning: Compare the chains starting at the parent (end) until they differ
    int m = reverse_frame_chain.size()-1;
    int n = frame_chain->size()-1;
    for (; m >= 0 && n >= 0; --m, --n)
    {
      if ((*frame_chain)[n] != reverse_frame_chain[m])
        break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      frame_chain->erase(frame_chain->begin() + (n-1), frame_chain->end());

    if (m < reverse_frame_chain.size())
    {
      for (int i = m; i >= 0; --i)
      {
        frame_chain->push_back(reverse_frame_chain[i]);
      }
    }
  }
  
  return NO_ERROR;
}



struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(1.0, 0.0, 0.0, 0.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(1.0, 0.0, 0.0, 0.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(1.0, 0.0, 0.0, 0.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCacheInterfacePtr cache, mapit::time::Stamp time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_.vector();
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_.vector();
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, mapit::time::Stamp _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        Eigen::Quaternionf inv_target_quat = target_to_top_quat.inverse();
        Eigen::Vector3f inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        Eigen::Quaternionf inv_target_quat = target_to_top_quat.inverse();
        Eigen::Vector3f inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  mapit::time::Stamp time;
  Eigen::Quaternionf source_to_top_quat;
  Eigen::Vector3f source_to_top_vec;
  Eigen::Quaternionf target_to_top_quat;
  Eigen::Vector3f target_to_top_vec;

  Eigen::Quaternionf result_quat;
  Eigen::Vector3f result_vec;
};

::tf::TransformStamped BufferCore::lookupTransform(const std::string& target_frame,
                                                   const std::string& source_frame,
                                                   const mapit::time::Stamp& time) const
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  if (target_frame == source_frame) {
    ::tf::TransformStamped identity;
    identity.frame_id = target_frame;
    identity.child_frame_id = source_frame;
    identity.transform.rotation.setIdentity();
    identity.transform.translation = identity.transform.translation.Identity();

    if (time == mapit::time::Stamp())
    {
      CompactFrameID target_id = lookupFrameNumber(target_frame);
      TimeCacheInterfacePtr cache = getFrame(target_id);
      if (cache)
        identity.stamp = cache->getLatestTimestamp();
      else
        identity.stamp = time;
    }
    else
      identity.stamp = time;

    return identity;
  }

  //Identify case does not need to be validated above
  CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
  CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);

  std::string error_string;
  TransformAccum accum;
  int retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
  if (retval != NO_ERROR)
  {
    switch (retval)
    {
    case CONNECTIVITY_ERROR:
      throw ConnectivityException(error_string);
    case EXTRAPOLATION_ERROR:
      throw ExtrapolationException(error_string);
    case LOOKUP_ERROR:
      throw LookupException(error_string);
    default:
      log_error("Unknown error code: " + std::to_string( retval ) );
      assert(0);
    }
  }

  ::tf::TransformStamped output_transform;
  transformTF2ToMsg(accum.result_quat, Eigen::Translation3f( accum.result_vec ), output_transform, accum.time, target_frame, source_frame);
  return output_transform;
}

                                                       
::tf::TransformStamped BufferCore::lookupTransform(const std::string& target_frame,
                                                        const mapit::time::Stamp& target_time,
                                                        const std::string& source_frame,
                                                        const mapit::time::Stamp& source_time,
                                                        const std::string& fixed_frame) const
{
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

  ::tf::TransformStamped output;
  ::tf::TransformStamped temp1 =  lookupTransform(fixed_frame, source_frame, source_time);
  ::tf::TransformStamped temp2 =  lookupTransform(target_frame, fixed_frame, target_time);
  
  Transform tf1, tf2;
  transformMsgToTF2(temp1.transform, tf1);
  transformMsgToTF2(temp2.transform, tf2);
  transformTF2ToMsg(tf2*tf1, output.transform);
  output.stamp = temp2.stamp;
  output.frame_id = target_frame;
  output.child_frame_id = source_frame;
  return output;
}



/*
geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const mapit::time::Stamp& time,
                                          const ros::Duration& averaging_interval) const
{
  try
  {
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, 
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const std::string& reference_frame,
                                          const tf2::Point & reference_point, 
                                          const std::string& reference_point_frame, 
                                          const mapit::time::Stamp& time,
                                          const ros::Duration& averaging_interval) const
{
  try{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}
*/

struct CanTransformAccum
{
  CompactFrameID gather(TimeCacheInterfacePtr cache, mapit::time::Stamp time, std::string* error_string)
  {
    return cache->getParent(time, error_string);
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, mapit::time::Stamp _time)
  {
  }

  TransformStorage st;
};

bool BufferCore::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                    const mapit::time::Stamp& time, std::string* error_msg) const
{
  if (target_id == 0 || source_id == 0)
  {
    return false;
  }

  if (target_id == source_id)
  {
    return true;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) == NO_ERROR)
  {
    return true;
  }

  return false;
}

bool BufferCore::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                                  const mapit::time::Stamp& time, std::string* error_msg) const
{
  boost::mutex::scoped_lock lock(frame_mutex_);
  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const mapit::time::Stamp& time, std::string* error_msg) const
{
  // Short circuit if target_frame == source_frame
  if (target_frame == source_frame)
    return true;

  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;

  boost::mutex::scoped_lock lock(frame_mutex_);

  CompactFrameID target_id = lookupFrameNumber(target_frame);
  CompactFrameID source_id = lookupFrameNumber(source_frame);

  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame, const mapit::time::Stamp& target_time,
                          const std::string& source_frame, const mapit::time::Stamp& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;
  if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
    return false;

  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
}


tf2::TimeCacheInterfacePtr BufferCore::getFrame(CompactFrameID frame_id) const
{
  if (frame_id >= frames_.size())
    return TimeCacheInterfacePtr();
  else
  {
    return frames_[frame_id];
  }
}

CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const
{
  CompactFrameID retval;
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(0);
  }
  else
    retval = map_it->second;
  return retval;
}

CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str)
{
  CompactFrameID retval = 0;
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(frames_.size());
    frames_.push_back(TimeCacheInterfacePtr());//Just a place holder for iteration
    frameIDs_[frameid_str] = retval;
    frameIDs_reverse.push_back(frameid_str);
  }
  else
    retval = frameIDs_[frameid_str];

  return retval;
}

const std::string& BufferCore::lookupFrameString(CompactFrameID frame_id_num) const
{
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];
}

void BufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                     lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}

std::string BufferCore::allFramesAsString() const
{
  boost::mutex::scoped_lock lock(frame_mutex_);
  return this->allFramesAsStringNoLock();
}

std::string BufferCore::allFramesAsStringNoLock() const
{
  std::stringstream mstream;

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCacheInterfacePtr frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->getData(mapit::time::Stamp(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

TF2Error BufferCore::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, mapit::time::Stamp & time, std::string * error_string) const
{
  // Error if one of the frames don't exist.
  if (source_id == 0 || target_id == 0) return LOOKUP_ERROR;

  if (source_id == target_id)
  {
    TimeCacheInterfacePtr cache = getFrame(source_id);
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    if (cache)
      time = cache->getLatestTimestamp();
    else
      time = mapit::time::Stamp();
    return NO_ERROR;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  mapit::time::Stamp common_time = mapit::time::Stamp::max();
  while (frame != 0)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if ( ! mapit::time::is_zero( latest.first ) )
    {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = common_time;
      if ( time == mapit::time::Stamp::max() )
      {
        time = mapit::time::Stamp();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = mapit::time::Stamp::max();
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      break;
    }

    if ( ! mapit::time::is_zero( latest.first ) )
    {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = common_time;
      if ( time == mapit::time::Stamp::max() )
      {
        time = mapit::time::Stamp();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it)
    {
      if ( ! mapit::time::is_zero( it->first ) )
      {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent)
      {
        break;
      }
    }
  }

  if ( common_time == mapit::time::Stamp::max() )
  {
    common_time = mapit::time::Stamp();
  }

  time = common_time;
  return NO_ERROR;
}

std::string BufferCore::allFramesAsYAML(double current_time) const
{
  std::stringstream mstream;
  boost::mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() ==1)
    mstream <<"[]";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);

   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    CompactFrameID cfid = CompactFrameID(counter);
    CompactFrameID frame_id_num;
    TimeCacheInterfacePtr cache = getFrame(cfid);
    if (!cache)
    {
      continue;
    }

    if(!cache->getData(mapit::time::Stamp(), temp))
    {
      continue;
    }

    frame_id_num = temp.frame_id_;

    std::string authority = "no recorded authority";
    std::map<CompactFrameID, std::string>::const_iterator it = frame_authority_.find(cfid);
    if (it != frame_authority_.end()) {
      authority = it->second;
    }

    double rate = cache->getListLength() / std::max((mapit::time::to_sec( cache->getLatestTimestamp() ) -
                                                     mapit::time::to_sec( cache->getOldestTimestamp() ) ), 0.0001);

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << frameIDs_reverse[cfid] << ": " << std::endl;
    mstream << "  parent: '" << frameIDs_reverse[frame_id_num] << "'" << std::endl;
    mstream << "  broadcaster: '" << authority << "'" << std::endl;
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: " << mapit::time::to_sec( cache->getLatestTimestamp() ) << std::endl;
    mstream << "  oldest_transform: " << mapit::time::to_sec( cache->getOldestTimestamp() ) << std::endl;
    if ( current_time > 0 ) {
      mstream << "  transform_delay: " << current_time - mapit::time::to_sec( cache->getLatestTimestamp() ) << std::endl;
    }
    mstream << "  buffer_length: " << mapit::time::to_sec( std::chrono::duration_cast<mapit::time::Duration>(cache->getLatestTimestamp() - cache->getOldestTimestamp()) ) << std::endl;
  }

  return mstream.str();
}

std::string BufferCore::allFramesAsYAML() const
{
  return this->allFramesAsYAML(0.0);
}

TransformableCallbackHandle BufferCore::addTransformableCallback(const TransformableCallback& cb)
{
  boost::mutex::scoped_lock lock(transformable_callbacks_mutex_);
  TransformableCallbackHandle handle = ++transformable_callbacks_counter_;
  while (!transformable_callbacks_.insert(std::make_pair(handle, cb)).second)
  {
    handle = ++transformable_callbacks_counter_;
  }

  return handle;
}

struct BufferCore::RemoveRequestByCallback
{
  RemoveRequestByCallback(TransformableCallbackHandle handle)
  : handle_(handle)
  {}

  bool operator()(const TransformableRequest& req)
  {
    return req.cb_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::removeTransformableCallback(TransformableCallbackHandle handle)
{
  {
    boost::mutex::scoped_lock lock(transformable_callbacks_mutex_);
    transformable_callbacks_.erase(handle);
  }

  {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    V_TransformableRequest::iterator it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByCallback(handle));
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}

TransformableRequestHandle BufferCore::addTransformableRequest(TransformableCallbackHandle handle, const std::string& target_frame, const std::string& source_frame, mapit::time::Stamp time)
{
  // shortcut if target == source
  if (target_frame == source_frame)
  {
    return 0;
  }

  TransformableRequest req;
  req.target_id = lookupFrameNumber(target_frame);
  req.source_id = lookupFrameNumber(source_frame);

  // First check if the request is already transformable.  If it is, return immediately
  if (canTransformInternal(req.target_id, req.source_id, time, 0))
  {
    return 0;
  }

  // Might not be transformable at all, ever (if it's too far in the past)
  if (req.target_id && req.source_id)
  {
    mapit::time::Stamp latest_time;
    // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if ( ! mapit::time::is_zero( latest_time ) && time + cache_time_ < latest_time)
    {
      return 0xffffffffffffffffULL;
    }
  }

  req.cb_handle = handle;
  req.time = time;
  req.request_handle = ++transformable_requests_counter_;
  if (req.request_handle == 0 || req.request_handle == 0xffffffffffffffffULL)
  {
    req.request_handle = 1;
  }

  if (req.target_id == 0)
  {
    req.target_string = target_frame;
  }

  if (req.source_id == 0)
  {
    req.source_string = source_frame;
  }

  boost::mutex::scoped_lock lock(transformable_requests_mutex_);
  transformable_requests_.push_back(req);

  return req.request_handle;
}

struct BufferCore::RemoveRequestByID
{
  RemoveRequestByID(TransformableRequestHandle handle)
  : handle_(handle)
  {}

  bool operator()(const TransformableRequest& req)
  {
    return req.request_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::cancelTransformableRequest(TransformableRequestHandle handle)
{
  boost::mutex::scoped_lock lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByID(handle));

  if (it != transformable_requests_.end())
  {
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}



// backwards compability for tf methods
boost::signals2::connection BufferCore::_addTransformsChangedListener(boost::function<void(void)> callback)
{
  boost::mutex::scoped_lock lock(transformable_requests_mutex_);
  return _transforms_changed_.connect(callback);
}

void BufferCore::_removeTransformsChangedListener(boost::signals2::connection c)
{
  boost::mutex::scoped_lock lock(transformable_requests_mutex_);
  c.disconnect();
}


bool BufferCore::_frameExists(const std::string& frame_id_str) const
{
  boost::mutex::scoped_lock lock(frame_mutex_);
  return frameIDs_.count(frame_id_str);
}

bool BufferCore::_getParent(const std::string& frame_id, mapit::time::Stamp time, std::string& parent) const
{

  boost::mutex::scoped_lock lock(frame_mutex_);
  CompactFrameID frame_number = lookupFrameNumber(frame_id);
  TimeCacheInterfacePtr frame = getFrame(frame_number);

  if (! frame)
    return false;
      
  CompactFrameID parent_id = frame->getParent(time, NULL);
  if (parent_id == 0)
    return false;

  parent = lookupFrameString(parent_id);
  return true;
};

void BufferCore::_getFrameStrings(std::vector<std::string> & vec) const
{
  vec.clear();

  boost::mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frameIDs_reverse.size(); counter ++)
  {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}




void BufferCore::testTransformableRequests()
{
  boost::mutex::scoped_lock lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it = transformable_requests_.begin();
  for (; it != transformable_requests_.end();)
  {
    TransformableRequest& req = *it;

    // One or both of the frames may not have existed when the request was originally made.
    if (req.target_id == 0)
    {
      req.target_id = lookupFrameNumber(req.target_string);
    }

    if (req.source_id == 0)
    {
      req.source_id = lookupFrameNumber(req.source_string);
    }

    mapit::time::Stamp latest_time;
    bool do_cb = false;
    TransformableResult result = TransformAvailable;
    // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if ( ! mapit::time::is_zero( latest_time ) && req.time + cache_time_ < latest_time)
    {
      do_cb = true;
      result = TransformFailure;
    }
    else if (canTransformInternal(req.target_id, req.source_id, req.time, 0))
    {
      do_cb = true;
      result = TransformAvailable;
    }

    if (do_cb)
    {
      {
        boost::mutex::scoped_lock lock2(transformable_callbacks_mutex_);
        M_TransformableCallback::iterator it = transformable_callbacks_.find(req.cb_handle);
        if (it != transformable_callbacks_.end())
        {
          const TransformableCallback& cb = it->second;
          cb(req.request_handle, lookupFrameString(req.target_id), lookupFrameString(req.source_id), req.time, result);
        }
      }

      if (transformable_requests_.size() > 1)
      {
        transformable_requests_[it - transformable_requests_.begin()] = transformable_requests_.back();
      }

      transformable_requests_.erase(transformable_requests_.end() - 1);
    }
    else
    {
      ++it;
    }
  }

  // unlock before allowing possible user callbacks to avoid potential detadlock (#91)
  lock.unlock();

  // Backwards compatability callback for tf
  _transforms_changed_();
}


std::string BufferCore::_allFramesAsDot(double current_time) const
{
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  boost::mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() == 1) {
    mstream <<"\"no tf data recieved\"";
  }
  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);

  for (unsigned int counter = 1; counter < frames_.size(); counter ++) // one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      continue;
    }
    if(!counter_frame->getData(mapit::time::Stamp(), temp)) {
      continue;
    } else {
      frame_id_num = temp.frame_id_;
    }
    std::string authority = "no recorded authority";
    std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(counter);
    if (it != frame_authority_.end())
      authority = it->second;

    double rate = counter_frame->getListLength() / std::max( mapit::time::to_sec(counter_frame->getLatestTimestamp()) -
                                                             mapit::time::to_sec(counter_frame->getOldestTimestamp()), 0.0001);

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
            << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
      //<< "Time: " << current_time.toSec() << "\\n"
            << "Broadcaster: " << authority << "\\n"
            << "Average rate: " << rate << " Hz\\n"
            << "Most recent transform: " << mapit::time::to_sec( counter_frame->getLatestTimestamp() ) <<" ";
    if (current_time > 0)
      mstream << "( "<<  current_time - mapit::time::to_sec( counter_frame->getLatestTimestamp() ) << " sec old)";
    mstream << "\\n"
      //    << "(time: " << getFrame(counter)->getLatestTimestamp().toSec() << ")\\n"
      //    << "Oldest transform: " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec() << " sec old \\n"
      //    << "(time: " << (getFrame(counter)->getOldestTimestamp()).toSec() << ")\\n"
            << "Buffer length: " << mapit::time::to_sec( std::chrono::duration_cast<mapit::time::Duration>(counter_frame->getLatestTimestamp() - counter_frame->getOldestTimestamp()) ) << " sec\\n"
            <<"\"];" <<std::endl;
  }

  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      if (current_time > 0) {
        mstream << "edge [style=invis];" <<std::endl;
        mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
                << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n "
                << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
      }
      continue;
    }
    if (counter_frame->getData(mapit::time::Stamp(), temp)) {
      frame_id_num = temp.frame_id_;
    } else {
    	frame_id_num = 0;
    }

    if(frameIDs_reverse[frame_id_num]=="NO_PARENT")
    {
      mstream << "edge [style=invis];" <<std::endl;
      mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n";
      if (current_time > 0)
        mstream << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n ";
      mstream << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}

std::string BufferCore::_allFramesAsDot() const
{
  return _allFramesAsDot(0.0);
}

void BufferCore::_chainAsVector(const std::string & target_frame, mapit::time::Stamp target_time, const std::string & source_frame, mapit::time::Stamp source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
{
  std::string error_string;

  output.clear(); //empty vector

  std::stringstream mstream;
  boost::mutex::scoped_lock lock(frame_mutex_);

  TransformAccum accum;

  // Get source frame/time using getFrame
  CompactFrameID source_id = lookupFrameNumber(source_frame);
  CompactFrameID fixed_id = lookupFrameNumber(fixed_frame);
  CompactFrameID target_id = lookupFrameNumber(target_frame);

  std::vector<CompactFrameID> source_frame_chain;
  int retval = walkToTopParent(accum, source_time, fixed_id, source_id, &error_string, &source_frame_chain);

  if (retval != NO_ERROR)
  {
    switch (retval)
    {
    case CONNECTIVITY_ERROR:
      throw ConnectivityException(error_string);
    case EXTRAPOLATION_ERROR:
      throw ExtrapolationException(error_string);
    case LOOKUP_ERROR:
      throw LookupException(error_string);
    default:
      log_error("Unknown error code: " + std::to_string( retval ));
      assert(0);
    }
  }
  if (source_time != target_time)
  {
    std::vector<CompactFrameID> target_frame_chain;
    retval = walkToTopParent(accum, target_time, target_id, fixed_id, &error_string, &target_frame_chain);

    if (retval != NO_ERROR)
    {
      switch (retval)
      {
      case CONNECTIVITY_ERROR:
        throw ConnectivityException(error_string);
      case EXTRAPOLATION_ERROR:
        throw ExtrapolationException(error_string);
      case LOOKUP_ERROR:
        throw LookupException(error_string);
      default:
        log_error("Unknown error code: " +  std::to_string( retval ));
        assert(0);
      }
    }
    int m = target_frame_chain.size()-1;
    int n = source_frame_chain.size()-1;
    for (; m >= 0 && n >= 0; --m, --n)
    {
      if (source_frame_chain[n] != target_frame_chain[m])
        break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      source_frame_chain.erase(source_frame_chain.begin() + (n-1), source_frame_chain.end());

    if (m < target_frame_chain.size())
    {
      for (unsigned int i = 0; i <= m; ++i)
      {
        source_frame_chain.push_back(target_frame_chain[i]);
      }
    }
  }

  // Write each element of source_frame_chain as string
  for (unsigned int i = 0; i < source_frame_chain.size(); ++i)
  {
    output.push_back(lookupFrameString(source_frame_chain[i]));
 }
}


} // namespace tf2
}
