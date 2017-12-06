#include <upns/layertypes/tflayer/utils.h>
#include <upns/logging.h>

using namespace upns::tf::store;


///////////////////////////////////////////////////////////
/// TransformStampedList
///////////////////////////////////////////////////////////

TransformStampedList::TransformStampedList(std::string frame_id, std::string child_frame_id, bool is_static)
  : frame_id_(frame_id)
  , child_frame_id_(child_frame_id)
  , is_static_(is_static)
{
  transforms_ = std::move( std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>>( new std::list<std::unique_ptr<upns::tf::TransformStamped>>() ) );
}

std::string
TransformStampedList::get_entity_name()
{
  return get_entity_name(frame_id_, child_frame_id_);
}

std::string
TransformStampedList::get_frame_id()
{
  return frame_id_;
}

std::string
TransformStampedList::get_child_frame_id()
{
  return child_frame_id_;
}

bool
TransformStampedList::get_is_static()
{
  return is_static_;
}

mapit::time::Stamp
TransformStampedList::get_stamp_earliest()
{
  return stamp_earliest_;
}

int
TransformStampedList::add_TransformStamped(std::unique_ptr<upns::tf::TransformStamped> in, bool is_static)
{
  // check pre conditions
  if (transforms_ == nullptr) { return 1; }
  if ( 0 != frame_id_.compare(in->frame_id) ) {
    log_error("tflayer: can't add transform, frame_id is not equal as of container");
    return 1;
  }
  if ( 0 != child_frame_id_.compare(in->child_frame_id) ) {
    log_error("tflayer: can't add transform, child_frame_id is not equal as of container");
    return 1;
  }
  if (is_static_ != is_static) {
    log_error("tflayer: can't add transform, is_static is not equal as of container");
    return 1;
  }

  // update stamp
  if (    stamp_earliest_ > in->stamp
       || transforms_->empty() ) {
    stamp_earliest_ = in->stamp;
  }

  // add transform
  transforms_->push_back( std::move( in ) );

  return 0;
}

std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>>
TransformStampedList::dispose()
{
  if (transforms_ == nullptr) { return nullptr; }
  std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> tmp
      = std::move( transforms_ );
  transforms_ = nullptr;
  return std::move( tmp );
}

///////////////////////////////////////////////////////////
/// TransformStampedListGatherer
///////////////////////////////////////////////////////////

TransformStampedListGatherer::TransformStampedListGatherer()
  : tfs_map_( std::make_shared<std::map<std::string, std::shared_ptr<TransformStampedList>>>() )
{ }

void
TransformStampedListGatherer::add_transform( std::unique_ptr<TransformStamped> tf, bool is_static )
{
  std::string entity_name = TransformStampedList::get_entity_name(tf->frame_id, tf->child_frame_id);
  // if not yet in list
  if (tfs_map_->find(entity_name) == tfs_map_->end()) {
    // create empty and add to map
    std::shared_ptr<TransformStampedList> tfs_store = std::make_shared<TransformStampedList>(tf->frame_id, tf->child_frame_id, is_static);
    (*tfs_map_)[ entity_name ] = tfs_store;
  }

  std::map<std::string, std::shared_ptr<TransformStampedList>>::iterator tfs_store_it = tfs_map_->find(entity_name);
  tfs_store_it->second->add_TransformStamped(std::move(tf), is_static);
}

upns::StatusCode
TransformStampedListGatherer::store_entities(CheckoutRaw* checkout, std::shared_ptr<mapit::Layer> layer)
{
  for (std::pair<std::string, std::shared_ptr<TransformStampedList>> tf_list : *tfs_map_) {
    std::shared_ptr<mapit::Entity> entity = checkout->getExistingOrNewEntity(
                                              layer
                                              , tf_list.first
                                              );
    entity->set_frame_id( tf_list.second->get_frame_id() );
    entity->set_stamp( tf_list.second->get_stamp_earliest() );

    std::shared_ptr<upns::tf::store::TransformStampedList> ed;
    if ( ! checkout->checkEntityData(entity) ) {
      // create new
      ed = tf_list.second;
    } else {
      std::shared_ptr<AbstractEntitydata> ed_a = checkout->getEntityDataReadWrite(entity);
      // appand
      if ( 0 != strcmp(TfEntitydata::TYPENAME(), ed_a->type()) ) {
        log_error("tflayer: internal error, entity for tf with name " + entity->getName() + " has entity date of other type then tf is: " + ed_a->type());
        return UPNS_STATUS_ERROR;
      }
      ed = std::static_pointer_cast<TfEntitydata>( ed_a )->getData();

      std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> tf_list_tmp = tf_list.second->dispose();
      for (auto &tf : *tf_list_tmp) {
        ed->add_TransformStamped(std::move( tf ), tf_list.second->get_is_static());
      }
    }
    checkout->storeEntity<TransformStampedList>(entity, ed);
  }

  return UPNS_STATUS_OK;
}
