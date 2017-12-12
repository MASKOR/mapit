#include "upns/layertypes/tflayer.h"
#include <upns/logging.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};

void readTfFromStream(upnsIStream &in, std::shared_ptr<tf::store::TransformStampedList> &tfsout )
{
  mapit::msgs::TransformStampedList tfs;
  if(!tfs.ParseFromIstream(&in)) {
    log_warn("TfEntitydata: tf entity is empty.");
    tfsout = nullptr;
    return;
  } else {
    std::string frame_id = tfs.frame_id();
    std::string child_frame_id = tfs.child_frame_id();

    tfsout = std::make_shared<tf::store::TransformStampedList>( std::move(tf::store::TransformStampedList(frame_id, child_frame_id, tfs.is_static())) );

    for (auto tf : tfs.transforms()) {
      std::unique_ptr<upns::tf::TransformStamped> tfout = std::make_unique<upns::tf::TransformStamped>();
      tfout->frame_id = frame_id;
      tfout->child_frame_id = child_frame_id;

      tfout->stamp = mapit::time::from_sec_and_nsec(tf.stamp().sec(), tf.stamp().nsec());

      tfout->transform.translation.x() = tf.transform().translation().x();
      tfout->transform.translation.y() = tf.transform().translation().y();
      tfout->transform.translation.z() = tf.transform().translation().z();

      tfout->transform.rotation.w() = tf.transform().rotation().w();
      tfout->transform.rotation.x() = tf.transform().rotation().x();
      tfout->transform.rotation.y() = tf.transform().rotation().y();
      tfout->transform.rotation.z() = tf.transform().rotation().z();

      tfsout->add_TransformStamped( std::move(tfout), tfs.is_static() );
    }
  }
}
void writeTfToStream(upnsOStream &out, tf::store::TransformStampedList &data )
{
  mapit::msgs::TransformStampedList tfs_to_write;
  tfs_to_write.set_is_static( data.get_is_static() );

  std::unique_ptr<std::list<std::unique_ptr<upns::tf::TransformStamped>>> tfs_in = data.dispose();

  bool at_least_one_data = false;

  for (const std::unique_ptr<upns::tf::TransformStamped>& tf_in : *tfs_in) {
    if ( ! at_least_one_data ) {
      tfs_to_write.set_child_frame_id( tf_in->child_frame_id );
      tfs_to_write.set_frame_id( tf_in->frame_id );
      at_least_one_data = true;
    }

    mapit::msgs::TransformWithTime* tf_to_write = new mapit::msgs::TransformWithTime();

    unsigned long sec, nsec;
    mapit::time::to_sec_and_nsec(tf_in->stamp, sec, nsec);
    tf_to_write->mutable_stamp()->set_sec( sec );
    tf_to_write->mutable_stamp()->set_nsec( nsec );
    tf_to_write->mutable_transform()->mutable_translation()->set_x( tf_in->transform.translation.x() );
    tf_to_write->mutable_transform()->mutable_translation()->set_y( tf_in->transform.translation.y() );
    tf_to_write->mutable_transform()->mutable_translation()->set_z( tf_in->transform.translation.z() );
    tf_to_write->mutable_transform()->mutable_rotation()->set_w( tf_in->transform.rotation.w() );
    tf_to_write->mutable_transform()->mutable_rotation()->set_x( tf_in->transform.rotation.x() );
    tf_to_write->mutable_transform()->mutable_rotation()->set_y( tf_in->transform.rotation.y() );
    tf_to_write->mutable_transform()->mutable_rotation()->set_z( tf_in->transform.rotation.z() );

    tfs_to_write.mutable_transforms()->AddAllocated( tf_to_write );
  }

  if ( ! at_least_one_data) {
    log_warn("list of transforms was empty, did not write anything");
    return;
  }
  tfs_to_write.SerializePartialToOstream(&out);
}

const char *TfEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

TfEntitydata::TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_transforms( NULL )
{
}

const char* TfEntitydata::type() const
{
    return TfEntitydata::TYPENAME();
}

bool TfEntitydata::hasFixedGrid() const
{
    return false;
}

bool TfEntitydata::canSaveRegions() const
{
    return false;
}

std::shared_ptr<tf::store::TransformStampedList> TfEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                       upnsReal x2, upnsReal y2, upnsReal z2,
                                       bool clipMode,
                                       int lod)
{
  if(m_transforms == nullptr) {
    upnsIStream *in = m_streamProvider->startRead();
    {
      readTfFromStream( *in, m_transforms );
    }
    m_streamProvider->endRead(in);
  }
  return m_transforms;
}

int TfEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                          upnsReal x2, upnsReal y2, upnsReal z2,
                          std::shared_ptr<tf::store::TransformStampedList> &data,
                          int lod)
{
  upnsOStream *out = m_streamProvider->startWrite();
  {
    writeTfToStream( *out, *data );
  }
  m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

std::shared_ptr<tf::store::TransformStampedList> TfEntitydata::getData(int lod)
{
  return getData(-std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                 false, lod);
}

int TfEntitydata::setData(std::shared_ptr<tf::store::TransformStampedList> &data, int lod)
{
  return setData(-std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                 data, lod);
}

void TfEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                              upnsReal &x1, upnsReal &y1, upnsReal &z1,
                              upnsReal &x2, upnsReal &y2, upnsReal &z2) const
{
  x1 = -std::numeric_limits<upnsReal>::infinity();
  y1 = -std::numeric_limits<upnsReal>::infinity();
  z1 = -std::numeric_limits<upnsReal>::infinity();
  x2 = +std::numeric_limits<upnsReal>::infinity();
  y2 = +std::numeric_limits<upnsReal>::infinity();
  z2 = +std::numeric_limits<upnsReal>::infinity();
}

int TfEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                       upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
  //TODO
  return 0;
}

upnsIStream *TfEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
  return m_streamProvider->startRead(start, len);
}

void TfEntitydata::endRead(upnsIStream *&strm)
{
  m_streamProvider->endRead(strm);
}

upnsOStream *TfEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
  return m_streamProvider->startWrite(start, len);
}

void TfEntitydata::endWrite(upnsOStream *&strm)
{
  m_streamProvider->endWrite(strm);
}

size_t TfEntitydata::size() const
{
  return m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//TODO: BIG TODO: Make libraries have a deleteEntitydata function and do not use shared pointers between libraries.
// TfEntitydata was deleted here although it was a plymesh
void deleteEntitydataTf(AbstractEntitydata *ld)
{
    TfEntitydata *p = dynamic_cast<TfEntitydata*>(ld);
    if(p)
    {
        delete p;
    }
    else
    {
        log_error("Wrong entitytype");
    }
}
void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<AbstractEntitydata>(new TfEntitydata( streamProvider ), deleteEntitydataTf);
}

