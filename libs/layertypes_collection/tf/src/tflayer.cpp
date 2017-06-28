#include "upns/layertypes/tflayer.h"
#include <upns/logging.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};


void readTfFromStream(upnsIStream &in, tf::Transform &tfout )
{
  mapit::msgs::Transform tf;
  if(!tf.ParseFromIstream(&in)) {
    log_error("Could not read tranform from stream. Proceeding with identity");
    // TODO: throw exeption
  } else {
    tfout.translation.x() = tf.translation().x();
    tfout.translation.y() = tf.translation().y();
    tfout.translation.z() = tf.translation().z();

    tfout.rotation.w() = tf.rotation().w();
    tfout.rotation.x() = tf.rotation().x();
    tfout.rotation.y() = tf.rotation().y();
    tfout.rotation.z() = tf.rotation().z();

    tfout.child_frame_id = tf.child_frame_id();
  }
}
void writeTfToStream(upnsOStream &out, tf::Transform &data )
{
    mapit::msgs::Transform tf;
    tf.mutable_translation()->set_x( data.translation.x() );
    tf.mutable_translation()->set_y( data.translation.y() );
    tf.mutable_translation()->set_z( data.translation.z() );

    tf.mutable_rotation()->set_w( data.rotation.w() );
    tf.mutable_rotation()->set_x( data.rotation.x() );
    tf.mutable_rotation()->set_y( data.rotation.y() );
    tf.mutable_rotation()->set_z( data.rotation.z() );

    tf.set_child_frame_id( data.child_frame_id );

    tf.SerializePartialToOstream(&out);
}

const char *TfEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

TfEntitydata::TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_transform( NULL )
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

tf::TransformPtr TfEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                       upnsReal x2, upnsReal y2, upnsReal z2,
                                       bool clipMode,
                                       int lod)
{
  if(m_transform == NULL) {
    m_transform = tf::TransformPtr( new tf::Transform );
    upnsIStream *in = m_streamProvider->startRead();
    {
      readTfFromStream( *in, *m_transform );
    }
    m_streamProvider->endRead(in);
  }
  return m_transform;
}

int TfEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                          upnsReal x2, upnsReal y2, upnsReal z2,
                          tf::TransformPtr &data,
                          int lod)
{
  upnsOStream *out = m_streamProvider->startWrite();
  {
    writeTfToStream( *out, *data );
  }
  m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

tf::TransformPtr TfEntitydata::getData(int lod)
{
  return getData(-std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                 -std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                  std::numeric_limits<upnsReal>::infinity(),
                 false, lod);
}

int TfEntitydata::setData(tf::TransformPtr &data, int lod)
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

