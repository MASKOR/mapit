#include "upns/layertypes/tflayer.h"
#include <upns/logging.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};


void readTfFromStream(upnsIStream &in, TfMat &tfout )
{
    mapit::msgs::Transform tf;
    if(!tf.ParseFromIstream(&in))
    {
        log_warn("Could not read tranform from stream. Proceeding with identity");
        //assert(false);
        tfout = TfMat::Identity();
    }
    else
    {
        tfout.matrix() << tf.m00(), tf.m01(), tf.m02(), tf.m03(),
                          tf.m10(), tf.m11(), tf.m12(), tf.m13(),
                          tf.m20(), tf.m21(), tf.m22(), tf.m23(),
                          tf.m30(), tf.m31(), tf.m32(), tf.m33();
    }
}
void writeTfToStream(upnsOStream &out, TfMat &data )
{
    mapit::msgs::Transform tf;
    tf.set_m00( data(0, 0) ); tf.set_m01( data(0, 1) ); tf.set_m02( data(0, 2) ); tf.set_m03( data(0, 3) );
    tf.set_m10( data(1, 0) ); tf.set_m11( data(1, 1) ); tf.set_m12( data(1, 2) ); tf.set_m13( data(1, 3) );
    tf.set_m20( data(2, 0) ); tf.set_m21( data(2, 1) ); tf.set_m22( data(2, 2) ); tf.set_m23( data(2, 3) );
    tf.set_m30( data(3, 0) ); tf.set_m31( data(3, 1) ); tf.set_m32( data(3, 2) ); tf.set_m33( data(3, 3) );
    tf.SerializePartialToOstream(&out);
}

const char *TfEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

TfEntitydata::TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_tf( NULL )
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

TfMatPtr TfEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_tf == NULL)
    {
        m_tf = TfMatPtr(new TfMat);
        upnsIStream *in = m_streamProvider->startRead();
        {
            readTfFromStream( *in, *m_tf );
        }
        m_streamProvider->endRead(in);
    }
    return m_tf;
}

int TfEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 TfMatPtr &data,
                                 int lod)
{
    upnsOStream *out = m_streamProvider->startWrite();
    {
        writeTfToStream( *out, *data );
    }
    m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

TfMatPtr TfEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int TfEntitydata::setData(TfMatPtr &data, int lod)
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

void TfEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *TfEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void TfEntitydata::endWrite(upnsOStream *strm)
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
void deleteEntitydata(AbstractEntitydata *ld)
{
    TfEntitydata *p = static_cast<TfEntitydata*>(ld);
    delete p;
}
void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<AbstractEntitydata>(new TfEntitydata( streamProvider ), deleteEntitydata);
}

