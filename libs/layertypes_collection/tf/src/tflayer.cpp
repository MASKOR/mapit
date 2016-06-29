#include "tflayer.h"

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};


void readTfFromStream(upnsIStream &in, TfMat &tfout )
{
    upns::Transform tf;
    if(!tf.ParseFromIstream(&in))
    {
        log_warn("Could not read tranform from stream. Proceeding with identity");
        //assert(false);
        tfout.setToIdentity();
    }
    else
    {
        float *d = tfout.data();
        d[0] = tf.m00(); d[1] = tf.m01(); d[2] = tf.m02(); d[3] = tf.m03();
        d[4] = tf.m10(); d[5] = tf.m11(); d[6] = tf.m12(); d[7] = tf.m13();
        d[8] = tf.m20(); d[9] = tf.m21(); d[10]= tf.m22(); d[11]= tf.m23();
        d[12]= tf.m30(); d[13]= tf.m31(); d[14]= tf.m32(); d[15]= tf.m33();
        tfout.optimize();
    }
}
void writeTfToStream(upnsOStream &out, TfMat &data )
{
    const float *d = data.constData();
    upns::Transform tf;
    tf.set_m00(d[0] ); tf.set_m01(d[1] ); tf.set_m02(d[2] ); tf.set_m03(d[3] );
    tf.set_m10(d[4] ); tf.set_m11(d[5] ); tf.set_m12(d[6] ); tf.set_m13(d[7] );
    tf.set_m20(d[8] ); tf.set_m21(d[9] ); tf.set_m22(d[10]); tf.set_m23(d[11]);
    tf.set_m30(d[12]); tf.set_m31(d[13]); tf.set_m32(d[14]); tf.set_m33(d[15]);
    tf.SerializePartialToOstream(&out);
}

TfEntitydata::TfEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_tf( NULL )
{
}

LayerType TfEntitydata::layerType() const
{
    return LayerType::POSES;
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

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//upnsSharedPointer<AbstractEntityData> createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
//void* createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
void deleteEntitydata(AbstractEntityData *ld)
{
    TfEntitydata *p = static_cast<TfEntitydata*>(ld);
    delete p;
}
void createEntitydata(upnsSharedPointer<AbstractEntityData> *out, upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    *out = upnsSharedPointer<AbstractEntityData>(new TfEntitydata( streamProvider ), deleteEntitydata);
}

