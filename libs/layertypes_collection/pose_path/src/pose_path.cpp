#include "upns/layertypes/pose_path.h"
#include <upns/logging.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};


void readTfsFromStream(upnsIStream &in, TfMatPosePath &tfsout )
{
    mapit::msgs::TransformPath tfs;
    if(!tfs.ParseFromIstream(&in))
    {
        log_warn("Could not read tranforms from stream. Proceeding with empty path");
    }
    else
    {
        for(int i=0 ; i<tfs.transforms_size(); ++i)
        {
            const mapit::msgs::Transform &tf = tfs.transforms(i);
            TfMatPose tfOut;
            tfOut.matrix() << tf.m00(), tf.m01(), tf.m02(), tf.m03(),
                             tf.m10(), tf.m11(), tf.m12(), tf.m13(),
                             tf.m20(), tf.m21(), tf.m22(), tf.m23(),
                             tf.m30(), tf.m31(), tf.m32(), tf.m33();
            tfsout.push_back(tfOut);
        }
    }
}
void writeTfToStream(upnsOStream &out, TfMatPosePath &data )
{
    mapit::msgs::TransformPath tfs;
    for(TfMatPosePath::const_iterator iter = data.cbegin(); iter != data.cend(); iter++)
    {
        mapit::msgs::Transform *tf = tfs.add_transforms();
        tf->set_m00( (*iter)(0, 0) ); tf->set_m01( (*iter)(0, 1) ); tf->set_m02( (*iter)(0, 2) ); tf->set_m03( (*iter)(0, 3) );
        tf->set_m10( (*iter)(1, 0) ); tf->set_m11( (*iter)(1, 1) ); tf->set_m12( (*iter)(1, 2) ); tf->set_m13( (*iter)(1, 3) );
        tf->set_m20( (*iter)(2, 0) ); tf->set_m21( (*iter)(2, 1) ); tf->set_m22( (*iter)(2, 2) ); tf->set_m23( (*iter)(2, 3) );
        tf->set_m30( (*iter)(3, 0) ); tf->set_m31( (*iter)(3, 1) ); tf->set_m32( (*iter)(3, 2) ); tf->set_m33( (*iter)(3, 3) );
    }
    tfs.SerializePartialToOstream(&out);
}

const char *PosePathEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

PosePathEntitydata::PosePathEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_posePath( nullptr )
{
}

const char* PosePathEntitydata::type() const
{
    return PosePathEntitydata::TYPENAME();
}

bool PosePathEntitydata::hasFixedGrid() const
{
    return false;
}

bool PosePathEntitydata::canSaveRegions() const
{
    return false;
}

TfMatPosePathPtr PosePathEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_posePath == NULL)
    {
        m_posePath = TfMatPosePathPtr(new TfMatPosePath);
        upnsIStream *in = m_streamProvider->startRead();
        {
            readTfsFromStream( *in, *m_posePath );
        }
        m_streamProvider->endRead(in);
    }
    return m_posePath;
}

int PosePathEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 TfMatPosePathPtr &data,
                                 int lod)
{
    upnsOStream *out = m_streamProvider->startWrite();
    {
        writeTfToStream( *out, *data );
    }
    m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

TfMatPosePathPtr PosePathEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int PosePathEntitydata::setData(TfMatPosePathPtr &data, int lod)
{
    return setData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   data, lod);
}

void PosePathEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
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

int PosePathEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                              upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
    //TODO
    return 0;
}

upnsIStream *PosePathEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startRead(start, len);
}

void PosePathEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *PosePathEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void PosePathEntitydata::endWrite(upnsOStream *strm)
{
    m_streamProvider->endWrite(strm);
}

size_t PosePathEntitydata::size() const
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
    PosePathEntitydata *p = static_cast<PosePathEntitydata*>(ld);
    delete p;
}
void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<AbstractEntitydata>(new PosePathEntitydata( streamProvider ), deleteEntitydata);
}

