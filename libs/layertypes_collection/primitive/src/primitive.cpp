#include "upns/layertypes/primitive.h"
#include <upns/logging.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};

const char *PrimitiveEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

PrimitiveEntitydata::PrimitiveEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_primitive( nullptr )
{
}

const char* PrimitiveEntitydata::type() const
{
    return PrimitiveEntitydata::TYPENAME();
}

bool PrimitiveEntitydata::hasFixedGrid() const
{
    return false;
}

bool PrimitiveEntitydata::canSaveRegions() const
{
    return false;
}

PrimitivePtr PrimitiveEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_primitive == NULL)
    {
        m_primitive = PrimitivePtr(new mapit::msgs::Primitive);
        upnsIStream *in = m_streamProvider->startRead();
        {
            if(!m_primitive->ParseFromIstream(in))
            {
                log_warn("Could not read tranforms from stream. Proceeding with empty path");
            }
        }
        m_streamProvider->endRead(in);
    }
    return m_primitive;
}

int PrimitiveEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 PrimitivePtr &data,
                                 int lod)
{
    upnsOStream *out = m_streamProvider->startWrite();
    {
        data->SerializePartialToOstream(out);
    }
    m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

PrimitivePtr PrimitiveEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int PrimitiveEntitydata::setData(PrimitivePtr &data, int lod)
{
    return setData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   data, lod);
}

void PrimitiveEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
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

int PrimitiveEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                              upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
    //TODO
    return 0;
}

upnsIStream *PrimitiveEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startRead(start, len);
}

void PrimitiveEntitydata::endRead(upnsIStream *&strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *PrimitiveEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void PrimitiveEntitydata::endWrite(upnsOStream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t PrimitiveEntitydata::size() const
{
    return m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
void deleteEntitydataPrimitive(AbstractEntitydata *ld)
{
    PrimitiveEntitydata *p = dynamic_cast<PrimitiveEntitydata*>(ld);
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
    *out = std::shared_ptr<AbstractEntitydata>(new PrimitiveEntitydata( streamProvider ), deleteEntitydataPrimitive);
}

