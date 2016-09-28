#include "upnszmqentitydata.h"
#include "services_internal.pb.h"

upns::ZmqEntitydata::ZmqEntitydata(upns::upnsString checkoutName, upns::upnsString pathOrOid, ZmqNode *node)
    :m_checkoutName(checkoutName),
     m_pathOrOid(pathOrOid),
     m_node(node),
     m_ed(nullptr)
{
}

upns::LayerType upns::ZmqEntitydata::layerType() const
{
    if(!m_e) initEntity();
    return m_e->type();
}

bool upns::ZmqEntitydata::hasFixedGrid() const
{
    //TODO: nyi
    //if(!m_ed) initHead();
    return false;//m_ed->hasFixedGrid();
}

bool upns::ZmqEntitydata::canSaveRegions() const
{
    //TODO: nyi
    //if(!m_ed) initHead();
    return false;
}

void upns::ZmqEntitydata::gridCellAt(upns::upnsReal x, upns::upnsReal y, upns::upnsReal z, upns::upnsReal &x1, upns::upnsReal &y1, upns::upnsReal &z1, upns::upnsReal &x2, upns::upnsReal &y2, upns::upnsReal &z2) const
{
    //TODO: nyi
    assert(false);
    return;
}

upns::upnsIStream *upns::ZmqEntitydata::startReadBytes(upns::upnsuint64 start, upns::upnsuint64 len)
{

}

void upns::ZmqEntitydata::endRead(upns::upnsIStream *strm)
{

}

upns::upnsOStream *upns::ZmqEntitydata::startWriteBytes(upns::upnsuint64 start, upns::upnsuint64 len)
{

}

void upns::ZmqEntitydata::endWrite(upns::upnsOStream *strm)
{

}

void upns::ZmqEntitydata::initHead() const
{
    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
    req->set_checkout(m_checkoutName);
    req->set_entitypath(m_pathOrOid);
    req->set_offset(0);
    req->set_maxsize(0);
    m_node->send(std::move(req));
    m_ed = upns::upnsSharedPointer<upns::ReplyEntitydata>(m_node->receive<upns::ReplyEntitydata>());
    // Receive empty frames to not disturb following receives.
    if(m_node->has_more())
    {
        char buf[0];
        int64_t more;
        do {
            m_node->receive_raw_body(buf, sizeof(buf));
        } while (m_node->has_more());
    }
}

void upns::ZmqEntitydata::initEntity() const
{
    std::unique_ptr<upns::RequestEntity> req(new upns::RequestEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyEntity> rep(m_node->receive<upns::ReplyEntity>());
    m_e = upns::upnsSharedPointer<upns::Entity>(new Entity(rep->entity()));
    // Receive empty frames to not disturb following receives.
    if(m_node->has_more())
    {
        char buf[0];
        int64_t more;
        do {
            m_node->receive_raw_body(buf, sizeof(buf));
        } while (m_node->has_more());
    }
}
