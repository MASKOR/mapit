//#include "upnszmqentitydata.h"
//#include "services_internal.pb.h"
//#include <sstream>

//deprecated

//upns::ZmqEntitydata::ZmqEntitydata(upns::upnsString checkoutName, upns::upnsString pathOrOid, ZmqNode *node)
//    :m_checkoutName(checkoutName),
//     m_pathOrOid(pathOrOid),
//     m_node(node),
//     m_e(nullptr),
//     m_ed(nullptr),
//     m_entityLength(0)
//{
//}

//upns::LayerType upns::ZmqEntitydata::layerType() const
//{
//    if(!m_e) initEntity();
//    return m_e->type();
//}

//bool upns::ZmqEntitydata::hasFixedGrid() const
//{
//    //TODO: nyi
//    //if(!m_ed) initHead();
//    return false;//m_ed->hasFixedGrid();
//}

//bool upns::ZmqEntitydata::canSaveRegions() const
//{
//    //TODO: nyi
//    //if(!m_ed) initHead();
//    return false;
//}

//void upns::ZmqEntitydata::gridCellAt(upns::upnsReal x, upns::upnsReal y, upns::upnsReal z, upns::upnsReal &x1, upns::upnsReal &y1, upns::upnsReal &z1, upns::upnsReal &x2, upns::upnsReal &y2, upns::upnsReal &z2) const
//{
//    //TODO: nyi
//    assert(false);
//    return;
//}

//upns::upnsIStream *upns::ZmqEntitydata::startReadBytes(upns::upnsuint64 start, upns::upnsuint64 length)
//{
//    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
//    req->set_checkout(m_checkoutName);
//    req->set_entitypath(m_pathOrOid);
//    req->set_offset(start);
//    req->set_maxlength(length);
//    m_node->send(std::move(req));
//    upnsSharedPointer<upns::ReplyEntitydata> rep(m_node->receive<upns::ReplyEntitydata>());
//    if(rep->status() != upns::ReplyEntitydata::SUCCESS)
//    {
//        log_error("received error from server when asked for entitydata");
//        return nullptr;
//    }
//    m_entityLength = rep->entitylength();
//    uint64_t recvlen = rep->receivedlength();
//    const uint64_t defaultBufferSize =  500ul*1024ul*1024ul; // 500 MB
//    const uint64_t maxBufferSize = 2ul*1024ul*1024ul*1024ul; // 2 GB
//    if(recvlen == 0)
//    {
//        log_info("Received empty entitydata (length: 0, entity: " + m_pathOrOid + ")");
//        recvlen = 1;
//    }
//    if(recvlen < 0)
//    {
//        log_warn("Entitydata with negative size received. (length: " + std::to_string(recvlen) + ")");
//        recvlen = defaultBufferSize;
//    }
//    if(recvlen > maxBufferSize)
//    {
//        log_warn("Entitydata length for single req/rep roundtrip exceeded maximum size. (length: " + std::to_string(recvlen) + ")");
//        recvlen = maxBufferSize;
//    }
//    char *buf = new char[recvlen]; // deleted by MemoryReaderDeleter

//    // eceive empty frames to not disturb following receives.
//    if(m_node->has_more())
//    {
//        size_t offset = 0;
//        do
//        {
//            offset += m_node->receive_raw_body(buf+offset, recvlen-offset);
//        } while (m_node->has_more() && offset <= recvlen);

//        if( offset != recvlen && !(offset == 0 && recvlen == 1) )
//        {
//            log_error("Entitydata does not have expected size. It may be corrupt. (entity: " + m_pathOrOid + ")");
//        }
//    }
//    return new MemoryReaderDeleter(buf, recvlen);




////    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
////    req->set_checkout(m_checkoutName);
////    req->set_entitypath(m_pathOrOid);
////    req->set_offset(0ul);
////    req->set_maxlength(0ul);
////    m_node->send(std::move(req));
////    m_ed = upns::upnsSharedPointer<upns::ReplyEntitydata>(m_node->receive<upns::ReplyEntitydata>());
////    // Receive empty frames to not disturb following receives.
////    if(m_node->has_more())
////    {
////        char buf[1];
////        do {
////            m_node->receive_raw_body(buf, sizeof(buf));
////        } while (m_node->has_more());
////    }



////    std::unique_ptr<upns::RequestEntity> req(new upns::RequestEntity);
////    req->set_checkout(m_checkoutName);
////    req->set_path(m_pathOrOid);
////    m_node->send(req);
////    upns::ReplyEntity *rep = m_d->receive<upns::ReplyEntity>();
////    if(rep == nullptr)
////    {
////        log_error("could not receive reply ReplyEntity");
////        return nullptr;
////    }

////    upnsSharedPointer<AbstractEntityData> ed = coraw->getEntityDataForReadWrite(msg->path());
////    upns::upnsOStream *stream = ed->startWriteBytes(msg->offset(), msg->sendlength());
////    size_t offset = 0;
////    while(has_more())
////    {
////        zmq::message_t *buf = receive_raw_body();
////        if(buf->size() + offset > msg->sendlength())
////        {
////            log_error("Tried to store entitydata with sendlength smaller than received datasize.");
////            return UPNS_STATUS_INVALID_ARGUMENT;
////        }
////        stream->write(static_cast<char*>(buf->data()), buf->size());
////        offset += buf->size();
////    }
////    if(offset != msg->sendlength())
////    {
////        log_error("Received entitydata of wrong length: should: " + std::to_string(msg->sendlength()) + ", is: " + std::to_string(offset) + "." );
////        return UPNS_STATUS_INVALID_DATA;
////    }

////    return UPNS_STATUS_OK;
//}

//void upns::ZmqEntitydata::endRead(upns::upnsIStream *strm)
//{
//    delete strm;
//}

//upns::upnsOStream *upns::ZmqEntitydata::startWriteBytes(upns::upnsuint64 start, upns::upnsuint64 len)
//{

//}

//void upns::ZmqEntitydata::endWrite(upns::upnsOStream *strm)
//{

//}

//size_t upns::ZmqEntitydata::getLength()
//{
//    if(m_entityLength == 0)
//    {
//        std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
//        req->set_checkout(m_checkoutName);
//        req->set_entitypath(m_pathOrOid);
//        req->set_offset(0ul);
//        req->set_maxlength(0ul);
//        m_node->send(std::move(req));
//        upnsSharedPointer<upns::ReplyEntitydata> rep(m_node->receive<upns::ReplyEntitydata>());
//        if(rep->status() != upns::ReplyEntitydata::SUCCESS)
//        {
//            log_error("received error from server when asked for entitydata");
//        }
//        else
//        {
//            m_entityLength = rep->entitylength();
//        }
//    }
//    return m_entityLength;
//}

//void upns::ZmqEntitydata::setLength(size_t length)
//{
//    //TODO: with e.entitylength = entitylength
//}

//void upns::ZmqEntitydata::initEntity() const
//{
//    std::unique_ptr<upns::RequestEntity> req(new upns::RequestEntity);
//    req->set_checkout(m_checkoutName);
//    req->set_path(m_pathOrOid);
//    m_node->send(std::move(req));
//    upns::upnsSharedPointer<upns::ReplyEntity> rep(m_node->receive<upns::ReplyEntity>());
//    if(rep == nullptr)
//    {
//        log_error("could not receive reply ReplyEntity");
//        m_e = nullptr;
//        return;
//    }
//    m_e = upns::upnsSharedPointer<upns::Entity>(new Entity(rep->entity()));
////    // Receive empty frames to not disturb following receives.
////    if(m_node->has_more())
////    {
////        char buf[1];
////        do {
////            m_node->receive_raw_body(buf, sizeof(buf));
////        } while (m_node->has_more());
////    }
//}
