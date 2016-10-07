#include "zmqnode.h"
#include "transport.pb.h"
#include <algorithm>

void my_free(void *data, void *hint)
{
    free (data);
}

ZmqNode::ZmqNode(bool reply)
{
  context_ = new zmq::context_t(1);
  socket_ = new zmq::socket_t(*context_, reply ? ZMQ_REP : ZMQ_REQ);
  connected_ = false;
  isReply_ = reply;
}

ZmqNode::~ZmqNode()
{
  delete(socket_);
  delete(context_);
}

void
ZmqNode::connect(std::string com)
{
  if (connected_) {
    // TODO: throw
  }

  socket_->connect(com);
  connected_ = true;
}

void
ZmqNode::bind(std::string com)
{
  if (connected_) {
    // TODO: throw
  }

  socket_->bind(com);
  connected_ = true;
}

void
ZmqNode::send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg, int flags)
{
  if ( ! connected_) {
    // TODO: throw
  }
  assert(!has_more());

  int size = msg->ByteSize();
  zmq::message_t msg_zmq( size );
  msg->SerializeToArray(msg_zmq.data(), size);
  socket_->send(msg_zmq, flags);
}

void
ZmqNode::send(std::unique_ptr< ::google::protobuf::Message> msg, bool sndmore)
{
  if ( ! connected_) {
    // TODO: throw
  }
  assert(!has_more());

  // check for COMP_ID and MSG_TYPE
  const google::protobuf::Descriptor *desc = msg->GetDescriptor();
  KeyType key = key_from_desc(desc);
  int comp_id = key.first;
  int msg_type = key.second;

  // send header
  std::unique_ptr<upns::Header> h(new upns::Header);
  h->set_comp_id(comp_id);
  h->set_msg_type(msg_type);

  send_pb_single(std::move(h), ZMQ_SNDMORE);

  // send msg
  send_pb_single(std::move(msg), sndmore ? ZMQ_SNDMORE : 0);
}

// Does not add header information. For multipart binary frames
void
ZmqNode::send_raw_body(const unsigned char* data, size_t size, int flags)
{
  if ( ! connected_) {
    // TODO: throw
  }
  assert(!has_more());

  zmq::message_t msg(size);
  memcpy(msg.data(), data, size);
  socket_->send(msg, flags);
}

void
ZmqNode::send_raw(unsigned char* data, size_t size, int flags)
{
  if ( ! connected_) {
    // TODO: throw
  }
  assert(!has_more());

  std::unique_ptr<upns::RawDataSize> pb(new upns::RawDataSize);
  pb->set_size(size);
  send_pb_single(std::move(pb), ZMQ_SNDMORE);

  zmq::message_t msg(size);
  memcpy(msg.data(), data, size);
  socket_->send(msg, flags);
}

void
ZmqNode::receive_and_dispatch(int milliseconds)
{
  if ( ! connected_) {
    // TODO: throw
  }

  socket_->setsockopt(ZMQ_RCVTIMEO, &milliseconds, sizeof(milliseconds));

  // receive header
  upns::Header h;
  zmq::message_t msg_h;
  bool status = socket_->recv( &msg_h );
  if(!status) return; // hopefully timeout
  h.ParseFromArray(msg_h.data(), msg_h.size());

  // receive msg
  zmq::message_t msg_zmq;
  socket_->recv( &msg_zmq );

  // dispatch msg
  ReceiveRawDelegate handler = get_handler_for_message(h.comp_id(), h.msg_type());
  if(!handler)
  {
      log_error("Remote seems to speak another language. Could not dispatch message.");
  }
  handler(msg_zmq.data(), msg_zmq.size());

  // Make sure the handler received everything.
  assert(!has_more());
}

void ZmqNode::discard_more()
{
    while(has_more())
    {
        zmq::message_t msg;
        socket_->recv( &msg );
    }
}

size_t
ZmqNode::receive_raw_body(void* data, size_t size)
{
    //zmq::message_t msg(data, size); //TODO: zero copy would be nice
    zmq::message_t msg;
    socket_->recv( &msg );
    size_t len = msg.size();
    memcpy(data, msg.data(), std::min(size, len));
    return len;
}

zmq::message_t*
ZmqNode::receive_raw_body()
{
    //zmq::message_t msg(data, size); //TODO: zero copy would be nice
    zmq::message_t *msg(new zmq::message_t);
    socket_->recv( msg );
    return msg;
}

bool ZmqNode::has_more()
{
    int64_t more;
    size_t more_size = sizeof (more);
    socket_->getsockopt(ZMQ_RCVMORE, &more, &more_size);
    return more != 0;
}

ZmqNode::KeyType
ZmqNode::key_from_desc(const google::protobuf::Descriptor *desc)
{
  const google::protobuf::EnumDescriptor *enumdesc = desc->FindEnumTypeByName("CompType");
  if (! enumdesc) {
    throw std::logic_error("Message does not have CompType enum");
  }
  const google::protobuf::EnumValueDescriptor *compdesc =
    enumdesc->FindValueByName("COMP_ID");
  const google::protobuf::EnumValueDescriptor *msgtdesc =
    enumdesc->FindValueByName("MSG_TYPE");
  if (! compdesc || ! msgtdesc) {
    throw std::logic_error("Message CompType enum hs no COMP_ID or MSG_TYPE value");
  }
  int comp_id = compdesc->number();
  int msg_type = msgtdesc->number();
  if (comp_id < 0 || comp_id > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid COMP_ID");
  }
  if (msg_type < 0 || msg_type > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid MSG_TYPE");
  }
  return KeyType(comp_id, msg_type);
}

ZmqNode::ReceiveRawDelegate
ZmqNode::get_handler_for_message(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (delegate_by_comp_type_.find(key) == delegate_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    log_error("Remote seems to speak another language. " + msg);
    throw std::runtime_error(msg);
  }

  ZmqNode::ReceiveRawDelegate delegate = delegate_by_comp_type_[key];
  return delegate;
}

ZmqNode::ConcreteTypeFactory
ZmqNode::get_factory_for_message(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (factory_by_comp_type_.find(key) == factory_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    log_error("Remote seems to speak another language. " + msg);
    throw std::runtime_error(msg);
  }

  ZmqNode::ConcreteTypeFactory delegate = factory_by_comp_type_[key];
  return delegate;
}
