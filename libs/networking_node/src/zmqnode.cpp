#include "zmqnode.h"
#include "transport.pb.h"

void my_free(void *data, void *hint)
{
    free (data);
}

ZMQNode::ZMQNode(bool reply)
{
  context_ = new zmq::context_t(1);
  socket_ = new zmq::socket_t(*context_, reply ? ZMQ_REP : ZMQ_REQ);
  connected_ = false;
  isReply_ = reply;
}

ZMQNode::~ZMQNode()
{
  delete(socket_);
  delete(context_);
}

void
ZMQNode::connect(std::string com)
{
  if (connected_) {
    // TODO: throw
  }

  socket_->connect(com);
  connected_ = true;
}

void
ZMQNode::bind(std::string com)
{
  if (connected_) {
    // TODO: throw
  }

  socket_->bind(com);
  connected_ = true;
}

void
ZMQNode::send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg)
{
  int size = msg->ByteSize();
  zmq::message_t msg_zmq( size );
  msg->SerializeToArray(msg_zmq.data(), size);
  socket_->send(msg_zmq);
}

void
ZMQNode::send(std::unique_ptr< ::google::protobuf::Message> msg)
{
  if ( ! connected_) {
    // TODO: throw
  }

  // check for COMP_ID and MSG_TYPE
  const google::protobuf::Descriptor *desc = msg->GetDescriptor();
  KeyType key = key_from_desc(desc);
  int comp_id = key.first;
  int msg_type = key.second;

  // send header
  std::unique_ptr<upns::Header> h(new upns::Header);
  h->set_comp_id(comp_id);
  h->set_msg_type(msg_type);

  send_pb_single(std::move(h));

  // send msg
  send_pb_single(std::move(msg));
}

void
ZMQNode::send_raw(unsigned char* data, size_t size)
{
  std::unique_ptr<upns::RawDataSize> pb(new upns::RawDataSize);
  pb->set_size(size);
  send_pb_single(std::move(pb));

  zmq::message_t msg(size);
  memcpy(msg.data(), data, size);
  socket_->send(msg);
//  zmq_msg_t msg_zmq;
//  zmq_msg_init_data(&msg_zmq, data, size, my_free, NULL);
//  zmq_send(socket_, &msg_zmq, size, 0);
}

std::shared_ptr< ::google::protobuf::Message>
ZMQNode::receive()
{
  if ( ! connected_) {
    // TODO: throw
  }

  // receive header
  upns::Header h;
  zmq::message_t msg_h;
  socket_->recv( &msg_h );
  h.ParseFromArray(msg_h.data(), msg_h.size());

  // receive msg
  zmq::message_t msg_zmq;
  socket_->recv( &msg_zmq );

  std::shared_ptr<google::protobuf::Message> msg = new_message_for(h.comp_id(), h.msg_type());
  msg->ParseFromArray(msg_zmq.data(), msg_zmq.size());

  return msg;
}

unsigned char *
ZMQNode::receive_raw(size_t & size)
{
  // receive size
  upns::RawDataSize pb;
  zmq::message_t msg_pb;
  socket_->recv( &msg_pb );
  pb.ParseFromArray(msg_pb.data(), msg_pb.size());
  size = pb.size();
  unsigned char * data = (unsigned char *) malloc( size );

  // receive data
  zmq::message_t msg;
  socket_->recv( &msg );
  memcpy(data, msg.data(), size);
//  zmq_recv(socket_, data, size, 0);
//  data = (unsigned char*)msg.data();
//  size = msg->size();

  return data;
}

ZMQNode::KeyType
ZMQNode::key_from_desc(const google::protobuf::Descriptor *desc)
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

std::shared_ptr<google::protobuf::Message>
ZMQNode::new_message_for(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (message_by_comp_type_.find(key) == message_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    throw std::runtime_error(msg);
  }

  google::protobuf::Message *m = message_by_comp_type_[key]->New();
  return std::shared_ptr<google::protobuf::Message>(m);
}
