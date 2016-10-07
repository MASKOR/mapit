#ifndef ZMQNODE_H
#define ZMQNODE_H

#include "upns_globals.h"
#include <google/protobuf/message.h>
#include "transport.pb.h"
#include <zmq.hpp>
#include <memory>
#include <map>
#include <functional>

class ZmqNode
{
private:
  zmq::context_t * context_;
  zmq::socket_t * socket_;
  bool connected_;
  bool isReply_;

  typedef std::pair<uint16_t, uint16_t> KeyType;
  //typedef void (*ReceiveDelegate)(google::protobuf::Message*);
  typedef std::function<void(google::protobuf::Message*)> ReceiveDelegate;
  typedef std::function<void(const void*, const size_t)> ReceiveRawDelegate;
  typedef google::protobuf::Message* (ZmqNode::*ConcreteTypeFactory)(const void*, const size_t);
  typedef std::map<KeyType, ConcreteTypeFactory> ConcreteTypeFactoryMap;
  typedef std::map<KeyType, ReceiveRawDelegate> DelegateMap;
  ConcreteTypeFactoryMap factory_by_comp_type_;
  DelegateMap delegate_by_comp_type_;

  KeyType key_from_desc(const google::protobuf::Descriptor *desc);
  ReceiveRawDelegate get_handler_for_message(uint16_t component_id, uint16_t msg_type);
  ConcreteTypeFactory get_factory_for_message(uint16_t component_id, uint16_t msg_type);
  void send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg, int flags);

  template <typename MT>
  google::protobuf::Message* to_concrete_type(const void* data, const size_t size)
  {
    MT* msg( new MT );
    msg->ParseFromArray(data, size);
    return msg;
  }

public:
  ZmqNode(bool reply);
  ~ZmqNode();

  void connect(std::string com);
  void bind(std::string com);
  void receive_and_dispatch(int milliseconds);
  void discard_more();

  template <typename MT>
  MT* receive()
  {
      const google::protobuf::Descriptor *desc = MT::descriptor();
      KeyType key = key_from_desc(desc);

      if ( ! connected_) {
        // TODO: throw
      }

      // receive header
      upns::Header h;
      zmq::message_t msg_h;
      bool status = socket_->recv( &msg_h );
      if(!status) return nullptr;
      h.ParseFromArray(msg_h.data(), msg_h.size());

      if(h.comp_id() != key.first || h.msg_type() != key.second)
      {
          log_error("Server seems to speak another language. Could not parse received message.");
          return NULL;
      }
      // receive msg
      zmq::message_t msg_zmq;
      socket_->recv( &msg_zmq );

      // dispatch msg
      ConcreteTypeFactory factory = get_factory_for_message(h.comp_id(), h.msg_type());
      MT* ret = static_cast<MT*>((this->*factory)(msg_zmq.data(), msg_zmq.size()));
      return ret;
  }

  // Note: receive_raw_body will break the req/rep pattern, if it is used for the first frame of a request/response
  size_t receive_raw_body(void* data, size_t size);

  zmq::message_t *receive_raw_body();
  bool has_more();
  void send(std::unique_ptr< ::google::protobuf::Message> msg, bool sndmore = false);
  void send_raw_body(const unsigned char *data, size_t size, int flags = 0);
  void send_raw(unsigned char* data, size_t size, int flags = 0);

  template <typename MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type add_receivable_message_type()
  {
    const google::protobuf::Descriptor *desc = MT::descriptor();
    KeyType key = key_from_desc(desc);
    if (factory_by_comp_type_.find(key) != factory_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    factory_by_comp_type_[key] = &ZmqNode::to_concrete_type<MT>;
  }

  template <typename MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type add_receivable_message_type(ReceiveDelegate &handler)
  {
    const google::protobuf::Descriptor *desc = MT::descriptor();
    KeyType key = key_from_desc(desc);
    if (factory_by_comp_type_.find(key) != factory_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    factory_by_comp_type_[key] = &ZmqNode::to_concrete_type<MT>;
    if (delegate_by_comp_type_.find(key) != delegate_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered with delegate";
      throw std::runtime_error(msg);
    }
    delegate_by_comp_type_[key] = [handler, this](const void* data, const size_t size)
    {
      handler( this->to_concrete_type<MT>(data, size) );
    };
  }
};


#endif // ZMQNODE_H
