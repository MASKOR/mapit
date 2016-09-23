#ifndef ZMQNODE_H
#define ZMQNODE_H

#include <google/protobuf/message.h>
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
  typedef std::map<KeyType, ReceiveRawDelegate> TypeMap;//google::protobuf::Message* (*)()> TypeMap;
  TypeMap message_by_comp_type_;

  KeyType key_from_desc(const google::protobuf::Descriptor *desc);
  ReceiveRawDelegate get_handler_for_message(uint16_t component_id, uint16_t msg_type);
  void send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg);

  template <class MT>
  MT* to_concrete_type(const void* data, const size_t size)
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
  void handle_receive();
  unsigned char * receive_raw(size_t & size);
  void send(std::unique_ptr< ::google::protobuf::Message> msg);
  void send_raw(unsigned char* data, size_t size);

  template <class MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type add_receivable_message_type(ReceiveDelegate &handler)
  {
    const google::protobuf::Descriptor *desc = MT::descriptor();
    KeyType key = key_from_desc(desc);
    if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    message_by_comp_type_[key] = [handler, this](const void* data, const size_t size)
      {
        handler( this->to_concrete_type<MT>(data, size) );
      };
  }
};

#endif // ZMQNODE_H
