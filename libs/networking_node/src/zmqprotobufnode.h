#ifndef ZMQNODE_H
#define ZMQNODE_H

#include <upns/typedefs.h>
#include <upns/logging.h>
#include <google/protobuf/message.h>
#include <mapit/msgs/transport.pb.h>
#include <zmq.hpp>
#include <memory>
#include <map>
#include <functional>

using namespace mapit::msgs;

///
/// \brief The ZmqProtobufNode class is the low level interface to access network using zmq and req/rep.
/// It is used for client and server. Client and server share functionality (send).
///

class ZmqProtobufNode
{
private:
  zmq::context_t *context_;
  zmq::socket_t *socket_;
  bool connected_;
  bool isReply_;
  std::string address_;

  typedef std::pair<uint16_t, uint16_t> KeyType;
  typedef std::function<void(google::protobuf::Message*)> ReceiveDelegate;
  typedef std::function<void(const void*, const size_t)> ReceiveRawDelegate;
  typedef google::protobuf::Message* (ZmqProtobufNode::*ConcreteTypeFactory)(const void*, const size_t);
  typedef std::map<KeyType, ConcreteTypeFactory> ConcreteTypeFactoryMap;
  typedef std::map<KeyType, ReceiveRawDelegate> DelegateMap;
  ConcreteTypeFactoryMap factory_by_comp_type_;
  DelegateMap delegate_by_comp_type_;

  KeyType key_from_desc(const google::protobuf::Descriptor *desc);
  ReceiveRawDelegate get_handler_for_message(uint16_t component_id, uint16_t msg_type);
  ConcreteTypeFactory get_factory_for_message(uint16_t component_id, uint16_t msg_type);
  void send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg, int flags);

  template <typename MT>
  google::protobuf::Message* to_concrete_type(const void* data, const size_t size);

public:
  /**
   * Replier can be used to ensure that a reply is sent when the scope os left.
   * This will result in cleaner and less error prone code when exceptions are used
   */
  template <typename MT>
  class Replier
  {
      ZmqProtobufNode *m_node;
      std::unique_ptr<MT> m_reply;
      bool m_wasSent;
  public:
    Replier(MT *reply, ZmqProtobufNode *node)
      : m_reply(reply), m_node(node), m_wasSent(false)
    {}
    std::unique_ptr<MT>& reply() { return m_reply; }
    void send()
    {
        m_node->send(std::move(m_reply));
        m_wasSent = true;
    }
    ~Replier()
    {
        assert(m_wasSent); // request reply pattern: a reply must be sent also in case of error. If this happens, the application did not send a reply.
    }
  };
  template <typename MT>
  Replier<MT> autoReplyOnScopeExit(MT *reply)
  {
      return Replier<MT> ( new Replier<MT>(reply, this));
  }

  /**
   * @brief ZmqNode Creates a Node communicating protobuf messages over Zmq using Request Reply
   * @param reply indicates if this is a request (most times client) or reply (most times server) node.
   */
  ZmqProtobufNode(bool reply);
  ~ZmqProtobufNode();

  /**
   * @brief connect Create a connection to another ZmqProtobufNode
   * @param com connection url for zmq. Example: "tcp://testserver:5555"
   */
  void connect(std::string com);

  /**
   * @brief bind listen and accept incoming connections
   * @param com see zmq documentation, most times a port. Example: "tcp://*:5555"
   */
  void bind(std::string com);

  /**
   * @brief receive_and_dispatch waits for a single message and calls a previously registered callback method. Blocking call.
   * The callback and messagetype must be registered using @sa add_receivable_message_type
   * @param milliseconds timeout if nothing is received.
   */
  void receive_and_dispatch(int milliseconds);

  /**
   * @brief discard_more discards all following parts of a multi-part-message.
   * Note that there must be no parts left, before another request reply pair is started.
   */
  void discard_more();

  /**
   * @brief receive receives a single message. This does typechecking for you.
   * The type must be a subclass of protobuf::Message.
   * The messagetype must be registered using @sa add_receivable_message_type
   */
  template <typename MT>
  MT* receive();

  /**
   * @brief receive_raw_body used to retreive parts of a multipart-message. This must not be used for the first part of a message.
   * Note: receive_raw_body will break the req/rep pattern, if it is used for the first frame of a request/response
   * @param data
   * @param size
   * @return
   */
  size_t receive_raw_body(void* data, size_t size);

  /**
   * @brief receive_raw_body used to retreive parts of a multipart-message. This must not be used for the first part of a message.
   * Note: receive_raw_body will break the req/rep pattern, if it is used for the first frame of a request/response
   * @param data
   * @param size
   * @return
   */
  zmq::message_t *receive_raw_body();

  /**
   * @brief has_more Check if there are more parts of the message which can be received.
   * @return true, if the message has parts, which have not yet been received(). Call receive_raw_body for more parts.
   */
  bool has_more();

  /**
   * @brief send Send a protobuf message
   * @param msg protobuf message
   * @param sndmore true for multipart. The last part of a multipart message uses "false" here, so the whole message can be sent.
   */
  void send(std::unique_ptr< ::google::protobuf::Message> msg, bool sndmore = false);
  /**
   * @brief send_raw_body used to send parts which are NOT the first part of a multipart-message
   * @param data
   * @param size
   * @param flags can be ZMQ_SNDMORE
   */
  void send_raw_body(const unsigned char *data, size_t size, int flags = 0);

  /**
   * @brief Use subclasses of protobuf::Message as a templateparameter. These Messages must contain an enum with MSG_ID and COMP_ID.
   * The node can then handle this type of message and do typechecking for you.
   * Use this, if you want to use the @sa receive() method (most times the client).
   */
  template <typename MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type add_receivable_message_type();

  /**
   * @brief Use subclasses of protobuf::Message as a templateparameter. These Messages must contain an enum with MSG_ID and COMP_ID.
   * The node can then dispatch message of this type to the associated callbeck method.
   * Use this, if you want to use the @sa receive_and_dispatch() method (most times the server).
   */
  template <typename MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type add_receivable_message_type(ReceiveDelegate &handler);
};


////// Implementations of template functions //////


template <typename MT>
google::protobuf::Message* ZmqProtobufNode::to_concrete_type(const void* data, const size_t size)
{
  MT* msg( new MT );
  msg->ParseFromArray(data, size);
  return msg;
}

template <typename MT>
MT* ZmqProtobufNode::receive()
{
    const google::protobuf::Descriptor *desc = MT::descriptor();
    KeyType key = key_from_desc(desc);

    if ( ! connected_) {
      return nullptr;
    }

    // receive header
    Header h;
    zmq::message_t msg_h;
    bool status = socket_->recv( &msg_h );
    if(!status) {
        socket_->disconnect(address_);
        connected_ = false;
        return nullptr;
    }
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

template <typename MT>
typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type ZmqProtobufNode::add_receivable_message_type()
{
  const google::protobuf::Descriptor *desc = MT::descriptor();
  KeyType key = key_from_desc(desc);
  if (factory_by_comp_type_.find(key) != factory_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
    throw std::runtime_error(msg);
  }
  factory_by_comp_type_[key] = &ZmqProtobufNode::to_concrete_type<MT>;
}

template <typename MT>
typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type ZmqProtobufNode::add_receivable_message_type(ReceiveDelegate &handler)
{
  const google::protobuf::Descriptor *desc = MT::descriptor();
  KeyType key = key_from_desc(desc);
  if (factory_by_comp_type_.find(key) != factory_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered";
    throw std::runtime_error(msg);
  }
  factory_by_comp_type_[key] = &ZmqProtobufNode::to_concrete_type<MT>;
  if (delegate_by_comp_type_.find(key) != delegate_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(key.first) + ":" + std::to_string(key.second) + " already registered with delegate";
    throw std::runtime_error(msg);
  }
  delegate_by_comp_type_[key] = [handler, this](const void* data, const size_t size)
  {
    google::protobuf::Message *msg = this->to_concrete_type<MT>(data, size);
    handler( msg );
    delete static_cast<MT*>(msg);
  };
}

#endif // ZMQNODE_H
