/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Based on protobuf_comm by Tim Niemueller */

/***************************************************************************
 *  message_register.cpp - Protobuf stream protocol - message register
 *
 *  Created: Fri Feb 01 15:48:36 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define NOMINMAX
#include "zmqprotobufnode.h"
#include <mapit/msgs/transport.pb.h>
#include <algorithm>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <stdlib.h>        // random()  RAND_MAX

//  Provide random number from 0..(num-1)
#define within(num) (int) ((float) (num) * random () / (RAND_MAX + 1.0))

void my_free(void *data, void *hint)
{
    free (data);
}

ZmqProtobufNode::ZmqProtobufNode(bool reply)
{
  context_ = new zmq::context_t(1);
  socket_ = new zmq::socket_t(*context_, reply ? ZMQ_REP : ZMQ_REQ);
//  if(!reply) {
//    std::stringstream ss;
//    ss << std::hex << std::uppercase
//       << std::setw(4) << std::setfill('0') << within (0x10000) << "-"
//       << std::setw(4) << std::setfill('0') << within (0x10000);
//    identity_ = ss.str();
//    socket_->setsockopt(ZMQ_IDENTITY, ss.str().c_str(), ss.str().length());
//  }
//#if (defined (WIN32))
//    s_set_id(socket_);//, (intptr_t)args);
//#else
//    s_set_id(socket_);          //  Set a printable identity
//#endif
  connected_ = false;
  isReply_ = reply;
}

ZmqProtobufNode::~ZmqProtobufNode()
{
  delete(socket_);
  if(connected_)
  {
    delete(context_);
  }
}

void
ZmqProtobufNode::connect(std::string com)
{
  if (connected_) {
    std::string msg = "connect called, but node is already connected";
    throw std::runtime_error(msg);
  }

  try {
    socket_->connect(com);
    address_ = com;
    connected_ = true;
    //const int receiveTimeout = 5000;
    //socket_->setsockopt(ZMQ_RCVTIMEO, &receiveTimeout, sizeof(receiveTimeout)); //Set Timeout before recv returns with EAGAIN
  } catch (zmq::error_t e) {
    log_error("Can't connect to server " + e.what());
    connected_ = false;
  }
}

void
ZmqProtobufNode::bind(std::string com)
{
  if (connected_) {
    std::string msg = "bind called, but node is already connected";
    throw std::runtime_error(msg);
  }

  socket_->bind(com);
  connected_ = true;
}

void
ZmqProtobufNode::send_pb_single(std::unique_ptr< ::google::protobuf::Message> msg, int flags)
{
  if ( ! connected_) {
    std::string msg = "send protobuf called, but node is not connected";
    throw std::runtime_error(msg);
  }
  assert(!has_more());

  int size = msg->ByteSize();
  zmq::message_t msg_zmq( size );
  msg->SerializeToArray(msg_zmq.data(), size);

  socket_->send(msg_zmq, flags);
}

void
ZmqProtobufNode::send(std::unique_ptr< ::google::protobuf::Message> msg, bool sndmore)
{
  if ( ! connected_) {
    std::string msg = "send called, but node is not connected";
    throw std::runtime_error(msg);
  }
  assert(!has_more());

  // check for COMP_ID and MSG_TYPE
  const google::protobuf::Descriptor *desc = msg->GetDescriptor();
  KeyType key = key_from_desc(desc);
  int comp_id = key.first;
  int msg_type = key.second;

  // send header
  std::unique_ptr<Header> h(new Header);
  h->set_comp_id(comp_id);
  h->set_msg_type(msg_type);
  send_pb_single(std::move(h), ZMQ_SNDMORE);

  // send msg
  send_pb_single(std::move(msg), sndmore ? ZMQ_SNDMORE : 0);
}

// Does not add header information. For multipart binary frames
void
ZmqProtobufNode::send_raw_body(const unsigned char* data, size_t size, int flags)
{
  if ( ! connected_) {
    std::string msg = "send raw called, but node is not connected";
    throw std::runtime_error(msg);
  }
  assert(!has_more());

  zmq::message_t msg(size);
  memcpy(msg.data(), data, size);
  socket_->send(msg, flags);
}

void
ZmqProtobufNode::receive_and_dispatch(int milliseconds)
{
  if ( ! connected_) {
    std::string msg = "Receive called, but node is not connected";
    throw std::runtime_error(msg);
  }

  socket_->setsockopt(ZMQ_RCVTIMEO, &milliseconds, sizeof(milliseconds));

  assert(isReply_);

  prepareForwardComChannel();

  // receive header
  Header h;
  zmq::message_t msg_h;
  bool status = socket_->recv( &msg_h );
  if(!status) {
      return; // hopefully timeout
  }
  h.ParseFromArray(msg_h.data(), msg_h.size());

  // receive msg
  zmq::message_t msg_zmq;
  socket_->recv( &msg_zmq );

  // dispatch msg
  ReceiveRawDelegate handler = get_handler_for_message(h.comp_id(), h.msg_type());
  if(!handler)
  {
      log_error("Remote seems to speak another language. Could not dispatch message.");
      return;
  }
  try {
    prepareBackComChannel();
    handler(msg_zmq.data(), msg_zmq.size());
  }
  catch(...)
  {
      log_error("Unknown Server error");
      try {
        discard_more();
      }
      catch(...) {}
  }

  // Make sure the handler received everything.
  try {
      assert(!has_more());
  } catch(...) {}
}

void ZmqProtobufNode::discard_more()
{
    while(has_more())
    {
        zmq::message_t msg;
        socket_->recv( &msg );
    }
}

size_t
ZmqProtobufNode::receive_raw_body(void* data, size_t size)
{
    //zmq::message_t msg(data, size); //TODO: zero copy would be nice
    zmq::message_t msg;
    socket_->recv( &msg );
    size_t len = msg.size();
    memcpy(data, msg.data(), std::min(size, len));
    return len;
}

zmq::message_t*
ZmqProtobufNode::receive_raw_body()
{
    //zmq::message_t msg(data, size); //TODO: zero copy would be nice
    zmq::message_t *msg(new zmq::message_t);
    socket_->recv( msg );
    return msg;
}

bool ZmqProtobufNode::has_more()
{
    int64_t more;
    size_t more_size = sizeof (more);
    socket_->getsockopt(ZMQ_RCVMORE, &more, &more_size);
    return more != 0;
}

void ZmqProtobufNode::prepareForwardComChannel()
{
}

void ZmqProtobufNode::prepareBackComChannel()
{
}

ZmqProtobufNode::KeyType
ZmqProtobufNode::key_from_desc(const google::protobuf::Descriptor *desc)
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

ZmqProtobufNode::ReceiveRawDelegate
ZmqProtobufNode::get_handler_for_message(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (delegate_by_comp_type_.find(key) == delegate_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    log_error("Remote seems to speak another language. " + msg);
    throw std::runtime_error(msg);
  }

  ZmqProtobufNode::ReceiveRawDelegate delegate = delegate_by_comp_type_[key];
  return delegate;
}

ZmqProtobufNode::ConcreteTypeFactory
ZmqProtobufNode::get_factory_for_message(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  if (factory_by_comp_type_.find(key) == factory_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" + std::to_string(msg_type) + " not registered";
    log_error("Remote seems to speak another language. " + msg);
    throw std::runtime_error(msg);
  }

  ZmqProtobufNode::ConcreteTypeFactory delegate = factory_by_comp_type_[key];
  return delegate;
}






