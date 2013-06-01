//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef TOPIC_PROXY_H
#define TOPIC_PROXY_H

#include <topic_proxy/service_client.h>

#include <topic_proxy/GetMessage.h>
#include <topic_proxy/PublishMessage.h>

namespace topic_proxy
{

extern const uint32_t g_default_port;
extern const std::string g_get_message_service;
extern const std::string g_publish_message_service;

class TopicProxy
{
private:
  std::string host_;
  uint16_t port_;

public:
  TopicProxy();
  TopicProxy(const std::string& host, uint32_t port = 0);
  virtual ~TopicProxy();

  bool connect();
  void shutdown();

  const std::string& getHost() const { return host_; }
  uint16_t getTCPPort() const { return port_; }

  template <class M> boost::shared_ptr<const M> getMessage(const std::string& topic, ros::Duration timeout = ros::Duration(), bool compressed = false);
  template <class M> void publishMessage(const M& message, const std::string& topic, bool compressed = false);

protected:
  ServiceClient get_message_;
  MessageInstanceConstPtr send(GetMessage::Request&);

  ServiceClient publish_message_;
  bool send(PublishMessage::Request&);
};

template <class M>
boost::shared_ptr<const M> TopicProxy::getMessage(const std::string& topic, ros::Duration timeout, bool compressed)
{
  GetMessage::Request request;
  request.topic = topic;
  request.compressed = compressed;
  request.timeout = timeout;

  MessageInstanceConstPtr result = send(request);
  if (!result) return boost::shared_ptr<const M>();
  return result ? result->blob.instantiate<const M>() : boost::shared_ptr<const M>();
}

template <class M>
void TopicProxy::publishMessage(const M& message, const std::string& topic, bool compressed)
{
  PublishMessage::Request request;
  request.message.topic = topic;
  request.message.md5sum = ros::message_traits::md5sum<M>(message);
  request.message.type = ros::message_traits::datatype<M>(message);
  request.message.message_definition = ros::message_traits::definition<M>(message);
  request.message.blob.setCompressed(compressed);
  request.message.blob.serialize(message);

  return send(request);
}

} // namespace topic_proxy

#endif // TOPIC_PROXY_H
