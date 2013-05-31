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

#include <ros/forwards.h>
#include <topic_proxy/GetMessage.h>

namespace topic_proxy
{

class TopicProxy
{
private:
  ros::ServiceServerLinkPtr link_;
  std::string host_;
  uint16_t port_;

public:
  static const std::string s_service_name;
  static const uint32_t s_default_port;

public:
  TopicProxy();
  TopicProxy(const std::string& host, uint32_t port = 0);
  virtual ~TopicProxy();

  bool isValid() const;

  const std::string& getHost() const { return host_; }
  uint16_t getTCPPort() const { return port_; }
  std::string getServiceName();

  template <class M> boost::shared_ptr<const M> requestTopic(const std::string& topic, ros::Duration timeout = ros::Duration(), bool compressed = false);

protected:
  bool init();
  MessageInstanceConstPtr sendRequest(GetMessage::Request& request);
};

template <class M>
boost::shared_ptr<const M> TopicProxy::requestTopic(const std::string& topic, ros::Duration timeout, bool compressed)
{
  GetMessage::Request request;
  request.topic = topic;
  request.compressed = compressed;
  request.timeout = timeout;

  MessageInstanceConstPtr result = sendRequest(request);
  if (!result) return boost::shared_ptr<const M>();
  return result ? result->blob.instantiate<M>() : boost::shared_ptr<M>();
}

} // namespace topic_proxy

#endif // TOPIC_PROXY_H
