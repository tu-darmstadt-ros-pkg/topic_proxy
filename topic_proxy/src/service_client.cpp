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

#include <topic_proxy/service_client.h>
#include <ros/service_server_link.h>
#include <ros/connection.h>
#include <ros/service_manager.h>
#include <ros/service.h>

#include <ros/poll_manager.h>
#include <ros/transport/transport_tcp.h>
#include <ros/connection_manager.h>

namespace topic_proxy {

using namespace ros;

ServiceClient::Impl::Impl()
{ }

ServiceClient::Impl::~Impl()
{
  shutdown();
}

void ServiceClient::Impl::shutdown()
{
  if (server_link_)
  {
    server_link_->getConnection()->drop(Connection::Destructing);
    server_link_.reset();
  }
}

bool ServiceClient::Impl::isValid() const
{
  return server_link_ && server_link_->isValid();
}

ServiceClient::ServiceClient()
{}

ServiceClient::ServiceClient(const ServiceClient& rhs)
{
  impl_ = rhs.impl_;
}

ServiceClient::~ServiceClient()
{
}

bool ServiceClient::init(const std::string& service_name, const std::string& service_md5sum, std::string host, uint32_t port, const M_string& header_values)
{
  if (impl_ && impl_->isValid()) return true;
  impl_.reset(new Impl());

  impl_->name_ = service_name;

  if (host.empty() || port == 0) {
    NodeHandle temp; // required for service::waitForService
    service::waitForService(service_name);
    if (!ServiceManager::instance()->lookupService(service_name, host, port)) return false;
  }

  TransportTCPPtr transport(new TransportTCP(&PollManager::instance()->getPollSet()));
  if (!transport->connect(host, port)) return false;
  ConnectionPtr connection(new Connection());
  ConnectionManager::instance()->addConnection(connection);

  impl_->server_link_.reset(new ServiceServerLink(service_name, true, service_md5sum, service_md5sum, header_values));
  connection->initialize(transport, false, HeaderReceivedFunc());
  impl_->server_link_->initialize(connection);

  return impl_->isValid();
}

bool ServiceClient::call(const SerializedMessage& req, SerializedMessage& resp, const std::string& service_md5sum)
{
  if (!isValid()) return false;
  return impl_->server_link_->call(req, resp);
}

bool ServiceClient::isValid() const
{
  return impl_ && impl_->isValid();
}

void ServiceClient::shutdown()
{
  if (impl_)
  {
    impl_->shutdown();
  }
}

std::string ServiceClient::getService()
{
  if (impl_)
  {
    return impl_->name_;
  }

  return "";
}

} // namespace topic_proxy
