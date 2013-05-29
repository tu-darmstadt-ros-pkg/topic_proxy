#include <topic_proxy/topic_proxy.h>
#include <topic_proxy/compression.h>
#include <topic_tools/shape_shifter.h>

#include <ros/serialization.h>
#include <ros/transport/transport_tcp.h>
#include <ros/service_manager.h>
#include <ros/poll_manager.h>
#include <ros/connection_manager.h>
#include <ros/service_server_link.h>

using namespace ros;

namespace topic_proxy
{
  const std::string TopicProxy::s_service_name = "/topic_request";
  const uint32_t TopicProxy::s_default_port = 11322;

  TopicProxy::TopicProxy()
  {
    init();
    compression_.reset(new Compression());
  }

  TopicProxy::TopicProxy(const std::string& host, uint32_t port)
    : host_(host)
    , port_(port)
  {
    init();
    compression_.reset(new Compression());
  }

  bool TopicProxy::init()
  {
    if (isValid()) return true;
    link_.reset();

    std::string host = host_;
    uint32_t port = port_;
    if (host.empty() || port == 0) {
      ros::NodeHandle temp; // required for service::waitForService
      service::waitForService(s_service_name);
      if (!ServiceManager::instance()->lookupService(s_service_name, host, port)) return false;
    }

    bool persistent = false;
    std::string request_md5sum  = service_traits::md5sum<TopicRequest::Request>();
    std::string response_md5sum = service_traits::md5sum<TopicRequest::Response>();
    M_string header_values;

    TransportTCPPtr transport(new TransportTCP(&PollManager::instance()->getPollSet()));
    if (transport->connect(host, port))
    {
      ConnectionPtr connection(new Connection());
      ConnectionManager::instance()->addConnection(connection);

      link_.reset(new ServiceServerLink(s_service_name, persistent, request_md5sum, response_md5sum, header_values));

      connection->initialize(transport, false, HeaderReceivedFunc());
      link_->initialize(connection);

      return true;
    }

    return false;
  }

  TopicProxy::~TopicProxy()
  {
  }

  bool TopicProxy::isValid() const
  {
    return (link_ && link_->isValid());
  }

  std::string TopicProxy::getServiceName()
  {
    if (!link_) return std::string();
    return link_->getServiceName();
  }

  ShapeShifter::Ptr TopicProxy::sendRequest(TopicRequest::Request& request)
  {
    if (!isValid() && !init()) return ShapeShifter::Ptr();

    namespace ser = serialization;
    SerializedMessage ser_req = ser::serializeMessage(request);
    SerializedMessage ser_resp;
    TopicRequest::Response response;

    bool ok = link_->call(ser_req, ser_resp);
    if (!ok)
    {
      return ShapeShifter::Ptr();
    }

    try
    {
      ser::deserializeMessage(ser_resp, response);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown while while deserializing service call: %s", e.what());
      return ShapeShifter::Ptr();
    }

    ShapeShifter::Ptr instance(new ShapeShifter());
    instance->morph(response.md5sum, response.type, response.message_definition, response.latching);

    if (response.is_compressed && compression_) {
      TopicRequest::Response::_data_type uncompressed;
      if (compression_->decompress(response.data, uncompressed)) {
        response.data.swap(uncompressed);

      } else {
        ROS_ERROR("%s decompression of a message of topic %s failed", compression_->getType().c_str(), request.topic.c_str());
        return ShapeShifter::Ptr();
      }
    }

    try {
      serialization::IStream stream(response.data.data(), response.data.size());
      instance->read(stream);
    } catch(Exception& e) {
      ROS_ERROR("Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
      return ShapeShifter::Ptr();
    }

    return instance;
  }

} // namespace topic_proxy
