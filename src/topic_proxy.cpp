#include <topic_proxy/topic_proxy.h>

#include <ros/serialization.h>
#include <ros/transport/transport_tcp.h>
#include <ros/service_manager.h>
#include <ros/poll_manager.h>
#include <ros/connection_manager.h>
#include <ros/service_server_link.h>
#include <ros/node_handle.h>
#include <ros/service.h>

using namespace ros;

namespace topic_proxy
{
  const std::string TopicProxy::s_service_name = "/get_message";
  const uint32_t TopicProxy::s_default_port = 11322;

  TopicProxy::TopicProxy()
  {
    init();
  }

  TopicProxy::TopicProxy(const std::string& host, uint32_t port)
    : host_(host)
    , port_(port)
  {
    if (port_ == 0) port_ = s_default_port;
    init();
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

    bool persistent = true;
    std::string request_md5sum  = service_traits::md5sum<GetMessage::Request>();
    std::string response_md5sum = service_traits::md5sum<GetMessage::Response>();
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

  MessageInstanceConstPtr TopicProxy::sendRequest(GetMessage::Request& request)
  {
    if (!isValid() && !init()) return MessageInstanceConstPtr();

    namespace ser = serialization;
    SerializedMessage ser_req = ser::serializeMessage(request);
    SerializedMessage ser_resp;
    GetMessage::Response response;

    bool ok = link_->call(ser_req, ser_resp);
    if (!ok)
    {
      return MessageInstanceConstPtr();
    }

    try
    {
      ser::deserializeMessage(ser_resp, response);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown while while deserializing service response: %s", e.what());
      return MessageInstanceConstPtr();
    }

    MessageInstanceConstPtr message;
    try {
      message.reset(new MessageInstance(response.message));
    } catch(Exception& e) {
      ROS_ERROR("Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
    }

    return message;
  }

} // namespace topic_proxy
