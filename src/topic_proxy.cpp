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
  const std::string g_get_message_service     = "/get_message";
  const std::string g_publish_message_service = "/publish_message";
  const uint32_t g_default_port               = 11322;

  TopicProxy::TopicProxy()
  {
  }

  TopicProxy::TopicProxy(const std::string& host, uint32_t port)
    : host_(host)
    , port_(port)
  {
    if (port_ == 0) port_ = g_default_port;
  }

  bool TopicProxy::connect()
  {
    return get_message_.init<GetMessage>(g_get_message_service, host_, port_)
        && publish_message_.init<PublishMessage>(g_publish_message_service, host_, port_);
  }

  void TopicProxy::shutdown()
  {
    get_message_.shutdown();
    publish_message_.shutdown();
  }

  TopicProxy::~TopicProxy()
  {
  }

  MessageInstanceConstPtr TopicProxy::send(GetMessage::Request& request)
  {
    if (!get_message_.isValid() && !get_message_.init<GetMessage>(g_get_message_service, host_, port_)) {
      return MessageInstanceConstPtr();
    }

    GetMessage::Response response;
    if (!get_message_.call(request, response)) {
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

  bool TopicProxy::send(PublishMessage::Request& request)
  {
    if (!publish_message_.isValid() && !publish_message_.init<PublishMessage>(g_publish_message_service, host_, port_)) {
      return false;
    }

    PublishMessage::Response response;
    if (!publish_message_.call(request, response)) {
      return false;
    }

    return true;
  }

} // namespace topic_proxy
