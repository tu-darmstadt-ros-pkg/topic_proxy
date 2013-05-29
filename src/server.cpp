#include <topic_proxy/topic_proxy.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/serialization.h>

#include <ros/network.h>
#include <ros/connection_manager.h>
#include <ros/transport/transport_tcp.h>

#include <topic_tools/shape_shifter.h>

namespace topic_proxy
{

using topic_tools::ShapeShifter;

class Server
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;

  std::string peer_;

  typedef ros::MessageEvent<const ShapeShifter> MessageEvent;
  // typedef boost::shared_ptr<MessageEvent> MessageEventPtr;
  struct SubscriptionInfo {
    std::string topic;
    ros::Subscriber subscriber;
    ros::CallbackQueue callback_queue;
    MessageEvent event;
  };
  typedef boost::shared_ptr<SubscriptionInfo> SubscriptionInfoPtr;
  std::map<std::string, SubscriptionInfoPtr> subscriptions_;

public:
  Server()
  {
    server_ = nh_.advertiseService(TopicProxy::s_service_name, &Server::handleRequest, this);
  }

  ~Server()
  {
    clearSubscriptions();
  }

  const std::string& getHost() const
  {
    return ros::network::getHost();
  }

  uint16_t getTCPPort() const
  {
    return ros::ConnectionManager::instance()->getTCPPort();
  }

  std::string getService() const
  {
    return server_.getService();
  }

protected:
  bool handleRequest(TopicRequest::Request& request, TopicRequest::Response& response)
  {
    SubscriptionInfoPtr subscription;

    if (!subscriptions_.count(request.topic)) {
      ROS_INFO("Subscribing to topic %s", request.topic.c_str());
      subscription.reset(new SubscriptionInfo());
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<ShapeShifter>(request.topic, 1, boost::bind(&Server::subscriberCallback, this, subscription, _1), ros::VoidConstPtr(), &(subscription->callback_queue));
      subscription->subscriber = nh_.subscribe(ops);
      subscription->topic = subscription->subscriber.getTopic();
      subscriptions_[request.topic] = subscription;

    } else {
      subscription = subscriptions_.at(request.topic);
    }

    // wait for exactly one callback and reset event pointer
    subscription->event = MessageEvent();
    subscription->callback_queue.callOne(ros::WallDuration(request.timeout.sec, request.timeout.nsec));
    if (!subscription->event.getConstMessage()) return false;

    response.topic = subscription->event.getConnectionHeader()["topic"];
    response.md5sum = subscription->event.getConnectionHeader()["md5sum"];
    response.type = subscription->event.getConnectionHeader()["type"];
    response.message_definition = subscription->event.getConnectionHeader()["message_definition"];
    response.latching = subscription->event.getConnectionHeader()["latching"];
    response.is_compressed = false;

    try {
      ShapeShifter::ConstPtr instance = subscription->event.getConstMessage();
      response.data.resize(instance->size());
      ros::serialization::OStream stream(response.data.data(), response.data.size());
      instance->write(stream);
    } catch(ros::Exception& e) {
      ROS_ERROR("Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
      return false;
    }

    return true;
  }

  void subscriberCallback(const SubscriptionInfoPtr& subscription, const MessageEvent& event)
  {
    subscription->event = event;
  }

  void clearSubscriptions() {
    for (std::map<std::string, SubscriptionInfoPtr>::iterator it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
      it->second->subscriber.shutdown();
    }
    subscriptions_.clear();
  }

};

} // namespace topic_proxy

int main(int argc, char **argv)
{
  // add __tcpros_server_port to remappings
  std::string tcpros_server_port_argv = "__tcpros_server_port:=" + boost::lexical_cast<std::string>(topic_proxy::TopicProxy::s_default_port);
  std::vector<char *> new_argv;
  new_argv.reserve(argc + 1);
  new_argv.assign(&argv[0], &argv[argc]);
  new_argv.push_back(const_cast<char *>(tcpros_server_port_argv.c_str()));
  argc = new_argv.size();
  argv = new_argv.data();

  ros::init(argc, argv, "topic_proxy_server");
  topic_proxy::Server server;
  ROS_INFO("Created topic_proxy server listening on %s:%u", server.getHost().c_str(), server.getTCPPort());
  ros::spin();
  return 0;
}
