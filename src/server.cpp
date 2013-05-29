#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/serialization.h>

#include <ros/network.h>
#include <ros/connection_manager.h>

#include <topic_tools/shape_shifter.h>
#include <topic_proxy/TopicRequest.h>

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
  Server(const std::string& peer = std::string())
    : peer_(peer)
  {
    server_ = nh_.advertiseService("/" + peer_ + "/topic_request", &Server::handleRequest, this);
  }

  ~Server()
  {
    clearSubscriptions();
  }

  const std::string& getHost() const
  {
    return ros::network::getHost();
  }

  uint32_t getTCPPort() const
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
  ros::init(argc, argv, "topic_proxy_server");

  std::string peer;
  if (argc < 2) {
    ROS_ERROR_STREAM("Syntax: rosrun " ROS_PACKAGE_NAME " server <peer>");
    return 1;
  }

  peer = std::string(argv[1]);
  topic_proxy::Server server(peer);

  ROS_INFO("Created topic_proxy server for %s on %s:%u", peer.c_str(), server.getHost().c_str(), server.getTCPPort());
  ros::spin();
  return 0;
}
