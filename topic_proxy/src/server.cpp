#include <topic_proxy/topic_proxy.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/serialization.h>

#include <ros/network.h>
#include <ros/connection_manager.h>
#include <ros/transport/transport_tcp.h>

#include <blob/shape_shifter.h>

namespace topic_proxy
{

using blob::ShapeShifter;

class Server
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_message_server_;
  ros::ServiceServer publish_message_server_;

  typedef ros::MessageEvent<const ShapeShifter> MessageEvent;

  struct SubscriptionInfo {
    std::string topic;
    ros::Subscriber subscriber;
    ros::CallbackQueue callback_queue;
    MessageEvent event;
    ShapeShifter::ConstPtr last_message;
  };
  typedef boost::shared_ptr<SubscriptionInfo> SubscriptionInfoPtr;
  std::map<std::string, SubscriptionInfoPtr> subscriptions_;

  struct PublicationInfo {
    std::string topic;
    ros::Publisher publisher;
  };
  typedef boost::shared_ptr<PublicationInfo> PublicationInfoPtr;
  std::map<std::string, PublicationInfoPtr> publications_;

public:
  Server()
  {
    get_message_server_     = nh_.advertiseService(g_get_message_service, &Server::handleGetMessage, this);
    publish_message_server_ = nh_.advertiseService(g_publish_message_service, &Server::handlePublishMessage, this);
  }

  ~Server()
  {
    clearSubscriptions();
    clearPublications();
  }

  const std::string& getHost() const
  {
    return ros::network::getHost();
  }

  uint16_t getTCPPort() const
  {
    return ros::ConnectionManager::instance()->getTCPPort();
  }

protected:  
  const SubscriptionInfoPtr& getSubscription(const std::string& topic)
  {
    if (subscriptions_.count(topic)) return subscriptions_.at(topic);
    SubscriptionInfoPtr subscription(new SubscriptionInfo());
    return subscriptions_.insert(std::pair<std::string, SubscriptionInfoPtr>(topic, subscription)).first->second;
  }

  const PublicationInfoPtr& getPublication(const std::string& topic)
  {
    if (publications_.count(topic)) return publications_.at(topic);
    PublicationInfoPtr publication(new PublicationInfo());
    return publications_.insert(std::pair<std::string, PublicationInfoPtr>(topic, publication)).first->second;
  }

  bool handleGetMessage(GetMessage::Request& request, GetMessage::Response& response)
  {
    SubscriptionInfoPtr subscription = getSubscription(request.topic);

    if (!subscription->subscriber) {
      ROS_INFO("Subscribing to topic %s", request.topic.c_str());
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<ShapeShifter>(request.topic, 1, boost::bind(&Server::subscriberCallback, this, subscription, _1), ros::VoidConstPtr(), &(subscription->callback_queue));
      subscription->subscriber = nh_.subscribe(ops);
      subscription->topic = subscription->subscriber.getTopic();
    }

    // wait for exactly one callback and reset event pointer
    ros::WallDuration timeout(request.timeout.sec, request.timeout.nsec);
    if (timeout > ros::WallDuration()) {
      // clear callback queue and ignore all messages received if a timeout was specified
      subscription->callback_queue.clear();
    }
    subscription->event = MessageEvent();
    subscription->callback_queue.callOne(timeout);

    ShapeShifter::ConstPtr instance;
    try {
      instance = subscription->event.getConstMessage();

    } catch(ros::Exception& e) {
      ROS_ERROR("Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
      return false;
    }

    // any message message has been received?
    if (instance) {
      // fill response
      response.message.topic = subscription->topic;
      response.message.md5sum = instance->getMD5Sum();
      response.message.type = instance->getDataType();
      response.message.message_definition = instance->getMessageDefinition();
      // response.message.latching = subscription->event.getConnectionHeader()["latching"];
      response.message.blob = instance->blob();
      response.message.blob.setCompressed(request.compressed);

      subscription->last_message = instance;

    } else {
      // fill response from last message (but without data)
      response.message.topic = subscription->topic;
      if (subscription->last_message) {
        response.message.md5sum = subscription->last_message->getMD5Sum();
        response.message.type = subscription->last_message->getDataType();
        response.message.message_definition = subscription->last_message->getMessageDefinition();
      }
    }

    return true;
  }

  void subscriberCallback(const SubscriptionInfoPtr& subscription, const MessageEvent& event)
  {
    subscription->event = event;
  }

  bool handlePublishMessage(PublishMessage::Request& request, PublishMessage::Response& response)
  {
    PublicationInfoPtr publication = getPublication(request.message.topic);

    if (!publication->publisher) {
      ROS_INFO("Publishing topic %s (%s)", request.message.topic.c_str(), request.message.type.c_str());
      ros::AdvertiseOptions ops(request.message.topic, 10, request.message.md5sum, request.message.type, request.message.message_definition,
                                boost::bind(&Server::connectCallback, this, publication, _1), boost::bind(&Server::disconnectCallback, this, publication, _1));
      ops.latch = request.latch;
      publication->publisher = nh_.advertise(ops);
      publication->topic = publication->publisher.getTopic();
    }

    publication->publisher.publish(
      request.message.blob.asMessage().morph(request.message.md5sum, request.message.type, request.message.message_definition)
    );
    return true;
  }

  void clearSubscriptions() {
    for (std::map<std::string, SubscriptionInfoPtr>::iterator it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
      it->second->subscriber.shutdown();
    }
    subscriptions_.clear();
  }

  void clearPublications() {
    for (std::map<std::string, PublicationInfoPtr>::iterator it = publications_.begin(); it != publications_.end(); ++it) {
      it->second->publisher.shutdown();
    }
    publications_.clear();
  }

protected:
  void connectCallback(const PublicationInfoPtr&, const ros::SingleSubscriberPublisher&)
  {}

  void disconnectCallback(const PublicationInfoPtr&, const ros::SingleSubscriberPublisher&)
  {}
};

} // namespace topic_proxy

int main(int argc, char **argv)
{
  // add __tcpros_server_port to remappings
  std::string tcpros_server_port_argv = "__tcpros_server_port:=" + boost::lexical_cast<std::string>(topic_proxy::g_default_port);
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
