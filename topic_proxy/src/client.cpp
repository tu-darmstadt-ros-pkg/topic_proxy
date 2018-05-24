#include <ros/ros.h>
#include <topic_proxy/topic_proxy.h>
#include <blob/shape_shifter.h>

#include <topic_proxy/RequestMessage.h>
#include <topic_proxy/AddPublisher.h>

namespace topic_proxy
{

using blob::ShapeShifter;

class Client : public TopicProxy
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer request_message_service_;
  ros::ServiceServer add_publisher_service_;

  struct SubscriptionInfo
  {
    std::string local_topic;
    std::string datatype;
    std::string md5sum;
    std::string message_definition;

    ros::Publisher publisher;
    bool latch;
    ros::Timer timer;
    GetMessage::Request request;
  };
  typedef boost::shared_ptr<SubscriptionInfo> SubscriptionInfoPtr;
  std::map<std::string, SubscriptionInfoPtr> subscriptions_;

  struct PublicationInfo
  {
    std::string remote_topic;
    ros::Subscriber subscriber;
    bool latch;
    bool compressed;
  };
  typedef boost::shared_ptr<PublicationInfo> PublicationInfoPtr;
  std::map<std::string, PublicationInfoPtr> publications_;

  std::string topic_prefix_;

public:
  Client(const std::string& ns)
    : TopicProxy()
    , nh_(ns)
  {
    init();
  }

  Client(const std::string& host, uint32_t port, const std::string& ns = std::string())
    : TopicProxy(host, port)
    , nh_(ns)
  {
    init();
  }

  bool init() {
    if (connect()) {
      if (!getHost().empty()) {
        ROS_INFO("Connected to topic_proxy server at %s:%u", getHost().c_str(), getTCPPort());
      } else {
        ROS_INFO("Connected to topic_proxy server at local master %s", ros::master::getURI().c_str());
      }
    } else {
      if (!getHost().empty()) {
        ROS_WARN("Could not connect to topic_proxy server at %s:%u", getHost().c_str(), getTCPPort());
      } else {
        ROS_WARN("Could not connect to topic_proxy server at local master %s", ros::master::getURI().c_str());
      }
      return false;
    }

    ros::param::get("~topic_prefix", topic_prefix_);
    request_message_service_ = nh_.advertiseService("request_message", &Client::handleRequestMessage, this);
    add_publisher_service_ = nh_.advertiseService("add_publisher", &Client::handleAddPublisher, this);

    XmlRpc::XmlRpcValue subscriptions;
    ros::param::get("~subscriptions", subscriptions);
    if (subscriptions.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < subscriptions.size(); ++i) {
        XmlRpc::XmlRpcValue p = subscriptions[i];
        std::string topic;
        if (p.getType() == XmlRpc::XmlRpcValue::TypeString) {
          topic = static_cast<std::string>(p);
        } else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("topic")) topic = static_cast<std::string>(p["topic"]);
        }
        if (topic.empty()) continue;

        SubscriptionInfoPtr subscription = getSubscription(topic);
        subscription->local_topic = topic;
        subscription->request.topic = topic;
        ros::Duration interval;
        if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("remote_topic")) subscription->request.topic = static_cast<std::string>(p["remote_topic"]);
          if (p.hasMember("timeout"))      subscription->request.timeout = ros::Duration(static_cast<double>(p["timeout"]));
          if (p.hasMember("compressed"))   subscription->request.compressed = static_cast<bool>(p["compressed"]);
          if (p.hasMember("interval"))     interval = ros::Duration(static_cast<double>(p["interval"]));
          if (p.hasMember("latch"))        subscription->latch = static_cast<bool>(p["latch"]);
        }

        if (!interval.isZero()) {
          subscription->timer = nh_.createTimer(interval, boost::bind(&Client::timerCallback, this, subscription, _1));
        }
      }
    }

    XmlRpc::XmlRpcValue publications;
    ros::param::get("~publications", publications);
    if (publications.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < publications.size(); ++i) {
        XmlRpc::XmlRpcValue p = publications[i];
        std::string topic;
        if (p.getType() == XmlRpc::XmlRpcValue::TypeString) {
          topic = static_cast<std::string>(p);
        } else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("topic")) topic = static_cast<std::string>(p["topic"]);
        }
        if (topic.empty()) continue;

        AddPublisher::Request request;
        AddPublisher::Response response;
        request.topic = topic;
        if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("remote_topic")) request.remote_topic = static_cast<std::string>(p["remote_topic"]);
          if (p.hasMember("compressed"))   request.compressed = static_cast<bool>(p["compressed"]);
          if (p.hasMember("latch"))        request.latch = static_cast<bool>(p["latch"]);
        }

        handleAddPublisher(request, response);
      }
    }

    return true;
  }

  ~Client()
  {
    clearSubscriptions();
    clearPublications();
  }

  bool republish(GetMessage::Request& request, bool latch = false)
  {
    republish(request, request.topic, latch);
  }

  bool republish(GetMessage::Request& request, const std::string& topic, bool latch = false)
  {
    MessageInstanceConstPtr instance = send(request);
    if (!instance) {
      ROS_ERROR("GetMessage request for topic %s failed", request.topic.c_str());
      return false;
    }

    // advertise locally
    SubscriptionInfoPtr subscription = getSubscription(topic);
    if (!subscription->publisher
        && !instance->type.empty() && !instance->md5sum.empty()) {
      if (subscription->local_topic.empty()) subscription->local_topic = topic;
      std::string advertised_topic = nh_.resolveName(topic_prefix_ + subscription->local_topic);
      if (!getHost().empty()) {
        ROS_INFO("Advertising topic %s from host %s as %s", request.topic.c_str(), getHost().c_str(), advertised_topic.c_str());
      } else {
        ROS_INFO("Advertising topic %s as %s", request.topic.c_str(), advertised_topic.c_str());
      }

      ros::AdvertiseOptions ops = ros::AdvertiseOptions(advertised_topic, 10, instance->md5sum, instance->type, instance->message_definition,
                                                        boost::bind(&Client::connectCallback, this, subscription, _1), boost::bind(&Client::disconnectCallback, this, subscription, _1));
      ops.latch = latch;
      subscription->publisher = nh_.advertise(ops);
    }

    if (instance->blob.empty()) {
      ROS_DEBUG("No message received on topic %s", request.topic.c_str());
      return false;
    }

    subscription->publisher.publish(
      instance->blob.asMessage().morph(instance->md5sum, instance->type, instance->message_definition)
    );
    return true;
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

  void publishCallback(const PublicationInfoPtr& publication, const blob::ShapeShifterConstPtr& message)
  {
    PublishMessage::Request request;
    request.message.topic = publication->remote_topic;
    request.message.type = message->getDataType();
    request.message.md5sum = message->getMD5Sum();
    request.message.message_definition = message->getMessageDefinition();
    request.message.blob = message->blob();
    request.message.blob.setCompressed(publication->compressed);
    request.latch = publication->latch;

    if (!send(request)) {
      ROS_ERROR("PublishMessage request for topic %s failed", request.message.topic.c_str());
    }
  }

  void timerCallback(const SubscriptionInfoPtr& subscription, const ros::TimerEvent& event)
  {
    republish(subscription->request, subscription->local_topic, subscription->latch);
  }

  bool handleRequestMessage(RequestMessage::Request& request, RequestMessage::Response& response)
  {
    SubscriptionInfoPtr subscription = getSubscription(request.topic);
    subscription->request.topic = request.remote_topic;
    subscription->request.compressed = request.compressed;
    subscription->request.timeout = request.timeout;
    subscription->latch = request.latch;
    subscription->local_topic = request.topic;

    if (request.interval > ros::Duration()) {
      // add new timer
      subscription->timer = nh_.createTimer(request.interval, boost::bind(&Client::timerCallback, this, subscription, _1));
    } else {
      // stop otimer
      subscription->timer.stop();
    }

    // request once
    if (request.interval.isZero()) {
      return republish(subscription->request, request.topic, request.latch);
    }

    return true;
  }

  bool handleAddPublisher(AddPublisher::Request& request, AddPublisher::Response& response)
  {
    PublicationInfoPtr publication = getPublication(request.topic);

    if (!publication->subscriber) {
      std::string subscribed_topic = nh_.resolveName(topic_prefix_ + request.topic);
      if (!getHost().empty()) {
        ROS_INFO("Subscribing to topic %s for host %s as %s", request.topic.c_str(), getHost().c_str(), subscribed_topic.c_str());
      } else {
        ROS_INFO("Subscribing to topic %s as %s", request.topic.c_str(), subscribed_topic.c_str());
      }

      publication->subscriber = nh_.subscribe<blob::ShapeShifter>(subscribed_topic, 10, boost::bind(&Client::publishCallback, this, publication, _1));
      publication->remote_topic = request.topic;
      if (!request.remote_topic.empty()) publication->remote_topic = request.remote_topic;
      publication->latch = request.latch;
      publication->compressed = request.compressed;
    }

    return true;
  }

  void clearSubscriptions()
  {
    for (std::map<std::string, SubscriptionInfoPtr>::iterator it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
      it->second->publisher.shutdown();
    }
    subscriptions_.clear();
  }

  void clearPublications()
  {
    for (std::map<std::string, PublicationInfoPtr>::iterator it = publications_.begin(); it != publications_.end(); ++it) {
      it->second->subscriber.shutdown();
    }
    publications_.clear();
  }

protected:
  void connectCallback(const SubscriptionInfoPtr&, const ros::SingleSubscriberPublisher&)
  {
  }

  void disconnectCallback(const SubscriptionInfoPtr&, const ros::SingleSubscriberPublisher&)
  {
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_proxy_client");

  std::string host;
  int port = 0;

  // get host/port from parameter server
  ros::param::get("~host", host);
  ros::param::get("~port", port);

  // get host/port from command line
  if (argc > 1) host = argv[1];
  if (argc > 2) port = boost::lexical_cast<uint32_t>(std::string(argv[2]));

  topic_proxy::Client client(host, port);
  ros::spin();
  return 0;
}
