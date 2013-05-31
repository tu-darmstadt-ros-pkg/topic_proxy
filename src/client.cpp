#include <ros/ros.h>
#include <topic_proxy/topic_proxy.h>
#include <blob/shape_shifter.h>

#include <topic_proxy/RequestMessage.h>

namespace topic_proxy
{

using blob::ShapeShifter;

class Client : public TopicProxy
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer request_service_;

  struct PublicationInfo
  {
    std::string topic;
    std::string datatype;
    std::string md5sum;
    std::string message_definition;

    ros::Publisher publisher;
    bool latch;
    ros::Timer timer;
    GetMessage::Request request;
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
    if (isValid()) {
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

        PublicationInfoPtr publication = getPublication(topic);
        ros::Duration interval;
        if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("timeout"))    publication->request.timeout = ros::Duration(static_cast<double>(p["timeout"]));
          if (p.hasMember("compressed")) publication->request.compressed = static_cast<bool>(p["compressed"]);
          if (p.hasMember("interval"))   interval = ros::Duration(static_cast<double>(p["interval"]));
          if (p.hasMember("latch"))      publication->latch = static_cast<bool>(p["latch"]);
        }

        if (!interval.isZero()) {
          publication->timer = nh_.createTimer(interval, boost::bind(&Client::timerCallback, this, publication, _1));
        }
      }
    }

    ros::param::get("~topic_prefix", topic_prefix_);
    request_service_ = nh_.advertiseService("/request_message", &Client::handleRequest, this);
  }

  ~Client()
  {
    clearPublications();
  }

  bool publish(GetMessage::Request& request, bool latch = false)
  {
    MessageInstanceConstPtr instance = sendRequest(request);
    if (!instance) {
      ROS_ERROR("Request for topic %s failed", request.topic.c_str());
      return false;
    }

    PublicationInfoPtr publication = getPublication(request.topic);
    if (!publications_[request.topic]->publisher) {
      std::string advertised_topic = nh_.resolveName(topic_prefix_ + request.topic);
      if (!getHost().empty()) {
        ROS_INFO("Advertising topic %s from host %s as %s", request.topic.c_str(), getHost().c_str(), advertised_topic.c_str());
      } else {
        ROS_INFO("Advertising topic %s as %s", request.topic.c_str(), nh_.resolveName(request.topic).c_str());
      }

      ros::AdvertiseOptions ops = ros::AdvertiseOptions(advertised_topic, 10, instance->md5sum, instance->type, instance->message_definition, boost::bind(&Client::connectCallback, this, publication), boost::bind(&Client::disconnectCallback, this, publication));
      ops.latch = latch;
      publication->publisher = nh_.advertise(ops);
      publication->topic = publication->publisher.getTopic();
      publications_[request.topic] = publication;
    }

    blob::ShapeShifter message(instance->blob.asMessage());
    message.morph(instance->md5sum, instance->type, instance->message_definition);
    publication->publisher.publish(message);
    return true;
  }

  const PublicationInfoPtr& getPublication(const std::string& topic)
  {
    if (publications_.count(topic)) return publications_.at(topic);
    PublicationInfoPtr publication(new PublicationInfo());
    return publications_.insert(std::pair<std::string, PublicationInfoPtr>(topic, publication)).first->second;
  }

  void timerCallback(const PublicationInfoPtr& publication, const ros::TimerEvent& event)
  {
    publish(publication->request, publication->latch);
  }

  bool handleRequest(RequestMessage::Request& request, RequestMessage::Response& response)
  {
    PublicationInfoPtr publication = getPublication(request.topic);
    publication->request.topic = request.topic;
    publication->request.compressed = request.compressed;
    publication->request.timeout = request.timeout;
    publication->latch = request.latch;

    if (request.interval > ros::Duration()) {
      // add new timer
      publication->timer = nh_.createTimer(request.interval, boost::bind(&Client::timerCallback, this, publication, _1));
    } else {
      // stop otimer
      publication->timer.stop();
    }

    // request once
    if (request.interval.isZero()) {
      return publish(publication->request);
    }

    return true;
  }

  void clearPublications()
  {
    for (std::map<std::string, PublicationInfoPtr>::iterator it = publications_.begin(); it != publications_.end(); ++it) {
      it->second->publisher.shutdown();
    }
    publications_.clear();
  }

protected:
  void connectCallback(const PublicationInfoPtr& publication)
  {

  }

  void disconnectCallback(const PublicationInfoPtr& publication)
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
