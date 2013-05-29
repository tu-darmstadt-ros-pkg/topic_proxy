#include <ros/ros.h>
#include <topic_proxy/topic_proxy.h>
#include <topic_tools/shape_shifter.h>

namespace topic_proxy
{

class Client : public TopicProxy
{
private:
  ros::NodeHandle nh_;

  struct PublicationInfo
  {
    std::string topic;
    std::string datatype;
    std::string md5sum;
    std::string message_definition;

    ros::Publisher publisher;
    bool latch;
    ros::Timer timer;
    TopicRequest::Request request;
  };
  typedef boost::shared_ptr<PublicationInfo> PublicationInfoPtr;
  std::map<std::string, PublicationInfoPtr> publications_;

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
        PublicationInfoPtr publication(new PublicationInfo());
        ros::Duration interval(1.0);
        if (p.getType() == XmlRpc::XmlRpcValue::TypeString) {
          publication->request.topic = static_cast<std::string>(p);
        } else if (p.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (p.hasMember("topic"))      publication->request.topic = static_cast<std::string>(p["topic"]);
          if (p.hasMember("timeout"))    publication->request.timeout = ros::Duration(static_cast<double>(p["timeout"]));
          if (p.hasMember("compressed")) publication->request.compressed = static_cast<bool>(p["compressed"]);
          if (p.hasMember("interval"))   interval = ros::Duration(static_cast<double>(p["interval"]));
          if (p.hasMember("latch"))      publication->latch = static_cast<bool>(p["latch"]);
        }

        if (!publication->request.topic.empty() && !interval.isZero()) {
          publication = addPublication(publication->request.topic, publication);
          publication->timer = nh_.createTimer(interval, boost::bind(&Client::timerCallback, this, publication, _1));
        }
      }
    }
  }

  ~Client()
  {
    clearPublications();
  }

  bool publish(TopicRequest::Request& request, bool latch = false)
  {
    ShapeShifter::Ptr instance = sendRequest(request);
    if (!instance) return false;

    PublicationInfoPtr publication = addPublication(request.topic);
    if (!publications_[request.topic]->publisher) {
      if (!getHost().empty()) {
        ROS_INFO("Advertising topic %s from host %s as %s", request.topic.c_str(), getHost().c_str(), nh_.resolveName(request.topic).c_str());
      } else {
        ROS_INFO("Advertising topic %s as %s", request.topic.c_str(), nh_.resolveName(request.topic).c_str());
      }

      ros::AdvertiseOptions ops = ros::AdvertiseOptions(request.topic, 10, instance->getMD5Sum(), instance->getDataType(), instance->getMessageDefinition(), boost::bind(&Client::connectCallback, this, publication), boost::bind(&Client::disconnectCallback, this, publication));
      ops.latch = latch;
      publication->publisher = nh_.advertise(ops);
      publication->topic = publication->publisher.getTopic();
      publications_[request.topic] = publication;
    }

    publication->publisher.publish(instance);
    return true;
  }

  const PublicationInfoPtr& addPublication(const std::string& topic, PublicationInfoPtr init = PublicationInfoPtr())
  {
    if (publications_.count(topic)) return publications_.at(topic);
    PublicationInfoPtr publication = init;
    if (!publication) publication.reset(new PublicationInfo());
    return publications_.insert(std::pair<std::string, PublicationInfoPtr>(topic, publication)).first->second;
  }

  void timerCallback(const PublicationInfoPtr& publication, const ros::TimerEvent& event)
  {
    publish(publication->request, publication->latch);
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
