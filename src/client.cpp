#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <ros/serialization.h>
#include <ros/network.h>

#include <topic_tools/shape_shifter.h>
#include <topic_proxy/TopicRequest.h>

namespace topic_proxy
{

using topic_tools::ShapeShifter;

class Client
{
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  ros::Subscriber request_subscriber_;

  std::string peer_;
  std::string host_;
  uint32_t port_;

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
  Client(const std::string& peer, const std::string& host, uint32_t port, const std::string& ns = std::string())
    : nh_(ns)
    , peer_(peer)
    , host_(host)
    , port_(port)
  {
    advertiseService();
    client_ = nh_.serviceClient<TopicRequest>("/" + peer_ + "/topic_request");
    request_subscriber_ = nh_.subscribe<TopicRequest::Request>("request", 10, boost::bind(&Client::requestCallback, this, _1));

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
          if (p.hasMember("interval"))   publication->latch = static_cast<bool>(p["latch"]);
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
    unadvertiseService();
  }

  const std::string& getHost() const
  {
    return host_;
  }

  uint32_t getTCPPort() const
  {
    return port_;
  }

  std::string getService()
  {
    return client_.getService();
  }

  ShapeShifter::Ptr requestTopic(const std::string& topic, ros::Duration timeout = ros::Duration(), bool compressed = false)
  {
    TopicRequest::Request request;
    request.topic = topic;
    request.timeout = timeout;
    request.compressed = compressed;
    return requestTopic(request);
  }

  ShapeShifter::Ptr requestTopic(TopicRequest::Request& request)
  {
    TopicRequest::Response response;
    if (!client_.waitForExistence()) return ShapeShifter::Ptr();
    if (!client_.call(request, response)) return ShapeShifter::Ptr();

    ShapeShifter::Ptr instance(new ShapeShifter());
    instance->morph(response.md5sum, response.type, response.message_definition, response.latching);

    try {
      ros::serialization::IStream stream(response.data.data(), response.data.size());
      instance->read(stream);
    } catch(ros::Exception& e) {
      ROS_ERROR("Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
      return ShapeShifter::Ptr();
    }

    return instance;
  }

  bool publish(TopicRequest::Request& request, bool latch = false)
  {
    ShapeShifter::Ptr instance = requestTopic(request);
    if (!instance) return false;

    PublicationInfoPtr publication = addPublication(request.topic);
    if (!publications_[request.topic]->publisher) {
      ROS_INFO("Advertising topic %s from host %s as %s", request.topic.c_str(), host_.c_str(), nh_.resolveName(request.topic).c_str());
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

  void requestCallback(const TopicRequest::Request::ConstPtr& request)
  {
    TopicRequest::Request copy(*request);
    if (!publish(copy)) {
      ROS_ERROR("Error while handling a request for topic %s", request->topic.c_str());
    }
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
  bool advertiseService()
  {
    // do not readvertise if this client runs on the same host as the server
    if (host_ == ros::network::getHost() || host_ == "localhost") return true;

    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    args[1] = "/" + peer_ + "/" + "topic_request";
    char uri_buf[1024];
    snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d", host_.c_str(), port_);
    args[2] = std::string(uri_buf);
    args[3] = ros::XMLRPCManager::instance()->getServerURI();
    return ros::master::execute("registerService", args, result, payload, true);
  }

  bool unadvertiseService()
  {
    // do not readvertise if this client runs on the same host as the server
    if (host_ == ros::network::getHost() || host_ == "localhost") return true;

    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    args[1] = "/" + peer_ + "/" + "topic_request";
    char uri_buf[1024];
    snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d", host_.c_str(), port_);
    args[2] = std::string(uri_buf);
    return ros::master::execute("unregisterService", args, result, payload, false);
  }

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

  std::string peer;
  std::string host;
  int port = 0;

  ros::param::get("~host", host);
  ros::param::get("~port", port);

  if (argc < 2 || host.empty() || port == 0) {
    ROS_ERROR_STREAM("Missing host or port parameter." << std::endl
                  << "Syntax: rosrun " ROS_PACKAGE_NAME " client <peer> _host:=<host> _port:=<port>");
    return 1;
  }

  peer = std::string(argv[1]);

  ROS_INFO("Contacting peer %s via %s:%u", peer.c_str(), host.c_str(), port);
  topic_proxy::Client client(peer, host, port);
  ros::spin();
  return 0;
}
