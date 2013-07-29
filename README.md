topic_proxy
===========

topic_proxy implements a ROS service server and client to pull single messages from one master and optionally
to republish them locally.

The package provides three targets:

* A server node that runs on the foreign master (usually the robot) and serves requests from clients.

  The TCP connection manager binds to a fixed port so that the client does not have to lookup the connection
  information at the foreign master (default port is 11322).
  
  Note: If you want to restart the server node you have to wait for approximately 1-2 minutes until the old
  socket listener transitions from TIME_WAIT to CLOSED state. Otherwise you will get an "Address already in use" error.
  The roscpp connection manager does not set the SO_REUSEADDR socket option. This problem is known as the "TCP TIME_WAIT
  problem".
  
* A client library named libtopic_proxy.so.

  The client library provides a high-level class interface for getting and publishing messages from a server. The most
  important public member functions are
  
        template <class M> boost::shared_ptr<const M> getMessage(string topic, Duration timeout, bool compressed);
        template <class M> void publishMessage(M message, string topic, bool compressed);
      
  which wrap the request into service calls.
  
* A client node that is built on top of the topic_proxy library and republishes or subscribes topics on the client side
  on behalf of the server.
  
  The client can be configured either via the parameter server (see
  [client.launch](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/launch/client.launch) file for a simple example)
  or by using service calls to request single messages or to configure the subscribers and publishers.
  
  If you want to connect to multiple servers (robots), you can run multiple clients in different namespaces.
  
Services
--------

### Server side:

* `/get_message` ([topic_proxy/GetMessage](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/srv/GetMessage.srv))

  Retrieve a single message from the server. The server will create a new subscriber if no one exists for the
  specified topic.
  
  Request parameters:

   * `string topic`:      Name of topic to subscribe at the server.
   * `bool compressed`:   If true, the message instance in the response is compressed using bzip2 compression.
   * `duration timeout`:  Time to wait for a new message, if no message has been received yet. If timeout is zero,
                          the last received message is returned without waiting.

* `/publish_message` ([topic_proxy/PublishMessage](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/srv/PublishMessage.srv))

  Publish a message at the server. The server creates a new publisher if no one exists for the specified topic.
  
  Request parameters:
  
    * `MessageInstance message`:     The message instance to publish.
    * `bool latch`:                  If true, the topic will be latched.
    
###Client side:

* `request_message` ([topic_proxy/RequestMessage](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/srv/RequestMessage.srv))

  Adds a new publisher at the client side (or modify the properties of an existing one). The client internally
  uses `/get_message` calls to retrieve messages from the server and republishes them locally.
  
  Request parameters:
  
   * `string topic`:      Name of the topic to subscribe at the server. The local topic is prefixed by
                          the value of the string parameter `topic_prefix` for the client.
   * `bool compressed`:   see `/get_message`
   * `duration timeout`:  see `/get_message`
   * `duration interval`: If >0, the client starts a timer to fetch messages periodically from the server.
                          If =0, a single message is retrieved and published. The service call will block
                          until the message is received and published.
                          If <=0 and there is an active timer for this topic, the timer will be stopped.
   * `bool latch`:        If true, the topic will be latched locally.
    
* `add_publisher` ([topic_proxy/AddPublisher](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/srv/AddPublisher.srv))

  Adds a new subscriber at the client side. The client forwards each incoming message to the server using a
  `/publish_message` call internally.
  
  Request parameters:
  
  * `string topic`:       Name of the topic to advertise at the server. The local topic is prefixed by
                          the value of the string parameter `topic_prefix` for the client.
  * `bool compressed`:    If true, all outgoing messages will be transferred compressed using bzip2 compression.
  * `bool latch`:         If true, the topic will be latched at the server.
  
  Note that you do not have to specify the message type. The first message that is reveived by the local subscriber
  will specify the message type and create the publisher at the server side. The first message sent from the client
  may therefore be missed by subscribers on the server side, as it takes some time for the connections to be
  established after the new publisher is created.
  

Example
-------

At the server:

    rostopic pub /chatter std_msgs/String "hello" -r 1 &
    rosrun topic_proxy server

At the client:

    ROS_NAMESPACE=client rosrun topic_proxy client [<host> [<port>]]
    
You have to use namespaces or the `topic_prefix` parameter if the client and server are using the same ROS master.
    
Then request a message from the server using

    rosservice call /client/request_message '{ topic: chatter }'
    
This will create a publisher on `/client/chatter` at the client and a subscriber on `/chatter` at the server. Only
one message is retrieved (interval parameter defaults to zero). If you want to poll the server periodically, specify
an interval:

    rosservice call /client/request_message '{ topic: chatter, interval: 1000000000 }'   # polls once a second
    
You can now listen to the server's chatter topic at the client:

    rostopic echo /client/chatter

To publish a response, call the `add_publiher` server to create a local subscriber:

    rosservice call /client/add_publisher '{ topic: chatter_response }'
    
The server-side publisher will be created once a message is received from the client:

    rostopic pub /client/chatter_response std_msgs/String "world" -r 1 &

The server will republish this message as `chatter` in his local namespace.

See [client.launch](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/topic_proxy/launch/client.launch) launch file in the topic_proxy package for a launch file example that does the same setup using
ROS parameters.
  

blob_tools
==========

blob provides a new message type blob/Blob for binary data.

Currently, only [roscpp](http://www.ros.org/wiki/roscpp) is supported.
Other client libraries like [rospy](http://www.ros.org/wiki/rospy) will serialize/deserialize blob data as uint8[],
as defined in the [message definition](https://github.com/tu-darmstadt-ros-pkg/topic_proxy/blob/master/blob/msg/Blob.msg).

Usage Example
-------------

MyMessage.msg:

    blob/Blob map_blob      # a serialized (and compressed) nav_msgs/OccupancyGridMap
    
Publisher:

    {
        ...
        MyMessage temp;
        nav_msgs::OccupancyGridMap my_map;
        ...
        temp.map_blob.serialize(my_map);
        temp.map_blob.setCompressed(true); // enable compression
        ...
        publisher.publish(temp);
    }

Subscriber:

    void callback(const MyMessageConstPtr& msg)
    {
        // WARNING: instantiation of the "wrong" data type may result in non-sense data and/or StreamOverrunExceptions
        // blob/Blob is not type-safe!
        nav_msgs::OccupancyGridMapPtr my_map = msg->map_blob.instantiate<nav_msgs::OccupancyGridMap>();
        
        if (my_map) {
            ....
        }
    }

