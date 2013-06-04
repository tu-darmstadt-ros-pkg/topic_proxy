blob_tools
==========

blob provides a new message type blob/Blob for binary data.

Currently, only [roscpp](http://www.ros.org/wiki/roscpp) is supported.
Other client libraries like [rospy](http://www.ros.org/wiki/rospy) will serialize/deserialize blob data as uint8[],
as defined in the [message definition](https://github.com/meyerj/blob_tools/blob/master/blob/msg/Blob.msg).

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
        nav_msgs::OccupancyGridMapPtr my_map = msg->instantiate<nav_msgs::OccupancyGridMap>();
        
        if (my_map) {
            ....
        }
    }
