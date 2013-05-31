//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef BLOB_SHAPE_SHIFTER_H
#define BLOB_SHAPE_SHIFTER_H

#include <ros/ros.h>
#include <blob/Blob.h>

namespace blob {

struct ShapeShifter
{
  typedef ShapeShifter Type;

  ShapeShifter();
  ShapeShifter(const Blob& blob);
  virtual ~ShapeShifter();

  typedef boost::shared_ptr< ShapeShifter > Ptr;
  typedef boost::shared_ptr< ShapeShifter const> ConstPtr;

  // Helpers for inspecting shapeshifter
  std::string const& getDataType()          const;
  std::string const& getMD5Sum()            const;
  std::string const& getMessageDefinition() const;

  ShapeShifter& morph(const std::string& md5sum, const std::string& datatype,
             const std::string& msg_def = std::string(), const std::string& latching = std::string());

  // Helper for advertising
  ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch=false,
                           const ros::SubscriberStatusCallback &connect_cb=ros::SubscriberStatusCallback()) const;

  //! Call to try instantiating as a particular type
  template<class M>
  boost::shared_ptr<M> instantiate() const;

  //! Write serialized message contents out to a stream
  template<typename Stream>
  void write(Stream& stream) const;

  template<typename Stream>
  void read(Stream& stream);

  //! Return the size of the serialized message
  uint32_t size() const;

  const Blob& blob() const
  {
    return blob_;
  }

  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

private:
  std::string md5, datatype, msg_def, latching;
  Blob blob_;
};

typedef boost::shared_ptr<ShapeShifter > ShapeShifterPtr;
typedef boost::shared_ptr<ShapeShifter const> ShapeShifterConstPtr;

std::ostream& operator<<(std::ostream& s, const ShapeShifter & v)
{
  ros::message_operations::Printer< ShapeShifter >::stream(s, "", v);
  return s;
}

template <typename ContainerAllocator>
ShapeShifter Blob_<ContainerAllocator>::asMessage() const
{
  return ShapeShifter(*this);
}

} // namespace blob

// Message traits allow shape shifter to work with the new serialization API
namespace ros {
namespace message_traits {

template <> struct IsMessage<blob::ShapeShifter> : TrueType { };
template <> struct IsMessage<const blob::ShapeShifter> : TrueType { };

template<>
struct MD5Sum<blob::ShapeShifter>
{
  static const char* value(const blob::ShapeShifter& m) { return m.getMD5Sum().c_str(); }

  // Used statically, a shapeshifter appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct DataType<blob::ShapeShifter>
{
  static const char* value(const blob::ShapeShifter& m) { return m.getDataType().c_str(); }

  // Used statically, a shapeshifter appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct Definition<blob::ShapeShifter>
{
  static const char* value(const blob::ShapeShifter& m) { return m.getMessageDefinition().c_str(); }
};

} // namespace message_traits


namespace serialization
{

template<>
struct Serializer<blob::ShapeShifter>
{
  template<typename Stream>
  inline static void write(Stream& stream, const blob::ShapeShifter& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, blob::ShapeShifter& m)
  {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const blob::ShapeShifter& m) {
    return m.size();
  }
};


template<>
struct PreDeserialize<blob::ShapeShifter>
{
  static void notify(const PreDeserializeParams<blob::ShapeShifter>& params)
  {
    std::string md5       = (*params.connection_header)["md5sum"];
    std::string datatype  = (*params.connection_header)["type"];
    std::string msg_def   = (*params.connection_header)["message_definition"];
    std::string latching  = (*params.connection_header)["latching"];

    typedef std::map<std::string, std::string> map_t;
    boost::shared_ptr<map_t> shmap(new map_t(*params.connection_header));
    params.message->__connection_header = shmap;
    params.message->morph(md5, datatype, msg_def, latching);
  }
};

} // namespace serialization
} // namespace ros


// Template implementations:

namespace blob
{

  template<class M>
  boost::shared_ptr<M> ShapeShifter::instantiate() const
  {
    return blob_.instantiate<M>();
  }

  template<typename Stream>
  void ShapeShifter::write(Stream& stream) const {
    if (blob_.empty()) return;
    std::copy(blob_.begin(), blob_.end(), stream.advance(blob_.size()));
  }

  template<typename Stream>
  void ShapeShifter::read(Stream& stream)
  {
    stream.getLength();
    stream.getData();

    Blob::BufferPtr buffer(new Blob::Buffer(stream.getLength()));
    std::copy(stream.getData(), stream.getData() + stream.getLength(), buffer->begin());
    blob_.set(buffer);
  }

} // namespace blob

#endif // BLOB_SHAPE_SHIFTER_H
