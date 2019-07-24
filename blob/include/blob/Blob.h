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

#ifndef BLOB_MESSAGE_BLOB_H
#define BLOB_MESSAGE_BLOB_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <ros/console.h>

#include <boost/shared_ptr.hpp>
#include <vector>

#include <blob/compression.h>

namespace blob {

class ShapeShifter;

template <class ContainerAllocator>
struct Blob_
{
  typedef Blob_<ContainerAllocator> Type;
  typedef uint8_t value_type;
  typedef uint32_t size_type;

  typedef std::vector<value_type> Buffer;
  typedef boost::shared_ptr<Buffer> BufferPtr;
  typedef boost::shared_ptr<const Buffer> ConstBufferPtr;

  Blob_()
    : compressed_(false)
  {
    clear();
  }

  Blob_(const ContainerAllocator& _alloc)
    : compressed_(false)
  {
    clear();
  }

  Blob_(const void *data, size_type size, bool compressed = false)
    : compressed_(compressed)
  {
    set(data, size);
  }

  typedef boost::shared_ptr< ::blob::Blob_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::blob::Blob_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

  // assign pointer, size and copy (if available)
  Blob_<ContainerAllocator>& operator=(const Blob_<ContainerAllocator>& other)
  {
    clear();
    pointer_ = other.pointer_;
    size_ = other.size_;
    copy_ = other.copy_;
    return *this;
  }

public:
  size_type size()          const { return size_; }
  const value_type *data()  const { return pointer_; }
  const value_type *begin() const { return pointer_; }
  const value_type *end()   const { return pointer_ + size_; }

  bool isCopy() const { return copy_; }
  void setCompressed(bool compressed) { compressed_ = compressed; }
  bool isCompressed() const { return compressed_; }

  void clear()
  {
    pointer_ = 0;
    size_ = 0;
    copy_.reset();
    compressed_blob_.reset();
  }

  bool empty() const
  {
    return (pointer_ == 0);
  }

  void set(const void *data, size_type size)
  {
    clear();
    pointer_ = reinterpret_cast<const value_type *>(data);
    size_ = size;
  }

  void set(ConstBufferPtr data)
  {
    clear();
    copy_ = data;
    pointer_ = data->data();
    size_ = data->size();
  }

  void copy()
  {
    if (copy_) return;
    if (empty()) return;
    BufferPtr copy(new Buffer(size()));
    std::copy(begin(), end(), copy->data());
    pointer_ = copy->data();
    copy_ = copy;
  }

  void copy(const value_type *data, size_type size)
  {
    set(data, size);
    copy();
  }

  ConstBufferPtr getCopy()
  {
    copy();
    return copy_;
  }

  /* ROS serialization */

  template<typename Stream>
  void write(Stream& stream) const
  {
    ROS_DEBUG_NAMED("blob", "Writing a blob of size %u at address %p to the stream", size(), data());

    if (!empty() && isCompressed()) {
      if (compress()) {
        ROS_DEBUG_NAMED("blob", "Using compression. Compressed size %u bytes (%.1f%%)", static_cast<uint32_t>(compressed_blob_->size()), 100.0 * (1.0 - static_cast<float>(compressed_blob_->size()) / static_cast<float>(size())));
        stream.next(static_cast<uint8_t>(true));
        stream.next(static_cast<uint32_t>(compressed_blob_->size()));
        std::copy(compressed_blob_->begin(), compressed_blob_->end(), stream.advance(compressed_blob_->size()));
        return;
      }
    }

    stream.next(static_cast<uint8_t>(false));
    stream.next(static_cast<uint32_t>(size()));
    std::copy(begin(), end(), stream.advance(size()));
  }

  template<typename Stream>
  void read(Stream& stream)
  {
    uint8_t is_compressed;
    uint32_t size;
    stream.next(is_compressed);
    stream.next(size);
    ROS_DEBUG_NAMED("blob", "Reading %s blob of size %u at address %p from the stream", std::string(is_compressed ? "a compressed" : "an uncompressed").c_str(), size, stream.getData());

    if (is_compressed) {
      if (!decompress(stream.advance(size), size)) {
        throw ros::serialization::StreamOverrunException("Decompression error");
      }
      return;
    }

    // ros::MessageDeserializer::deserialize frees the original buffer of the SerializedMessage
    // We have to create a copy.
    // set(stream.advance(size), size);
    copy(stream.advance(size), size);
  }

  uint32_t serializedLength() const
  {
    uint32_t length = ros::serialization::serializationLength(true) + ros::serialization::serializationLength(size());

    if (!empty() && isCompressed()) {
      if (compress()) {
        return length + compressed_blob_->size();
      }
    }

    return length + size();
  }

  /* Instantiation and Serialization */

  template <typename M>
  boost::shared_ptr<M> instantiate() const {
    if (empty()) return boost::shared_ptr<M>();
    boost::shared_ptr<typename boost::remove_const<M>::type> m(new M());
    ros::serialization::IStream stream(const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(data())), size());
    ros::serialization::deserialize(stream, *m);
    return m;
  }

  template <typename M>
  void serialize(const M& message) {
    clear();
    BufferPtr buffer(new Buffer(ros::serialization::serializationLength(message)));
    ros::serialization::OStream stream(reinterpret_cast<uint8_t *>(buffer->data()), buffer->size());
    ros::serialization::serialize(stream, message);
    set(buffer);
  }

  // implemented in shape_shifter.h
  ShapeShifter asMessage() const;

  /* Compression */

  ConstBufferPtr getCompressedBlob() const {
    compress();
    return compressed_blob_;
  }

  bool setFromCompressedData(const void *data, uint32_t size) {
    return decompress(reinterpret_cast<const uint8_t *>(data), size);
  }

private:
  // compress blob into compressed_blob_
  bool compress() const
  {
    if (!compressed_blob_) {
      BufferPtr temp(new Buffer());

      if (!::blob::deflate(data(), size(), *temp)) {
        ROS_WARN_NAMED("blob", "Error during compression of a blob of size %u", size());
        return false;
      }

      compressed_blob_ = temp;
    }

    return (compressed_blob_->size() < size());
  }

  // decompress compressed_blob_ into copy_
  bool decompress(const uint8_t *data, uint32_t size)
  {
    clear();
    BufferPtr temp(new Buffer());

    if (!::blob::inflate(data, size, *temp)) {
      ROS_WARN_NAMED("blob", "Error during decompression of a blob of size %u", size);
      return false;
    }

    set(temp);
    return true;
  }

private:
  bool compressed_;
  const value_type *pointer_;
  size_type size_;

  ConstBufferPtr copy_;
  mutable ConstBufferPtr compressed_blob_;
}; // struct Blob_

typedef ::blob::Blob_<std::allocator<void> > Blob;
typedef boost::shared_ptr< ::blob::Blob > BlobPtr;
typedef boost::shared_ptr< ::blob::Blob const> BlobConstPtr;

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::blob::Blob_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::blob::Blob_<ContainerAllocator> >::stream(s, "", v);
  return s;
}

} // namespace blob

namespace ros
{
namespace message_traits
{

template <class ContainerAllocator>
struct IsFixedSize< ::blob::Blob_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::blob::Blob_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::blob::Blob_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::blob::Blob_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::blob::Blob_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::blob::Blob_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::blob::Blob_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8115c3ed9d7b2e23c47c6ecaff2d4b13";
  }

  static const char* value(const ::blob::Blob_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8115c3ed9d7b2e23ULL;
  static const uint64_t static_value2 = 0xc47c6ecaff2d4b13ULL;
};

template<class ContainerAllocator>
struct DataType< ::blob::Blob_<ContainerAllocator> >
{
  static const char* value()
  {
    return "blob/Blob";
  }

  static const char* value(const ::blob::Blob_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::blob::Blob_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool compressed\n\
uint8[] data\n\
\n\
";
  }

  static const char* value(const ::blob::Blob_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::blob::Blob_<ContainerAllocator> >
  {
    template<typename Stream, typename T>
    inline static void write(Stream& stream, const T& t)
    {
      t.write(stream);
    }

    template<typename Stream, typename T>
    inline static void read(Stream& stream, T& t)
    {
      t.read(stream);
    }

    template<typename T>
    inline static uint32_t serializedLength(const T& t)
    {
      return t.serializedLength();
    }
  }; // struct Blob_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::blob::Blob_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::blob::Blob_<ContainerAllocator>& v)
  {
    s << indent << "(blob)";
  }
};

} // namespace message_operations
} // namespace ros

#endif // BLOB_MESSAGE_BLOB_H
