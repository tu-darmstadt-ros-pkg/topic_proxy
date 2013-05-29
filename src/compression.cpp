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

#include <topic_proxy/compression.h>
#include <stdint.h>
#include <vector>
#include <assert.h>

#ifdef HAVE_ZLIB

#include <zlib.h>

namespace topic_proxy {

  std::string Compression::getType() const {
    return "zlib";
  }

  bool Compression::compress(const std::vector<uint8_t> &in, std::vector<uint8_t> &out)
  {
    out.clear();

    const size_t BUFSIZE = 128 * 1024;
    uint8_t temp_buffer[BUFSIZE];

    z_stream strm;
    strm.zalloc = 0;
    strm.zfree = 0;
    strm.next_in = const_cast<uint8_t *>(in.data());
    strm.avail_in = in.size();
    strm.next_out = temp_buffer;
    strm.avail_out = BUFSIZE;

    deflateInit(&strm, Z_BEST_COMPRESSION);

    while (strm.avail_in != 0)
    {
      int res = deflate(&strm, Z_NO_FLUSH);
      assert(res == Z_OK || res == Z_STREAM_END);
      if (strm.avail_out == 0)
      {
        out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }
    }

    int deflate_res = Z_OK;
    while (deflate_res == Z_OK)
    {
      if (strm.avail_out == 0)
      {
        out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }

      deflate_res = deflate(&strm, Z_FINISH);
    }

    assert(deflate_res == Z_STREAM_END);
    out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
    deflateEnd(&strm);

    return true;
  }

  bool Compression::decompress(const std::vector<uint8_t> &in, std::vector<uint8_t> &out)
  {
    out.clear();

    const size_t BUFSIZE = 128 * 1024;
    uint8_t temp_buffer[BUFSIZE];

    z_stream strm;
    strm.zalloc = 0;
    strm.zfree = 0;
    strm.next_in = const_cast<uint8_t *>(in.data());
    strm.avail_in = in.size();
    strm.next_out = temp_buffer;
    strm.avail_out = BUFSIZE;

    inflateInit(&strm);

    while (strm.avail_in != 0)
    {
      int res = inflate(&strm, Z_NO_FLUSH);
      assert(res == Z_OK || res == Z_STREAM_END);
      if (strm.avail_out == 0)
      {
        out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }
    }

    int inflate_res = Z_OK;
    while (inflate_res == Z_OK)
    {
      if (strm.avail_out == 0)
      {
        out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }

      inflate_res = inflate(&strm, Z_FINISH);
    }

    assert(inflate_res == Z_STREAM_END);
    out.insert(out.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
    inflateEnd(&strm);

    return true;
  }

} // namespace topic_proxy

#else // HAVE_ZLIB

namespace topic_proxy {

  std::string Compression::getType() const {
    return std::string();
  }

  bool Compression::compress(const std::vector &, std::vector &)
  {
    return false;
  }

  bool Compression::decompress(const std::vector &, std::vector &)
  {
    return false;
  }

} // namespace topic_proxy

#endif // HAVE_ZLIB
