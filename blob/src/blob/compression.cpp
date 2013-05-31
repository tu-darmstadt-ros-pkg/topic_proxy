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

#include <blob/compression.h>

#ifndef HAVE_BZIP2
  #define HAVE_BZIP2
#endif

#ifdef HAVE_BZIP2

#include <bzlib.h>

namespace blob
{
  static const std::size_t CHUNK_SIZE = 10 * 1024; // 10k chunks
  static const int VERBOSITY = 1;

  bool compressionAvailable() { return true; }

  bool deflate(const uint8_t *data, uint32_t size, std::vector<uint8_t>& deflated)
  {
    static int BLOCK_SIZE_100K = 5;
    deflated.clear();

    bz_stream stream;
    stream.next_in   = reinterpret_cast<char *>(const_cast<uint8_t *>(data));
    stream.avail_in  = size;
    stream.bzalloc   = NULL;
    stream.bzfree    = NULL;
    stream.opaque    = NULL;

    if (BZ2_bzCompressInit(&stream, BLOCK_SIZE_100K, 0, 0) != BZ_OK) {
      return false;
    }

    deflated.resize(CHUNK_SIZE);
    stream.next_out  = reinterpret_cast<char *>(deflated.data());
    stream.avail_out = deflated.size();

    int result = BZ_RUN_OK;
    int state  = BZ_RUN;
    while(result == BZ_RUN_OK || result == BZ_FLUSH_OK || result == BZ_FINISH_OK) {
      if (stream.avail_in == 0) {
        state = BZ_FINISH;
      }

      if (stream.avail_out == 0) {
        deflated.resize(deflated.size() + CHUNK_SIZE);
        stream.next_out  = reinterpret_cast<char *>(deflated.data() + deflated.size() - CHUNK_SIZE);
        stream.avail_out = CHUNK_SIZE;
      }

      result = BZ2_bzCompress(&stream, state);
    }

    if (result != BZ_STREAM_END) {
      deflated.clear();
      BZ2_bzCompressEnd(&stream);
      return false;
    }

    deflated.resize(deflated.size() - stream.avail_out);
    BZ2_bzCompressEnd(&stream);
    return true;
  }

  bool inflate(const uint8_t *data, uint32_t size, std::vector<uint8_t>& inflated)
  {
    static const int SMALL = 1;
    inflated.clear();

    bz_stream stream;
    stream.next_in   = reinterpret_cast<char *>(const_cast<uint8_t *>(data));
    stream.avail_in  = size;
    stream.bzalloc   = NULL;
    stream.bzfree    = NULL;
    stream.opaque    = NULL;

    if (BZ2_bzDecompressInit(&stream, VERBOSITY, SMALL) != BZ_OK) {
      return false;
    }

    inflated.resize(CHUNK_SIZE);
    stream.next_out  = reinterpret_cast<char *>(inflated.data());
    stream.avail_out = inflated.size();

    int result = BZ_OK;
    while(result == BZ_OK || result == BZ_FLUSH_OK || result == BZ_FINISH_OK) {
      if (stream.avail_out == 0) {
        inflated.resize(inflated.size() + CHUNK_SIZE);
        stream.next_out  = reinterpret_cast<char *>(inflated.data() + inflated.size() - CHUNK_SIZE);
        stream.avail_out = CHUNK_SIZE;
      }

      result = BZ2_bzDecompress(&stream);
    }

    if (result != BZ_STREAM_END) {
      inflated.clear();
      BZ2_bzDecompressEnd(&stream);
      return false;
    }

    inflated.resize(inflated.size() - stream.avail_out);
    BZ2_bzDecompressEnd(&stream);
    return true;
  }

} // namespace blob

#else // HAVE_BZIP2

namespace blob
{
  bool compressionAvailable() { return false; }
  bool deflate(const uint8_t *data, uint32_t size, std::vector<uint8_t>& deflated) { return false; }
  bool inflate(const uint8_t *data, uint32_t size, std::vector<uint8_t>& inflated) { return false; }
}

#endif // HAVE_BZIP2
