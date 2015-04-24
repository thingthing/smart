#ifndef     CHUNK_FACTORY_H_
# define    CHUNK_FACTORY_H_

#include <pcl/common/common.h>
#include "pcl/impl/point_types.hpp"
#include "slam/Landmarks.hh"
#include "utils/NonCopyable.h"

#include <vector>
#include <string>
#include <deque>

namespace Network
{

/*
 * Definition of a CHUNK
 *
 * The maximum Chunk's size is 512 Bytes
 *
 * Only contain complete class (no fragmentation)
 * (pcl::PointCloud::PointXYZ 'P' or Landmarks::Landmark 'L')
 *
 * int:    4 Bytes = 4 char
 * float:  4 Bytes = 4 char  // see the wiki/doc for the encoding process
 * double  8 Bytes = 8 char  // of float and double
 *
 * Exemple:
 *   L(cccciiiioooooooo...)P(cccciiiioooo)....
 *
 */

class   ChunkFactory
{
public:
  ChunkFactory();
  ~ChunkFactory();

  void  processData(const std::vector<Landmarks::Landmark>&);
  void  processData(const Landmarks::Landmark&);
  void  processData(const pcl::PointCloud<pcl::PointXYZ>&);

  // Getters
  bool  isFullChunkReady() const;
  bool  isChunkReady() const;

  std::string   getChunk();

private:
  void          pushChunkToChunks();
  void          addEncodedClassToChunk(const std::string&);

  std::string   fromLandmarkToString(const Landmarks::Landmark&);
  std::string   fromPclPointCloudToString(const pcl::PointCloud<pcl::PointXYZ>&);
  std::string   fromPclPointToString(const pcl::PointXY&);
  std::string   fromPclPointToString(const pcl::PointXYZ&);

  std::string   encodeNbIntoString(void*, unsigned long);

  void          increaseSizeChunks(unsigned int);
  void          decreaseSizeChunks(unsigned int);

  NON_COPYABLE(ChunkFactory)

  // ATTRIBUTES
  std::deque<std::string>       _chunks; // max size 1Mo?
  std::string                   _tmpChunk; // fill it, push it to _chunks

  bool          _fullChunkReadiness; // true: there is at least 1 chunk in _chunks
  bool          _chunkReadiness; // true: _tmpChunk is not empty neither full
  unsigned int  _sizeChunks;
  unsigned int  _maxSizeChunk; // can also depend on the MTU
};

} // end of namespace

#endif
