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
 * It begins and finish with '[' ']'
 *
 * Only contain complete class (no fragmentation)
 * (pcl::PointCloud::PointXYZ 'P' or Landmarks::Landmark 'L')
 *
 * int:    4 Bytes = 4 char
 * float:  4 Bytes = 4 char  // see the wiki/doc for the encoding process
 * double  8 Bytes = 8 char  // of float and double
 *
 * Exemple:
 *   [L(cccciiiioooooooo...)P(cccciiiioooo)....]
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
  void  processData(const pcl::PointCloud<pcl::PointXY>&);

  bool  isFullChunkReady() const;
  bool  isChunkReady() const;

  std::string   getChunk();

private:
  void          pushChunk(const std::string &);

  std::string   fromLandmarkToString(const Landmarks::Landmark&);
  std::string   fromPclPointToString(const pcl::PointCloud<pcl::PointXY>&);
  std::string   fromPclPointToStringRaw(const pcl::PointCloud<pcl::PointXY>&);
  std::string   fromPclPointToString(const pcl::PointCloud<pcl::PointXYZ>&);
  std::string   fromPclPointToStringRaw(const pcl::PointCloud<pcl::PointXYZ>&);

  std::string   fromIntToString(int);
  std::string   fromFloatToString(float);
  std::string   fromDoubleToString(double);

  void          increaseSizeChunks(unsigned int);
  void          decreaseSizeChunks(unsigned int);

  NON_COPYABLE(ChunkFactory)

  // ATTRIBUTES
  std::deque<std::string>       _chunks; // taille max 1Mo?
  std::string                   _tmpChunk; // fill it, push it to _chunks

  bool          _fullChunkReadiness;
  bool          _chunkReadiness;
  unsigned int  _sizeChunks;
  unsigned int  _maxSizeChunk; // can also depend on the MTU
};

} // end of namespace

#endif
