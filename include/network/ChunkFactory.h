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

/**
 * @class ChunkFactory
 * @brief Split 3D information into chunks to sent it through UDP.
 * @details The maximum size of a chunk is 512 Bytes.
 * The classes encoded are tagged with: \n
 * "P" for pcl::PointCloud::PointXYZ \n
 * "L" for Landmarks::Landmark
 * @author Maxence
 * @version 0.3
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
  std::deque<std::string>       _chunks; /*!< contain all ready chunks, max size? */
  std::string                   _tmpChunk; /*!< fill it, push it to _chunks */

  bool          _fullChunkReadiness; /*!< true: there is at least 1 chunk in _chunks */
  bool          _chunkReadiness; /*!< true: _tmpChunk is not empty neither full */
  unsigned int  _sizeChunks; /*!< Total size of chunks in _chunks */
  unsigned int  _maxSizeChunk; /*!< could also depend on the MTU */
  unsigned long long    _chunkID; /*!< Used to give ID to chunks */
};

} // end of namespace

#endif
