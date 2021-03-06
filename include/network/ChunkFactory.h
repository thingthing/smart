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
 * "P" for pcl::PointCloud::PointXYZRGBA \n
 * "L" for Landmarks::Landmark
 * @author Maxence
 * @version 1.0
 */
class   ChunkFactory
{
public:
  ChunkFactory();
  ~ChunkFactory();

  void  processData(const std::vector<Landmarks::Landmark*>&);
  void  processData(const Landmarks::Landmark&);
  void  processData(const pcl::PointCloud<pcl::PointXYZRGBA>&);

  // Getters
  bool  isFullChunkReady() const;
  bool  isChunkReady() const;

  std::string   getChunk();

private:
  void setTrueStringFromPoints(float data, std::string &stringPoints);
  void          pushTmpChunkToChunks();

  std::string   fromLandmarkToString(const Landmarks::Landmark&);

  int           calculateTotalPacketNeeded(const pcl::PointCloud<pcl::PointXYZRGBA>&);
  std::string   createPacketMetadata(unsigned int currentPacket,
                                     unsigned int totalPacket,
                                     std::string &packet
                                    );
  std::string   convertDataPacket(const pcl::PointCloud<pcl::PointXYZRGBA> &, unsigned int &cloudIndex);
  std::string   convertRangeOfPoint(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, unsigned int &cloudIndex, unsigned int nbOfPoint);
  std::string   fromPclPointToString(const pcl::PointXYZ&);
  std::string   fromPclPointRGBToString(const pcl::PointXYZRGBA& points);

  std::string   encodeNbIntoString(void*, unsigned long);

  void          increaseSizeChunks(unsigned int);
  void          decreaseSizeChunks(unsigned int);
  std::string   getNewChunkID();

  NON_COPYABLE(ChunkFactory)

  // ATTRIBUTES
  std::deque<std::string>       _chunks; /*!< contain all ready chunks, max size? */
  std::string                   _tmpChunk; /*!< fill it, push it to _chunks */

  unsigned int          _sizeChunks; /*!< Total size of chunks in _chunks */
  unsigned int          _chunkID; /*!< Used to give ID to chunks */
  unsigned int          _packetID; /*!< Used to give ID to packets */

  // CONST
  const unsigned int    MAX_SIZE_CHUNK = 512;
  const std::string     MAGIC_NB_LANDMARK = "L";
  const std::string     MAGIC_NB_POINTCLOUD = "P";
  const short           CHUNK_HEADER_SIZE = sizeof(char) + sizeof(_chunkID); // Magic Number + Chunk ID
  const unsigned int    POINTCLOUD_HEADER_SIZE = sizeof(unsigned int)   // [packet ID]
                           + sizeof(unsigned int)                       // [current packet nb]
                           + sizeof(unsigned int)                       // [total packet nb]
                           + sizeof(unsigned short);                    // [current packet's size];
  const unsigned int   SIZE_IN_PACKET = (MAX_SIZE_CHUNK - CHUNK_HEADER_SIZE) - POINTCLOUD_HEADER_SIZE;
  const unsigned short SIZE_OF_POINT_XYZ = 24;
};

} // end of namespace

#endif
