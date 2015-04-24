#ifndef     CHUNK_FACTORY_H_
# define    CHUNK_FACTORY_H_

#include "pcl/impl/point_types.hpp"
#include "slam/Landmarks.hh"
#include "utils/NonCopyable.h"

#include <vector>
#include <string>
#include <deque>

namespace Network { ChunkFactory; }

/*
 * Definition d'un CHUNK
 *
 * Un Chunk fait maximum 512 Bytes
 * Commence est termine par '[' ']'
 *
 * Contient uniquement des parties entieres d'objets
 * (pcl::PointXYZ 'P' ou Landmarks::Landmark 'L')
 *
 * int:    4 Bytes = 4 char
 * float:  4 Bytes = 4 char  // voir le wiki/doc pour l'encodage
 * double  8 Bytes = 8 char  // des float et des double
 *
 * Exemple:
 *   [L(cccc|cccc|cccccccc|...)P(cccc|cccc|cccc)....]
 *
 */

class   ChunkFactory
{
public:
  ChunkFactory();
  ~ChunkFactory();

  void  processData(const std::vector<Landmarks::Landmark>&);
  void  processData(const Landmarks::Landmark&);
  void  processData(const pcl::pointcloud<pcl::PointXYZ>&);
  void  processData(const pcl::pointcloud<pcl::PointXY>&);

  std::string   getChunk();

private:
  void          pushChunk(const std::string &);

  std::string   fromLandmarkToString(const Landmarks::Landmark&);
  std::string   fromPclPointToString(const pcl::pointcloud<pcl::PointXY>&);
  std::string   fromPclPointToStringRaw(const pcl::pointcloud<pcl::PointXY>&);
  std::string   fromPclPointToString(const pcl::pointcloud<pcl::PointXYZ>&);
  std::string   fromPclPointToStringRaw(const pcl::pointcloud<pcl::PointXYZ>&);

  std::string   fromIntToString(int);
  std::string   fromFloatToString(float);
  std::string   fromDoubleToString(double);

  NonCopyable(ChunkFactory);

  //-----------------------------
  std::deque<std::string>       chunks; // taille max 1Mo?

  bool          fullChunkReadiness;
  bool          chunkReadiness;
  unsigned int  sizeChunks;
};

#endif
