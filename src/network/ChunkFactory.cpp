#include "network/ChunkFactory.h"

/*
 * Structure de "pcl::PointXYZ structure" 3 float
 *      float x
 *      float y
 *      float z // non disponible dans "pcl::PointXY"
 */

/*
 * Structure de "Landmarks::Landmark" 1 PointXY, 1 PointXYZ, 3 int, 6 double
 *  pcl::PointXY pos;
 *  pcl::PointXYZ robotPos;
 *  int id;
 *  int life;
 *  int totalTimeObserved;
 *  double bearing;
 *  double range;
 *  double a;
 *  double b;
 *  double rangeError;
 *  double bearingError;
 */


// PUBLIC

chunkFactory::chunkFactory():   isFullChunkReady(false),
                                isChunkReady(false),
                                sizeChunks(0) {}

chunkFactory::~chunkFactory() {}

void chunkFactory::processData(const std::vector< Landmarks::Landmark >& landmarks_)
{
    std::vector<Landmarks::Landmark>::iterator it = landmarks_.begin();
    for (it; it != landmarks_.end(); ++it)
        processData(*it);
}

void chunkFactory::processData(const Landmarks::Landmark& landmark_)
{
    // TODO
}

void chunkFactory::processData(const pcl::pointcloud< pcl::PointXYZ >& points)
{
    // TODO
}

void chunkFactory::processData(const pcl::pointcloud< pcl::PointXY >& points)
{
    // TODO
}

std::string chunkFactory::getChunk()
{
    // TODO
}

// PRIVATE

void chunkFactory::pushChunk(const std::string& chunk)
{
    // TODO
    // update sizeChunks
}

// Return a string like "L(cccc|cccc|cccccccc|....)"
std::string chunkFactory::fromLandmarkToString(const Landmarks::Landmark& landmark_)
{
    // TODO
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXY
std::string chunkFactory::fromPclPointToString(const pcl::pointcloud< pcl::PointXY >& points)
{
    // TODO
}

// Return a string without 'P('.....')' FOR PointXY
std::string chunkFactory::fromPclPointToStringRaw(const pcl::pointcloud< pcl::PointXY >& points)
{
    // TODO
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXYZ
std::string chunkFactory::fromPclPointToString(const pcl::pointcloud< pcl::PointXYZ >& points)
{
    // TODO
}

// Return a string without 'P('.....')' FOR PointXYZ
std::string chunkFactory::fromPclPointToStringRaw(const pcl::pointcloud< pcl::PointXYZ >& points)
{
    // TODO
}

std::string chunkFactory::fromIntToString(int nb)
{
    // TODO
}

std::string chunkFactory::fromFloatToString(float nb)
{
    // TODO
}

std::string chunkFactory::fromDoubleToString(double nb)
{
    // TODO
}
