#include "network/ChunkFactory.h"

/*
 * Class used :
 * "pcl::PointCloud::PointXYZ"
 *      float x
 *      float y
 *      float z // not avaible in "pcl::PointCloud::PointXY"
 *
 * "Landmarks::Landmark"
 *  pcl::PointCloud::PointXY pos;
 *  pcl::PointCloud::PointXYZ robotPos;
 *  int id;
 *  int life;
 *  int totalTimeObserved;
 *  double bearing;
 *  double range;
 */


// PUBLIC

ChunkFactory::ChunkFactory():   fullChunkReadiness(false),
                                chunkReadiness(false),
                                sizeChunks(0) {}

// TODO clean the deque if needed?
ChunkFactory::~ChunkFactory() {}

void ChunkFactory::processData(const std::vector< Landmarks::Landmark >& landmarks_)
{
    std::vector<Landmarks::Landmark>::iterator it = landmarks_.begin();
    for (it; it != landmarks_.end(); ++it)
        processData(*it);
}

void ChunkFactory::processData(const Landmarks::Landmark& landmark_)
{
    // TODO
}

void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO
}

void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXY >& points)
{
    // TODO
}

// Getters
bool ChunkFactory::isFullChunkReady() const { return fullChunkReadiness; }
bool ChunkFactory::isChunkReady() const     { return chunkReadiness; }

std::string ChunkFactory::getChunk()
{
    // TODO
}

// PRIVATE

void ChunkFactory::pushChunk(const std::string& chunk)
{
    // TODO
    // update sizeChunks
}

// Return a string like "L(cccc|cccc|cccccccc|....)"
std::string ChunkFactory::fromLandmarkToString(const Landmarks::Landmark& landmark_)
{
    // TODO
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXY
std::string ChunkFactory::fromPclPointToString(const pcl::PointCloud< pcl::PointXY >& points)
{
    // TODO
}

// Return a string without 'P('.....')' FOR PointXY
std::string ChunkFactory::fromPclPointToStringRaw(const pcl::PointCloud< pcl::PointXY >& points)
{
    // TODO
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXYZ
std::string ChunkFactory::fromPclPointToString(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO
}

// Return a string without 'P('.....')' FOR PointXYZ
std::string ChunkFactory::fromPclPointToStringRaw(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO
}

std::string ChunkFactory::fromIntToString(int nb)
{
    // TODO
}

std::string ChunkFactory::fromFloatToString(float nb)
{
    // TODO
}

std::string ChunkFactory::fromDoubleToString(double nb)
{
    // TODO
}
