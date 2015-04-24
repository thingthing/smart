#include "network/ChunkFactory.h"

namespace Network
{

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

ChunkFactory::ChunkFactory():   _fullChunkReadiness(false),
                                _chunkReadiness(false),
                                _sizeChunks(0) {}

// TODO clean the deque if needed?
ChunkFactory::~ChunkFactory() {}

void ChunkFactory::processData(const std::vector< Landmarks::Landmark >& landmarks_)
{
    std::vector<Landmarks::Landmark>::const_iterator it = landmarks_.begin();
    for ( ; it != landmarks_.end(); ++it)
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
bool ChunkFactory::isFullChunkReady() const { return _fullChunkReadiness; }
bool ChunkFactory::isChunkReady() const     { return _chunkReadiness; }

std::string ChunkFactory::getChunk()
{
    std::string chunk = "";

    if (_chunks.size() != 0)
    {
        chunk = _chunks.back();
        _chunks.pop_back();
        decreaseSizeChunks(chunk.size());
    }
    return chunk;
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
    return "";
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXY
std::string ChunkFactory::fromPclPointToString(const pcl::PointCloud< pcl::PointXY >& points)
{
    // TODO
    return "";
}

// Return a string without 'P('.....')' FOR PointXY
std::string ChunkFactory::fromPclPointToStringRaw(const pcl::PointCloud< pcl::PointXY >& points)
{
    // TODO
    return "";
}

// Return a string like "P(cccc|cccc|cccccccc|....)" FOR PointXYZ
std::string ChunkFactory::fromPclPointToString(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO
    return "";
}

// Return a string without 'P('.....')' FOR PointXYZ
std::string ChunkFactory::fromPclPointToStringRaw(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO
    return "";
}

std::string ChunkFactory::fromIntToString(int nb)
{
    // TODO
    return "";
}

std::string ChunkFactory::fromFloatToString(float nb)
{
    // TODO
    return "";
}

std::string ChunkFactory::fromDoubleToString(double nb)
{
    // TODO
    return "";
}

void ChunkFactory::increaseSizeChunks(unsigned int cSize) { _sizeChunks += cSize; }
void ChunkFactory::decreaseSizeChunks(unsigned int cSize) { _sizeChunks -= cSize; }

} // end of namespace
