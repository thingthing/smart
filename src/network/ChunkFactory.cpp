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
                                _sizeChunks(0),
                                _maxSizeChunk(512) {}

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
    addEncodedClassToChunk("L(" + fromLandmarkToString(landmark_) + ")");
}

// ATTENTION must convert PointCloud in all PointXYZ first
void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    // TODO do a loop on each point of the cloud?
    // addEncodedClassToChunk("P(" + fromPclPointToString(points) + ")");
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

void ChunkFactory::pushChunkToChunks()
{
    unsigned int chunkSize = _tmpChunk.size();

    if (chunkSize > 0)
    {
        _chunks.push_front(_tmpChunk);
        increaseSizeChunks(chunkSize);
        _tmpChunk.erase();
        _chunkReadiness = false;
    }
}

void ChunkFactory::addEncodedClassToChunk(const std::string& encodedClass)
{
    if (_tmpChunk.size() + encodedClass.size() > _maxSizeChunk)
        pushChunkToChunks();

    _tmpChunk += encodedClass;
    if (!_chunkReadiness)
        _chunkReadiness = true;
}

// Return a string like "cccciiiioooooooo"
std::string ChunkFactory::fromLandmarkToString(const Landmarks::Landmark& landmark_)
{
    std::string stringLandmark = "";

/*
 * ATTENTION "pos" and "robotPos" are cloud of points!
 * pcl::PointCloud::PointXY pos;
 * pcl::PointCloud::PointXYZ robotPos;
*/
    stringLandmark += fromPclPointToString(landmark_.pos);
    stringLandmark += fromPclPointToString(landmark_.robotPos);
    stringLandmark += fromIntToString(landmark_.id);
    stringLandmark += fromIntToString(landmark_.life);
    stringLandmark += fromIntToString(landmark_.totalTimeObserved);
    stringLandmark += fromDoubleToString(landmark_.bearing);
    stringLandmark += fromDoubleToString(landmark_.range);
    return stringLandmark;
}

std::string ChunkFactory::fromPclPointCloudToString(const pcl::PointCloud< pcl::PointXYZ >& pointCloud)
{
    // TODO
    return "";
}

// Return a string like "cccciiii" FOR PointXY
std::string ChunkFactory::fromPclPointToString(const pcl::PointXY& points)
{
    std::string stringPoints = "";

    stringPoints += fromFloatToString(points.x);
    stringPoints += fromFloatToString(points.y);

    return stringPoints;
}

// Return a string like "cccciiiioooo" FOR PointXYZ
std::string ChunkFactory::fromPclPointToString(const pcl::PointXYZ& points)
{
    std::string stringPoints = "";

    stringPoints += fromFloatToString(points.x);
    stringPoints += fromFloatToString(points.y);
    stringPoints += fromFloatToString(points.z);

    return stringPoints;
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
