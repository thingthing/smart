#include "network/ChunkFactory.h"

namespace Network
{

// PUBLIC

ChunkFactory::ChunkFactory():   _fullChunkReadiness(false),
                                _chunkReadiness(false),
                                _sizeChunks(0),
                                _maxSizeChunk(512) {}

ChunkFactory::~ChunkFactory()
{
    _chunks.clear();
}

/**
 * @brief Convert a vector of Landmark into string to put it in chunk(s)
 * @details Iterates on a vector of Landmarks using processData()
 * @param landmarks_ Vector of landmark_ containing informations about the captured points
 */
void ChunkFactory::processData(const std::vector< Landmarks::Landmark >& landmarks_)
{
    std::vector<Landmarks::Landmark>::const_iterator it;
    for (it = landmarks_.begin() ; it != landmarks_.end(); ++it)
        processData(*it);
}

/**
 * @brief Convert a Landmark into string to put it in chunk
 * @details Call fromLandmarkToString() to get a string of information from the
 * class landmark_ and surround it with "L(" ")" to tag it as a landmark packet.
 * Then it add the packet to the current built chunk with addEncodedClassToChunk()
 * @param landmarks_ Class landmark_ containing informations about the captured points
 */
void ChunkFactory::processData(const Landmarks::Landmark& landmark_)
{
    addEncodedClassToChunk("L(" + fromLandmarkToString(landmark_) + ")");
}

/**
 * @brief Convert a PointCloud of PointXYZ into string(s) to put it in chunk(s)
 * @details Call fromPclPointCloudToString() to get a string of information
 * from the class PointCloud and surround it with "P(" ")" to tag it as a
 * PointCloud packet. Then it add the packet to the current built chunk
 * with addEncodedClassToChunk().
 * @param points PointCloud containing only 3D point informations about the captured points
 */
void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXYZ >& points)
{
    addEncodedClassToChunk("P(" + fromPclPointCloudToString(points) + ")");
}

// Getters
/**
 * @brief Getter of _fullChunkReadiness
 * @return A boolean to tell you weither a totally filled chunk is ready to sent or not.
 */
bool ChunkFactory::isFullChunkReady() const { return _fullChunkReadiness; }
/**
 * @brief Getter of _chunkReadiness
 * @return A boolean to tell you weither a partially filled chunk is ready to sent or not.
 */
bool ChunkFactory::isChunkReady() const     { return _chunkReadiness; }

/**
 * @brief Get a chunk already prepared by ChunkFactory
 * @return A chunk (string) ready to sent through UDP. If there is no chunk prepared, it will
 * send an empty string.
 */
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

/**
 * @brief Add a chunk to the deque of chunk if not empty.
 * @details If not empty, put the temporary chunk into the deque of chunks
 * and clear it. It also set _chunkReadiness to false, and call increaseSizeChunks()
 */
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

/**
 * @brief Get an encoded string and add it to the temporary chunk
 * @details It pushes the temporary chunk into the deque of chunk if needed
 * using pushChunkToChunks(). Then it add an encoded string to the temporary
 * chunk _tmpChunk and set _chunkReadiness to true
 * @param encodedClass 3D informations already converted into string
 */
void ChunkFactory::addEncodedClassToChunk(const std::string& encodedClass)
{
    if (_tmpChunk.size() + encodedClass.size() > _maxSizeChunk)
        pushChunkToChunks();

    _tmpChunk += encodedClass;
    if (!_chunkReadiness)
        _chunkReadiness = true;
}

/**
 * @brief Convert the class Landmark into a string
 * @details The chosen variable converted into the string are:
 * pos, robotPos, id, life, totalTimeObserved, bearing, range.
 * @param landmark_ Class containing 3D information that we have to convert
 * @return Return a string containing chosen information of the class Landmark
 */
std::string ChunkFactory::fromLandmarkToString(const Landmarks::Landmark& landmark_)
{
    std::string strLandmark = "";

    strLandmark += fromPclPointToString(landmark_.pos);
    strLandmark += fromPclPointToString(landmark_.robotPos);
    strLandmark += encodeNbIntoString((void*)&(landmark_.id), sizeof(landmark_.id));
    strLandmark += encodeNbIntoString((void*)&(landmark_.life), sizeof(landmark_.life));
    strLandmark += encodeNbIntoString((void*)&(landmark_.totalTimeObserved), sizeof(landmark_.totalTimeObserved));
    strLandmark += encodeNbIntoString((void*)&(landmark_.bearing), sizeof(landmark_.bearing));
    strLandmark += encodeNbIntoString((void*)&(landmark_.range), sizeof(landmark_.range));

    return strLandmark;
}

/**
 * @brief Convert the PointCloud into strings to put it in chunk(s)
 * @todo This function has to be rework to be able to handle a lot a point.
 */
std::string ChunkFactory::fromPclPointCloudToString(const pcl::PointCloud< pcl::PointXYZ >& pointCloud)
{
    std::vector< pcl::PointXYZ, Eigen::aligned_allocator < pcl::PointXYZ > >::const_iterator it;
    std::string encodedString = "";

    encodedString += encodeNbIntoString((void*)&(pointCloud.width), sizeof(pointCloud.width));
    encodedString += encodeNbIntoString((void*)&(pointCloud.height), sizeof(pointCloud.height));

    for (it = pointCloud.begin() ; it != pointCloud.end() ; ++it)
        encodedString += fromPclPointToString((*it));

    return encodedString;
}

/**
 * @todo DEPRECATED To remove, PointXY will not be used. Only PointXYZ
 */
std::string ChunkFactory::fromPclPointToString(const pcl::PointXY& points)
{
    std::string stringPoints = "";

    stringPoints += encodeNbIntoString((void*)&(points.x), sizeof(points.x));
    stringPoints += encodeNbIntoString((void*)&(points.y), sizeof(points.y));

    return stringPoints;
}

/**
 * @brief Convert PointXYZ into a string
 * @details It convert in a string the variable x, y and z
 * So the length of the string would be sizeof(x) + sizeof(y) + sizeof(z)
 * @return An encoded string containing PointXYZ informations
 */
std::string ChunkFactory::fromPclPointToString(const pcl::PointXYZ& points)
{
    std::string stringPoints = "";

    stringPoints += encodeNbIntoString((void*)&(points.x), sizeof(points.x));
    stringPoints += encodeNbIntoString((void*)&(points.y), sizeof(points.y));
    stringPoints += encodeNbIntoString((void*)&(points.z), sizeof(points.z));

    return stringPoints;
}

/**
 * @brief Convert any atomic variable into a string
 * @details Use a generic pointer to be able to read any function as a character string.
 * Then it uses the nbOfByte (which is a sizeof() of the variable) to read correctly Byte by
 * Byte the variable and add it to encoded (a string).
 * @param *nb A generic pointer pointing on the variable you want to encode
 * @param nbOfByte A sizeof() of the variable you want to encode
 * @return A string containing the bytes informations of the variable
 */
std::string ChunkFactory::encodeNbIntoString(void* nb, unsigned long nbOfByte)
{
  char* nb_ = (char*)nb;
  std::string encoded = "";

  for (unsigned long i = 0; i < nbOfByte; ++i)
    {
      char tmp = 0;
      tmp |= nb_[i];
      encoded += tmp;
    }
  //  std::cout << std::hex << encoded;
  return encoded;
}

/// @brief Add the size of the added chunk into the deque of chunks _chunks
void ChunkFactory::increaseSizeChunks(unsigned int cSize) { _sizeChunks += cSize; }
/// @brief Subtract the size of the removed chunk from the deque of chunks _chunks
void ChunkFactory::decreaseSizeChunks(unsigned int cSize) { _sizeChunks -= cSize; }

} // end of namespace
