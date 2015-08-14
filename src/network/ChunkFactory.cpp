#include "network/ChunkFactory.h"

namespace Network
{

// PUBLIC

ChunkFactory::ChunkFactory():   _sizeChunks(0),
                                _chunkID(1),
                                _packetID(1) {}

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
 * @details Create the chunk header and add a landmark_ packet
 * to it using fromLandmarkToString().
 * Then it push the temporary chunk into the deque of chunks.
 * @param landmarks_ Class landmark_ containing informations about the captured points
 */
void ChunkFactory::processData(const Landmarks::Landmark& landmark_)
{
    _tmpChunk = MAGIC_NB_LANDMARK; // The Magic
    _tmpChunk += getNewChunkID(); // The Chunk ID
    _tmpChunk += fromLandmarkToString(landmark_); // The packet

    pushTmpChunkToChunks();
}

/**
 * @brief Convert a PointCloud of PointXYZ into packets of string(s) to put in chunk(s)
 * @details
 * @param points PointCloud containing only 3D point informations about the captured points
 */
void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXYZ >& pointCloud)
{
    unsigned int cloudIndex = 0;
    unsigned int packetDone = 0;
    unsigned int totalPacketNeeded;
    std::string packet;
    std::string metadataPacket;

    totalPacketNeeded = calculateTotalPacketNeeded(pointCloud);

    // First Packet
    packet = convertDataFirstPacket(pointCloud, cloudIndex);
    metadataPacket = createPacketMetadata(++packetDone, totalPacketNeeded, packet);
    _tmpChunk = MAGIC_NB_POINTCLOUD; // The Magic
    _tmpChunk += getNewChunkID(); // The Chunk ID
    _tmpChunk += metadataPacket + packet; // The Packet
    pushTmpChunkToChunks();

    // Other Packets (if needed)
    while (packetDone < totalPacketNeeded)
    {
        packet = convertDataPacket(pointCloud, cloudIndex);
        metadataPacket = createPacketMetadata(++packetDone, totalPacketNeeded, packet);
        _tmpChunk = MAGIC_NB_POINTCLOUD; // The Magic
        _tmpChunk += getNewChunkID(); // The Chunk ID
        _tmpChunk += metadataPacket + packet; // The Packet
        pushTmpChunkToChunks();
    }
}

// Getters
/**
 * @brief Return whether _chunks is empty or not
 * @return A boolean to tell you whether _chunks is empty or not.
 */
bool ChunkFactory::isFullChunkReady() const { return (_chunks.size() == 0)?(false):(true); }

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
 * @brief Push the temporary chunk into the deque _chunks
 * @details If not empty, put the temporary chunk into the deque of
 * chunks and clear it. It also call increaseSizeChunks()
 */
void ChunkFactory::pushTmpChunkToChunks()
{
    if (_tmpChunk.size() > 0)
    {
        _chunks.push_front(_tmpChunk);
        increaseSizeChunks(_tmpChunk.size());
        _tmpChunk.erase();
    }
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

/*
 * Calculate the total number of packet we need to do to send the cloud
 */
int ChunkFactory::calculateTotalPacketNeeded(const pcl::PointCloud< pcl::PointXYZ >& cloud)
{
    unsigned int        totalPacketNeeded = 1;
    unsigned int        totalNbOfPoint = cloud.size();
    unsigned int        nbOfPointInFirstPacket;
    unsigned int        nbOfPointInPacket;

    nbOfPointInFirstPacket = (SIZE_IN_PACKET - sizeof(cloud.width) - sizeof(cloud.height)) / SIZE_OF_POINT_XYZ;
    nbOfPointInPacket = SIZE_IN_PACKET / SIZE_OF_POINT_XYZ;

    totalPacketNeeded = (totalNbOfPoint - nbOfPointInFirstPacket) / nbOfPointInPacket;

    if ((totalNbOfPoint - nbOfPointInFirstPacket) % nbOfPointInPacket > 1)
        ++totalPacketNeeded;

    return totalPacketNeeded;
}

std::string ChunkFactory::createPacketMetadata(unsigned int currentPacket, unsigned int totalPacket, std::string& packet)
{
    unsigned short      packetSize = (unsigned short)(packet.size());
    std::string         metadata = "";

    metadata += encodeNbIntoString((void*)&(_packetID), sizeof(_packetID));
    metadata += encodeNbIntoString((void*)&(currentPacket), sizeof(currentPacket));
    metadata += encodeNbIntoString((void*)&(totalPacket), sizeof(totalPacket));
    metadata += encodeNbIntoString((void*)&(packetSize), sizeof(packetSize));

    return metadata;
}

std::string ChunkFactory::convertDataFirstPacket(const pcl::PointCloud< pcl::PointXYZ >& cloud, unsigned int& cloudIndex)
{
    std::string packetData = "";
    unsigned int nbOfPoint;

    packetData += encodeNbIntoString((void*)&(cloud.width), sizeof(cloud.width));
    packetData += encodeNbIntoString((void*)&(cloud.height), sizeof(cloud.height));

    nbOfPoint = (SIZE_IN_PACKET - sizeof(cloud.width) - sizeof(cloud.height)) / SIZE_OF_POINT_XYZ;

    packetData += convertRangeOfPoint(cloud, cloudIndex, nbOfPoint);

    return packetData;
}

std::string ChunkFactory::convertDataPacket(const pcl::PointCloud< pcl::PointXYZ >& cloud, unsigned int& cloudIndex)
{
    std::string packetData = "";
    unsigned int nbOfPoint;

    nbOfPoint = SIZE_IN_PACKET / SIZE_OF_POINT_XYZ;

    packetData += convertRangeOfPoint(cloud, cloudIndex, nbOfPoint);

    return packetData;
}

std::string ChunkFactory::convertRangeOfPoint(const pcl::PointCloud< pcl::PointXYZ >& cloud, unsigned int& cloudIndex, unsigned int nbOfPoint)
{
    std::string convertedPoints = "";
    unsigned int i;

    for (i = 0; i < nbOfPoint; ++i)
        convertedPoints += fromPclPointToString(cloud[cloudIndex + i]);

    cloudIndex += i;
    return convertedPoints;
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

/// @brief Return a new Chunk ID in a string and increase it
std::string ChunkFactory::getNewChunkID()
{
    std::string tmpChunkID = encodeNbIntoString((void*) &(_chunkID), sizeof(_chunkID));
    ++_chunkID;

    return tmpChunkID;
}

} // end of namespace
