#include <netinet/in.h>
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
void ChunkFactory::processData(const std::vector< Landmarks::Landmark* >& landmarks_)
{
    std::vector<Landmarks::Landmark*>::const_iterator it;
    for (it = landmarks_.begin() ; it != landmarks_.end(); ++it)
    {
        if (*it != NULL)
            processData((Landmarks::Landmark&)(*(*it))); // Get the reference from the pointer.
    }
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
 * @brief Convert a PointCloud of PointXYZRGBA into packets of string(s) to put in chunk(s)
 * @details
 * @param points PointCloud containing only 3D point informations about the captured points
 */
void ChunkFactory::processData(const pcl::PointCloud< pcl::PointXYZRGBA >& pointCloud)
{
    unsigned int cloudIndex = 0;
    unsigned int packetDone = 0;
    unsigned int totalPacketNeeded;
    std::string packet;
    std::string metadataPacket;

    totalPacketNeeded = calculateTotalPacketNeeded(pointCloud);

    while (packetDone < totalPacketNeeded)
    {
        packet = convertDataPacket(pointCloud, cloudIndex);
        metadataPacket = createPacketMetadata(++packetDone, totalPacketNeeded, packet);
        _tmpChunk = MAGIC_NB_POINTCLOUD; // The Magic
        _tmpChunk += getNewChunkID(); // The Chunk ID
        _tmpChunk += metadataPacket + packet; // The Packet
        pushTmpChunkToChunks();
    }
    ++_packetID;
}

// Getters
/**
 * @brief Return whether _chunks is empty or not
 * @return A boolean to tell you whether _chunks is empty or not.
 */
bool ChunkFactory::isFullChunkReady() const { return (_chunks.size() == 0) ? (false) : (true); }

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
    strLandmark += encodeNbIntoString((void*) & (landmark_.id), sizeof(landmark_.id));
    strLandmark += encodeNbIntoString((void*) & (landmark_.life), sizeof(landmark_.life));
    strLandmark += encodeNbIntoString((void*) & (landmark_.totalTimeObserved), sizeof(landmark_.totalTimeObserved));
    strLandmark += encodeNbIntoString((void*) & (landmark_.bearing), sizeof(landmark_.bearing));
    strLandmark += encodeNbIntoString((void*) & (landmark_.range), sizeof(landmark_.range));

    return strLandmark;
}

/*
 * Calculate the total number of packet we need to do to send the cloud
 */
int ChunkFactory::calculateTotalPacketNeeded(const pcl::PointCloud< pcl::PointXYZRGBA >& cloud)
{
    unsigned int        totalPacketNeeded = 0;
    unsigned int        totalNbOfPoint = cloud.size();
    unsigned int        nbOfPointInPacket;

    nbOfPointInPacket = SIZE_IN_PACKET / SIZE_OF_POINT_XYZ;
    totalPacketNeeded = (totalNbOfPoint / nbOfPointInPacket);

    return ((totalNbOfPoint % nbOfPointInPacket) > 0) ? (++totalPacketNeeded) : (totalPacketNeeded) ;
}

std::string ChunkFactory::createPacketMetadata(unsigned int currentPacket, unsigned int totalPacket, std::string& packet)
{
    unsigned short      packetSize = (unsigned short)(packet.size());
    std::string         metadata = "";

    metadata += encodeNbIntoString((void*) & (_packetID), sizeof(_packetID));
    metadata += encodeNbIntoString((void*) & (currentPacket), sizeof(currentPacket));
    metadata += encodeNbIntoString((void*) & (totalPacket), sizeof(totalPacket));
    metadata += encodeNbIntoString((void*) & (packetSize), sizeof(packetSize));

    return metadata;
}

std::string ChunkFactory::convertDataPacket(const pcl::PointCloud< pcl::PointXYZRGBA >& cloud, unsigned int& cloudIndex)
{
    std::string packetData = "";
    unsigned int nbOfPoint;

    nbOfPoint = SIZE_IN_PACKET / SIZE_OF_POINT_XYZ;

    packetData += convertRangeOfPoint(cloud, cloudIndex, nbOfPoint);

    return packetData;
}

std::string ChunkFactory::convertRangeOfPoint(const pcl::PointCloud< pcl::PointXYZRGBA >& cloud, unsigned int& cloudIndex, unsigned int nbOfPoint)
{
    std::string convertedPoints = "";
    unsigned int i;

    for (i = 0; i < nbOfPoint; ++i)
        convertedPoints += fromPclPointRGBToString(cloud[cloudIndex + i]);

    cloudIndex += i;
    return convertedPoints;
}

int dumper4(const void* target)
{
    int* tmp = (int*)target;
    return (int)(*tmp);
}

long long dumper8(const void* target)
{
    long long* tmp = (long long*)target;
    return (long long)(*tmp);
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

    //std::cout << points.x << " " << points.y << " " << points.z << std::endl;
    // std::cout << std::hex << dumper4(&points.x) << " " << dumper4(&points.y) << " " << dumper4(&points.z) << std::endl;

    stringPoints += encodeNbIntoString((void*) & (points.x), sizeof(points.x));
    stringPoints += encodeNbIntoString((void*) & (points.y), sizeof(points.y));
    stringPoints += encodeNbIntoString((void*) & (points.z), sizeof(points.z));

    return stringPoints;
}

void ChunkFactory::setTrueStringFromPoints(float data, std::string &stringPoints) {
    // float default_value = 0;
    // if (data == data)
  stringPoints += encodeNbIntoString((void*) & (data), sizeof(data));
    // else
    //     stringPoints += encodeNbIntoString((void*) & (default_value), sizeof(float));
    //std::cerr << "data in float "<< data << std::endl;
}
/**
 * @brief Convert PointXYZRGBA into a string
 * @details It convert in a string the variable x, y and z
 * So the length of the string would be sizeof(x) + sizeof(y) + sizeof(z)
 * @return An encoded string containing PointXYZRGBA informations
 */
std::string ChunkFactory::fromPclPointRGBToString(const pcl::PointXYZRGBA& points)
{
    std::string stringPoints = "";

    //std::cerr << points.x << " " << points.y << " " << points.z << std::endl;

    // std::cerr << (int)points.r << " " << (int)points.g << " " << (int)points.b << std::endl;
    // std::cerr << "size of points " << sizeof((float)points.r) << sizeof(uint8_t) << std::endl;
    // std::cout << std::hex << dumper4(&points.x) << " " << dumper4(&points.y) << " " << dumper4(&points.z) << std::endl;
    // std::cerr << "size of point.r == " << sizeof(float)  << std::endl;
    // std::cerr << "size of cloud == " << sizeof(points)  << std::endl;
    //To do to check if points.x is set, strange but that how it works
    setTrueStringFromPoints(points.x, stringPoints);
    setTrueStringFromPoints(points.y, stringPoints);
    setTrueStringFromPoints(points.z, stringPoints);
    setTrueStringFromPoints(points.r, stringPoints);
    setTrueStringFromPoints(points.g, stringPoints);
    setTrueStringFromPoints(points.b, stringPoints);
    // stringPoints += encodeNbIntoString((void*) & (((float)points.r)), sizeof(float));
    // stringPoints += encodeNbIntoString((void*) & (((float)points.g)), sizeof(float));
    // stringPoints += encodeNbIntoString((void*) & (((float)points.b)), sizeof(float));

    //std::cerr << "stringPoints to send " << stringPoints << std::endl;
    // if (points.x == points.x)
    //     stringPoints += encodeNbIntoString((void*) & (points.x), sizeof(points.x));
    // else
    //     stringPoints += encodeNbIntoString((void*) & (0), sizeof(float));
    // if (points.y == points.y)
    //     stringPoints += encodeNbIntoString((void*) & (points.y), sizeof(points.y));
    // else
    //     stringPoints += encodeNbIntoString((void*) & (0), sizeof(float));
    // if (points.z == points.z)
    //     stringPoints += encodeNbIntoString((void*) & (points.z), sizeof(points.z));
    // else
    //     stringPoints += encodeNbIntoString((void*) & (0), sizeof(float));
    // if (points.r == points.r)
    //     stringPoints += encodeNbIntoString((void*) & (points.r), sizeof(float));
    // else
    //     stringPoints += encodeNbIntoString((void*) & (0), sizeof(float));
    // stringPoints += encodeNbIntoString((void*) & (g), sizeof(float));
    // stringPoints += encodeNbIntoString((void*) & (b), sizeof(float));

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

    for (long i = nbOfByte - 1; i >= 0; --i)
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
    std::string tmpChunkID = encodeNbIntoString((void*) & (_chunkID), sizeof(_chunkID));
    ++_chunkID;

    return tmpChunkID;
}

} // end of namespace
