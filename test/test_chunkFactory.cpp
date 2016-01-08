#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include <stdio.h>
#include "ChunkFactory.h"

using namespace igloo;


When_Only(convert_landmark_to_string_using_chunkFactory)
{
    void SetUp()
    {
        landmark.bearing = 666.90;
        landmark.pos = pcl::PointXYZ(1.2, 56.3, -123.45);

        chunkFactory.processData(landmark);
        chunkReadyToSend = chunkFactory.getChunk();
    }

    Then(it_should_tell_that_a_chunk_is_ready)
    {
        chunkFactory.processData(landmark);
        Assert::That(chunkFactory.isFullChunkReady(), Equals(true));
    }

    Then(it_should_tell_that_a_chunk_is_not_ready)
    {
        Assert::That(chunkFactory.isFullChunkReady(), Equals(false));
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

    Then(it_should_create_a_correct_header)
    {
        long long chunkID = 1;
        long long originalChunkID_hexa;
        long long recoveredChunkID_hexa;

        Assert::That((int)'L', Equals((int)chunkReadyToSend[0]));

        originalChunkID_hexa = dumper8(&chunkID);
        recoveredChunkID_hexa = dumper8((const char*)(chunkReadyToSend.substr(1,8).c_str()));

        Assert::That(originalChunkID_hexa, Equals(recoveredChunkID_hexa));
    }

    /* Ne fonctionne forcement pas si la plateforme a des int de 8bits
     * cf: dumper4() ou dumper8()
     * Only try 2 data... but it's enough I guess
     * - Landmark.pos.x
     * - Landmark.bearing
     */
    Then(it_should_create_a_correct_data)
    {
        int originalLandmarkPosX_hexa;
        int recoveredLandmarkPosX_hexa;

        originalLandmarkPosX_hexa = dumper4(&(landmark.pos.x));
        recoveredLandmarkPosX_hexa = dumper4((const char*)chunkReadyToSend.substr(9, 16).c_str());
std::cout << originalLandmarkPosX_hexa << std::endl << recoveredLandmarkPosX_hexa << std::endl;
        Assert::That(originalLandmarkPosX_hexa, Equals(recoveredLandmarkPosX_hexa));


        int originalBearing_hexa;
        int recoveredBearing_hexa;

        originalBearing_hexa = dumper8(&(landmark.bearing)); // [44, 51]
        recoveredBearing_hexa = dumper8((const char*)chunkReadyToSend.substr(43, 50).c_str());
std::cout << originalBearing_hexa << std::endl << recoveredBearing_hexa << std::endl;
        Assert::That(originalBearing_hexa, Equals(recoveredBearing_hexa));
    }

    Landmarks::Landmark         landmark;
    Network::ChunkFactory       chunkFactory;
    std::string                 chunkReadyToSend;
};
