#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <algorithm>
#include <motioncam/Decoder.hpp>
#include <nlohmann/json.hpp>


// This example produces raw video output to stdout. The program reads frames
// from your input file (via motioncam::Decoder) and writes each frame’s raw
// data (10-bit stored in a 16-bit container, scaled to use the full 16-bit range)
// consecutively. You can then pipe this binary output into ffmpeg to encode it,
// for example, as ProRes.
//
// Usage: decoder input_file [-n num_frames]
int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: decoder input_file [-n num_frames]" << std::endl;
        return -1;
    }

    std::string inputPath(argv[1]);
    int endFrame = -1;
    if (argc > 3)
    {
        if (std::string(argv[2]) == "-n")
            endFrame = std::stoi(argv[3]);
    }

    try
    {
        motioncam::Decoder d(inputPath);
        auto frames = d.getFrames();

        if (frames.empty())
        {
            std::cerr << "No frames found in input file." << std::endl;
            return -1;
        }

        // If no frame limit was provided, output them all.
        if (endFrame < 0)
            endFrame = static_cast<int>(frames.size());
        else
            endFrame = std::min(static_cast<int>(frames.size()), endFrame);

        std::vector<uint16_t> data;
        nlohmann::json metadata;

        // Load the first frame to get image dimensions.
        d.loadFrame(frames[0], data, metadata);
        unsigned int width = metadata["width"];
        unsigned int height = metadata["height"];

        // For this example, we assume that each sample is stored in a 16-bit word,
        // even though only 10 bits per sample initially carry image data.
        // After scaling, every pixel will use the full 16-bit dynamic range.
        const size_t numPixels = width * height;
        const size_t frameSize = numPixels * sizeof(uint16_t);

        // Scale the 10-bit values (range 0-1023) to 16-bit (range 0-65535).
        for (size_t i = 0; i < data.size(); i++)
        {
            // Multiply by 65535 and add half the divisor (511) for rounding, then divide by 1023.
            data[i] = static_cast<uint16_t>((data[i] * 65535 + 511) / 1023);
        }

        // Write out the first frame.
        std::cout.write(reinterpret_cast<const char *>(data.data()), frameSize);

        // Process and write the remaining frames.
        for (int i = 1; i < endFrame; i++)
        {
            d.loadFrame(frames[i], data, metadata);

            if (data.size() != numPixels)
            {
                throw std::runtime_error("Frame size mismatch. Aborting.");
            }

            // Scale each pixel value from 10-bit to 16-bit.
            for (size_t j = 0; j < data.size(); j++)
            {
                data[j] = static_cast<uint16_t>((data[j] * 65535 + 511) / 1023);
            }

            std::cout.write(reinterpret_cast<const char *>(data.data()), frameSize);
        }

        std::cout.flush();
    }
    catch (motioncam::MotionCamException &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    catch (std::exception &ex)
    {
        std::cerr << "Unexpected error: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}