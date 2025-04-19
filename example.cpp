#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <algorithm>
#include <motioncam/Decoder.hpp>
#include <nlohmann/json.hpp>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

// This example produces raw video output to stdout. The program reads frames
// from your input file (via motioncam::Decoder) and writes each frame’s raw
// data (10-bit stored in a 16-bit container) consecutively. You can then pipe
// this binary output into ffmpeg to encode it, for example, as ProRes.

// Usage: decoder input_file [-n num_frames]
int main(int argc, const char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: decoder input_file [-n num_frames]" << std::endl;
        return -1;
    }
    
    std::string inputPath(argv[1]);
    int endFrame = -1;
    if (argc > 3) {
        if (std::string(argv[2]) == "-n")
            endFrame = std::stoi(argv[3]);
    }
    
    try {
        motioncam::Decoder d(inputPath);
        auto frames = d.getFrames();
        
        if (frames.empty()) {
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
        
        // On Windows, set stdout to binary mode:
        #ifdef _WIN32
            _setmode(_fileno(stdout), _O_BINARY);
        #endif

        // Load the first frame to get image dimensions.
        d.loadFrame(frames[0], data, metadata);
        unsigned int width  = metadata["width"];
        unsigned int height = metadata["height"];
        
        // For this example, we assume that each sample is stored in a 16-bit word,
        // even though only 10 bits per sample carry image data.
        const size_t frameSize = width * height * sizeof(uint16_t);
        
        // Write out the first frame.
        std::cout.write(reinterpret_cast<const char*>(data.data()), frameSize);
        
        // Write the remaining frames.
        for (int i = 1; i < endFrame; i++) {
            d.loadFrame(frames[i], data, metadata);
            if (data.size() * sizeof(uint16_t) != frameSize) {
                throw std::runtime_error("Frame size mismatch. Aborting.");
            }
            std::cout.write(reinterpret_cast<const char*>(data.data()), frameSize);
        }
        
        std::cout.flush();
    }
    catch (motioncam::MotionCamException& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    catch (std::exception& ex) {
        std::cerr << "Unexpected error: " << ex.what() << std::endl;
        return -1;
    }
    
    return 0;
}