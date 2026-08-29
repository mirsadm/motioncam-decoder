/*
 * Copyright 2026 MotionCam
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <motioncam/Decoder.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

namespace {
    int failures = 0;

    void expectTrue(const std::string& name, const bool value) {
        if(!value) {
            std::cerr << "FAIL " << name << '\n';
            ++failures;
        }
    }

    template<typename Expected, typename Actual>
    void expectEq(const std::string& name, const Expected& expected, const Actual& actual) {
        if(!(expected == actual)) {
            std::cerr << "FAIL " << name << ": expected " << expected << ", got " << actual << '\n';
            ++failures;
        }
    }

    void expectFloatEq(const std::string& name, const float expected, const float actual) {
        constexpr float epsilon = 0.000001f;
        if(std::abs(expected - actual) > epsilon) {
            std::cerr << "FAIL " << name << ": expected " << expected << ", got " << actual << '\n';
            ++failures;
        }
    }

    class TemporaryFile {
    public:
        TemporaryFile() {
            static uint64_t sequence = 0;
            const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            mPath = std::filesystem::temp_directory_path()
                / ("motioncam-decoder-gyro-" + std::to_string(now) + "-" + std::to_string(sequence++) + ".mcraw");
        }

        ~TemporaryFile() {
            std::error_code error;
            std::filesystem::remove(mPath, error);
        }

        const std::filesystem::path& path() const {
            return mPath;
        }

    private:
        std::filesystem::path mPath;
    };

    template<typename Value>
    void writeValue(std::ofstream& output, const Value& value) {
        output.write(reinterpret_cast<const char*>(&value), sizeof(Value));
    }

    int64_t outputOffset(std::ofstream& output) {
        return static_cast<int64_t>(output.tellp());
    }

    const std::vector<motioncam::MotionSample>& expectedSamples() {
        static const std::vector<motioncam::MotionSample> samples = {
            { 1'000'000'000, 0.1f, -0.2f, 0.3f },
            { 1'002'500'000, 0.4f, 0.5f, -0.6f },
            { 1'005'000'000, -0.7f, 0.8f, 0.9f }
        };
        return samples;
    }

    motioncam::BufferOffset writeGyroChunk(
        std::ofstream& output,
        const motioncam::MotionSample* samples,
        const uint32_t numSamples) {
        const motioncam::BufferOffset offset { outputOffset(output), samples[0].timestampNs };
        const motioncam::Item item {
            motioncam::Type::GYRO_DATA,
            static_cast<uint32_t>(sizeof(motioncam::GyroDataHeader) + sizeof(motioncam::MotionSample) * numSamples)
        };
        const motioncam::GyroDataHeader header { motioncam::GYRO_DATA_VERSION, numSamples };

        writeValue(output, item);
        writeValue(output, header);
        output.write(reinterpret_cast<const char*>(samples), sizeof(motioncam::MotionSample) * numSamples);
        return offset;
    }

    void writeContainer(
        const std::filesystem::path& path,
        const bool includeFrame,
        const bool includeGyro,
        const bool malformedGyroIndex = false) {
        std::ofstream output(path, std::ios::binary);

        motioncam::Header header{};
        header.version = motioncam::CONTAINER_VERSION;
        std::memcpy(header.ident, motioncam::CONTAINER_ID, sizeof(motioncam::CONTAINER_ID));
        writeValue(output, header);

        const std::string metadata = "{}";
        const motioncam::Item metadataItem {
            motioncam::Type::METADATA,
            static_cast<uint32_t>(metadata.size())
        };
        writeValue(output, metadataItem);
        output.write(metadata.data(), static_cast<std::streamsize>(metadata.size()));

        std::vector<motioncam::BufferOffset> gyroOffsets;
        const auto& samples = expectedSamples();
        if(includeGyro) {
            const uint32_t firstChunkSize = includeFrame ? 2 : static_cast<uint32_t>(samples.size());
            gyroOffsets.push_back(writeGyroChunk(output, samples.data(), firstChunkSize));
        }

        std::vector<motioncam::BufferOffset> frameOffsets;
        if(includeFrame) {
            constexpr int64_t frameTimestampNs = 1'010'000'000;
            frameOffsets.push_back({ outputOffset(output), frameTimestampNs });

            const uint8_t frameData = 0;
            const motioncam::Item frameItem { motioncam::Type::BUFFER, sizeof(frameData) };
            writeValue(output, frameItem);
            writeValue(output, frameData);
            writeValue(output, metadataItem);
            output.write(metadata.data(), static_cast<std::streamsize>(metadata.size()));

            if(includeGyro)
                gyroOffsets.push_back(writeGyroChunk(output, samples.data() + 2, 1));
        }

        if(includeGyro) {
            const uint32_t itemSize = malformedGyroIndex
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(sizeof(motioncam::GyroIndex)
                    + sizeof(motioncam::BufferOffset) * gyroOffsets.size());
            const uint32_t numOffsets = malformedGyroIndex
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(gyroOffsets.size());
            const motioncam::Item gyroIndexItem { motioncam::Type::GYRO_INDEX, itemSize };
            const motioncam::GyroIndex gyroIndex { motioncam::GYRO_INDEX_VERSION, numOffsets };
            writeValue(output, gyroIndexItem);
            writeValue(output, gyroIndex);

            if(!malformedGyroIndex) {
                output.write(
                    reinterpret_cast<const char*>(gyroOffsets.data()),
                    static_cast<std::streamsize>(sizeof(motioncam::BufferOffset) * gyroOffsets.size()));
            }
        }

        const int64_t frameIndexDataOffset = outputOffset(output);
        output.write(
            reinterpret_cast<const char*>(frameOffsets.data()),
            static_cast<std::streamsize>(sizeof(motioncam::BufferOffset) * frameOffsets.size()));

        const motioncam::Item frameIndexItem { motioncam::Type::BUFFER_INDEX, sizeof(motioncam::BufferIndex) };
        const motioncam::BufferIndex frameIndex {
            static_cast<int32_t>(motioncam::INDEX_MAGIC_NUMBER),
            static_cast<int32_t>(frameOffsets.size()),
            frameIndexDataOffset
        };
        writeValue(output, frameIndexItem);
        writeValue(output, frameIndex);
    }

    void expectSamples(const std::string& context, const std::vector<motioncam::MotionSample>& actual) {
        const auto& expected = expectedSamples();
        expectEq(context + " count", expected.size(), actual.size());
        if(actual.size() != expected.size())
            return;

        for(size_t i = 0; i < expected.size(); ++i) {
            expectEq(context + " timestamp " + std::to_string(i), expected[i].timestampNs, actual[i].timestampNs);
            expectFloatEq(context + " x " + std::to_string(i), expected[i].x, actual[i].x);
            expectFloatEq(context + " y " + std::to_string(i), expected[i].y, actual[i].y);
            expectFloatEq(context + " z " + std::to_string(i), expected[i].z, actual[i].z);
        }
    }

    void testGyroDataLoadsAcrossChunks() {
        TemporaryFile file;
        writeContainer(file.path(), true, true);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("frame container has gyro data", decoder.hasGyroData());

        std::vector<motioncam::MotionSample> samples;
        decoder.loadGyroData(samples);
        expectSamples("frame container", samples);
    }

    void testGyroOnlyContainerLoads() {
        TemporaryFile file;
        writeContainer(file.path(), false, true);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("gyro-only container has no frames", decoder.getFrames().empty());
        expectTrue("gyro-only container has gyro data", decoder.hasGyroData());

        std::vector<motioncam::MotionSample> samples;
        decoder.loadGyroData(samples);
        expectSamples("gyro-only container", samples);
    }

    void testContainerWithoutGyroRemainsSupported() {
        TemporaryFile file;
        writeContainer(file.path(), true, false);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("legacy container has no gyro data", !decoder.hasGyroData());

        std::vector<motioncam::MotionSample> samples;
        decoder.loadGyroData(samples);
        expectTrue("legacy container loads no gyro samples", samples.empty());
    }

    void testMalformedGyroIndexIsRejectedBeforeAllocation() {
        TemporaryFile file;
        writeContainer(file.path(), true, true, true);

        bool rejected = false;
        try {
            motioncam::Decoder decoder(file.path().string());
        }
        catch(const motioncam::IOException&) {
            rejected = true;
        }
        expectTrue("malformed gyro index is rejected", rejected);
    }
}

int main() {
    testGyroDataLoadsAcrossChunks();
    testGyroOnlyContainerLoads();
    testContainerWithoutGyroRemainsSupported();
    testMalformedGyroIndexIsRejectedBeforeAllocation();

    if(failures == 0)
        std::cout << "DecoderGyroTest passed\n";
    return failures == 0 ? 0 : 1;
}
