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
                / ("motioncam-decoder-accelerometer-" + std::to_string(now) + "-" + std::to_string(sequence++) + ".mcraw");
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
            { 1'000'123'456, 0.0f, -9.80665f, 0.0f },
            { 1'004'123'456, 1.25f, -2.5f, 9.5f },
            { 1'012'123'456, -3.75f, 4.0f, -9.0f }
        };
        return samples;
    }

    motioncam::BufferOffset writeAccelerometerChunk(
        std::ofstream& output,
        const motioncam::MotionSample* samples,
        const uint32_t numSamples) {
        const motioncam::BufferOffset offset { outputOffset(output), samples[0].timestampNs };
        const motioncam::Item item {
            motioncam::Type::ACCELEROMETER_DATA,
            static_cast<uint32_t>(sizeof(motioncam::AccelerometerDataHeader) + sizeof(motioncam::MotionSample) * numSamples)
        };
        const motioncam::AccelerometerDataHeader header { motioncam::ACCELEROMETER_DATA_VERSION, numSamples };

        writeValue(output, item);
        writeValue(output, header);
        output.write(reinterpret_cast<const char*>(samples), sizeof(motioncam::MotionSample) * numSamples);
        return offset;
    }

    void writeContainer(
        const std::filesystem::path& path,
        const bool includeFrame,
        const bool includeAccelerometer,
        const bool malformedAccelerometerIndex = false,
        const bool includeOtherStreams = false) {
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

        std::vector<motioncam::BufferOffset> accelerometerOffsets;
        const auto& samples = expectedSamples();
        if(includeAccelerometer) {
            const uint32_t firstChunkSize = includeFrame ? 2 : static_cast<uint32_t>(samples.size());
            accelerometerOffsets.push_back(writeAccelerometerChunk(output, samples.data(), firstChunkSize));
        }

        motioncam::BufferOffset gyroOffset{};
        if(includeOtherStreams) {
            gyroOffset = { outputOffset(output), 1'001'000'000 };
            writeValue(output, motioncam::Item { motioncam::Type::GYRO_DATA, 32 });
            writeValue(output, motioncam::GyroDataHeader { motioncam::GYRO_DATA_VERSION, 1 });
            writeValue(output, motioncam::MotionSample { gyroOffset.timestamp, 0.1f, -0.2f, 0.3f });
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

        }

        if(includeOtherStreams) {
            const motioncam::BufferOffset audioOffset { outputOffset(output), 0 };
            writeValue(output, motioncam::Item { motioncam::Type::AUDIO_DATA, 4 });
            writeValue(output, int16_t { 123 });
            writeValue(output, int16_t { -456 });
            writeValue(output, motioncam::Item { motioncam::Type::AUDIO_DATA_METADATA, 8 });
            writeValue(output, motioncam::AudioMetadata { 1'000'000'000 });
            writeValue(output, motioncam::Item { motioncam::Type::AUDIO_INDEX, 32 });
            writeValue(output, motioncam::AudioIndex { 1, 1000 });
            writeValue(output, audioOffset);

            writeValue(output, motioncam::Item { motioncam::Type::GYRO_INDEX, 24 });
            writeValue(output, motioncam::GyroIndex { motioncam::GYRO_INDEX_VERSION, 1 });
            writeValue(output, gyroOffset);

            const motioncam::BufferOffset oisOffset { outputOffset(output), 1'002'000'000 };
            // OIS payload: version/count followed by timestamp and two pixel shifts.
            writeValue(output, motioncam::Item { motioncam::Type::OIS_DATA, 24 });
            writeValue(output, uint32_t { 1 });
            writeValue(output, uint32_t { 1 });
            writeValue(output, oisOffset.timestamp);
            writeValue(output, 0.25f);
            writeValue(output, -0.5f);
            writeValue(output, motioncam::Item { motioncam::Type::OIS_INDEX, 24 });
            writeValue(output, uint32_t { 1 });
            writeValue(output, uint32_t { 1 });
            writeValue(output, oisOffset);
        }

        if(includeAccelerometer && includeFrame)
            accelerometerOffsets.push_back(writeAccelerometerChunk(output, samples.data() + 2, 1));

        if(includeAccelerometer) {
            const uint32_t itemSize = malformedAccelerometerIndex
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(sizeof(motioncam::AccelerometerIndex)
                    + sizeof(motioncam::BufferOffset) * accelerometerOffsets.size());
            const uint32_t numOffsets = malformedAccelerometerIndex
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(accelerometerOffsets.size());
            const motioncam::Item accelerometerIndexItem { motioncam::Type::ACCELEROMETER_INDEX, itemSize };
            const motioncam::AccelerometerIndex accelerometerIndex { motioncam::ACCELEROMETER_INDEX_VERSION, numOffsets };
            writeValue(output, accelerometerIndexItem);
            writeValue(output, accelerometerIndex);

            if(!malformedAccelerometerIndex) {
                output.write(
                    reinterpret_cast<const char*>(accelerometerOffsets.data()),
                    static_cast<std::streamsize>(sizeof(motioncam::BufferOffset) * accelerometerOffsets.size()));
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

    void testAccelerometerDataLoadsAcrossChunks() {
        TemporaryFile file;
        writeContainer(file.path(), true, true);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("frame container has accelerometer data", decoder.hasAccelerometerData());

        std::vector<motioncam::MotionSample> samples;
        decoder.loadAccelerometerData(samples);
        expectSamples("frame container", samples);
        decoder.loadAccelerometerData(samples);
        expectEq("second load appends", expectedSamples().size() * 2, samples.size());
        if(samples.size() == expectedSamples().size() * 2) {
            std::vector<motioncam::MotionSample> appended(samples.begin() + expectedSamples().size(), samples.end());
            expectSamples("appended samples", appended);
        }
    }

    void testAccelerometerOnlyContainerLoads() {
        TemporaryFile file;
        writeContainer(file.path(), false, true);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("accelerometer-only container has no frames", decoder.getFrames().empty());
        expectTrue("accelerometer-only container has accelerometer data", decoder.hasAccelerometerData());

        std::vector<motioncam::MotionSample> samples;
        decoder.loadAccelerometerData(samples);
        expectSamples("accelerometer-only container", samples);
    }

    void testContainerWithoutAccelerometerRemainsSupported() {
        TemporaryFile file;
        writeContainer(file.path(), true, false);

        motioncam::Decoder decoder(file.path().string());
        expectTrue("legacy container has no accelerometer data", !decoder.hasAccelerometerData());

        std::vector<motioncam::MotionSample> samples = { { 42, 1.0f, 2.0f, 3.0f } };
        decoder.loadAccelerometerData(samples);
        expectEq("legacy container leaves output unchanged", size_t { 1 }, samples.size());
        expectEq("legacy container retains sentinel", int64_t { 42 }, samples.front().timestampNs);
    }

    void testMalformedAccelerometerIndexIsRejectedBeforeAllocation() {
        TemporaryFile file;
        writeContainer(file.path(), true, true, true);

        bool rejected = false;
        try {
            motioncam::Decoder decoder(file.path().string());
        }
        catch(const motioncam::IOException&) {
            rejected = true;
        }
        expectTrue("malformed accelerometer index is rejected", rejected);
    }
    void testMixedStreamsRemainReadable() {
        TemporaryFile file;
        writeContainer(file.path(), true, true, false, true);
        motioncam::Decoder decoder(file.path().string());
        expectEq("mixed container frame count", size_t { 1 }, decoder.getFrames().size());
        nlohmann::json metadata;
        decoder.loadFrameMetadata(decoder.getFrames().front(), metadata);
        expectTrue("mixed container frame metadata readable", metadata.is_object());

        expectTrue("mixed container has acceleration beyond OIS", decoder.hasAccelerometerData());
        std::vector<motioncam::MotionSample> samples;
        decoder.loadAccelerometerData(samples);
        expectSamples("mixed container", samples);

        expectTrue("mixed container retains gyro", decoder.hasGyroData());
        std::vector<motioncam::MotionSample> gyro;
        decoder.loadGyroData(gyro);
        expectEq("mixed gyro count", size_t { 1 }, gyro.size());
        if(gyro.size() == 1) {
            expectEq("gyro timestamp unchanged", int64_t { 1'001'000'000 }, gyro[0].timestampNs);
            expectFloatEq("gyro unit unchanged", 0.1f, gyro[0].x);
        }
        std::vector<motioncam::AudioChunk> audio;
        decoder.loadAudio(audio);
        expectEq("mixed audio chunk count", size_t { 1 }, audio.size());
        if(audio.size() == 1) {
            expectEq("audio timestamp unchanged", int64_t { 1'000'000'000 }, audio[0].first);
            expectTrue("audio samples unchanged", audio[0].second == std::vector<int16_t>({ 123, -456 }));
        }
        // Frame/audio reads move the shared FILE cursor; motion loads must seek independently.
        samples.clear();
        decoder.loadAccelerometerData(samples);
        expectSamples("mixed container reload", samples);
    }

    void testGyroAndOisWithoutAccelerometerRemainReadable() {
        TemporaryFile file;
        writeContainer(file.path(), true, false, false, true);
        motioncam::Decoder decoder(file.path().string());
        expectTrue("gyro/OIS container has no accelerometer", !decoder.hasAccelerometerData());
        expectTrue("gyro/OIS container retains gyro", decoder.hasGyroData());
        std::vector<motioncam::MotionSample> samples;
        decoder.loadAccelerometerData(samples);
        expectTrue("gyro/OIS container loads no acceleration", samples.empty());
    }

    template<typename Value>
    bool patchItem(const std::filesystem::path& path, motioncam::Type type, int64_t relativeOffset, const Value& value) {
        std::fstream stream(path, std::ios::binary | std::ios::in | std::ios::out);
        int64_t offset = sizeof(motioncam::Header);
        while(stream) {
            stream.seekg(offset);
            motioncam::Item item{};
            if(!stream.read(reinterpret_cast<char*>(&item), sizeof(item)))
                return false;
            if(item.type == type) {
                stream.seekp(offset + relativeOffset);
                stream.write(reinterpret_cast<const char*>(&value), sizeof(value));
                return static_cast<bool>(stream);
            }
            offset += sizeof(item) + item.size;
        }
        return false;
    }

    template<typename Value>
    void testMalformedItemIsRejected(
        const std::string& name, motioncam::Type type, int64_t relativeOffset, const Value& value) {
        TemporaryFile file;
        writeContainer(file.path(), true, true, false, true);
        expectTrue(name + " fixture patched", patchItem(file.path(), type, relativeOffset, value));
        bool rejected = false;
        try {
            motioncam::Decoder decoder(file.path().string());
            std::vector<motioncam::MotionSample> samples;
            decoder.loadAccelerometerData(samples);
        }
        catch(const motioncam::IOException&) {
            rejected = true;
        }
        expectTrue(name + " rejected", rejected);
    }

    void testMalformedMotionDataIsRejected() {
        using motioncam::Type;
        constexpr int64_t payload = sizeof(motioncam::Item);
        constexpr int64_t count = payload + sizeof(uint32_t);
        constexpr int64_t firstOffset = payload + sizeof(motioncam::AccelerometerIndex);
        testMalformedItemIsRejected("unsupported data version", Type::ACCELEROMETER_DATA, payload, uint32_t { 2 });
        testMalformedItemIsRejected("empty data chunk", Type::ACCELEROMETER_DATA, count, uint32_t { 0 });
        testMalformedItemIsRejected("oversized sample count", Type::ACCELEROMETER_DATA, count, UINT32_MAX);
        testMalformedItemIsRejected("short data header", Type::ACCELEROMETER_DATA, 4, uint32_t { 4 });
        testMalformedItemIsRejected("unsupported index version", Type::ACCELEROMETER_INDEX, payload, uint32_t { 2 });
        testMalformedItemIsRejected("inconsistent index count", Type::ACCELEROMETER_INDEX, count, uint32_t { 3 });
        testMalformedItemIsRejected("short index header", Type::ACCELEROMETER_INDEX, 4, uint32_t { 4 });
        testMalformedItemIsRejected("negative data offset", Type::ACCELEROMETER_INDEX, firstOffset, int64_t { -1 });
        testMalformedItemIsRejected("wrong item at data offset", Type::ACCELEROMETER_INDEX, firstOffset, int64_t { 0 });
        testMalformedItemIsRejected("out-of-file data offset", Type::ACCELEROMETER_INDEX, firstOffset, int64_t { 1'000'000 });
        testMalformedItemIsRejected("OIS data exceeds file", Type::OIS_DATA, 4, UINT32_MAX);
        testMalformedItemIsRejected("OIS index exceeds file", Type::OIS_INDEX, 4, UINT32_MAX);
    }

}

int main() {
    testAccelerometerDataLoadsAcrossChunks();
    testMixedStreamsRemainReadable();
    testGyroAndOisWithoutAccelerometerRemainReadable();
    testMalformedMotionDataIsRejected();
    testAccelerometerOnlyContainerLoads();
    testContainerWithoutAccelerometerRemainsSupported();
    testMalformedAccelerometerIndexIsRejectedBeforeAllocation();

    if(failures == 0)
        std::cout << "DecoderAccelerometerTest passed\n";
    return failures == 0 ? 0 : 1;
}
