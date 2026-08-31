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
#include <motioncam/RawData.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

namespace {
    constexpr int64_t CURRENT_FRAME_TIMESTAMP_NS = 100'000'000;
    constexpr int64_t LEGACY_FRAME_TIMESTAMP_NS = 200'000'000;
    constexpr uint16_t CURRENT_PIXEL_VALUE = 321;
    constexpr uint16_t LEGACY_EVEN_PIXEL_VALUE = 100;
    constexpr uint16_t LEGACY_ODD_PIXEL_VALUE = 200;

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
            std::cerr << "FAIL " << name << '\n';
            ++failures;
        }
    }

    class TemporaryFile {
    public:
        TemporaryFile() {
            static uint64_t sequence = 0;
            const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            mPath = std::filesystem::temp_directory_path()
                / ("motioncam-decoder-test-" + std::to_string(now) + "-" + std::to_string(sequence++) + ".mcraw");
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

    void writeBytes(std::ofstream& output, const std::vector<uint8_t>& values) {
        output.write(reinterpret_cast<const char*>(values.data()), static_cast<std::streamsize>(values.size()));
    }

    int64_t outputOffset(std::ofstream& output) {
        return static_cast<int64_t>(output.tellp());
    }

    void writeUint32(std::vector<uint8_t>& output, const size_t offset, const uint32_t value) {
        output[offset] = static_cast<uint8_t>(value);
        output[offset + 1] = static_cast<uint8_t>(value >> 8);
        output[offset + 2] = static_cast<uint8_t>(value >> 16);
        output[offset + 3] = static_cast<uint8_t>(value >> 24);
    }

    std::vector<uint8_t> makeCurrentConstantFrame() {
        constexpr uint32_t encodedWidth = 64;
        constexpr uint32_t encodedHeight = 4;
        constexpr uint32_t bitsOffset = 16;
        constexpr uint32_t referencesOffset = 22;
        constexpr uint32_t metadataValues = 64;

        std::vector<uint8_t> encoded(28, 0);
        writeUint32(encoded, 0, encodedWidth);
        writeUint32(encoded, 4, encodedHeight);
        writeUint32(encoded, 8, bitsOffset);
        writeUint32(encoded, 12, referencesOffset);

        writeUint32(encoded, bitsOffset, metadataValues);
        encoded[bitsOffset + 4] = 0;
        encoded[bitsOffset + 5] = 0;

        writeUint32(encoded, referencesOffset, metadataValues);
        encoded[referencesOffset + 4] = static_cast<uint8_t>((CURRENT_PIXEL_VALUE >> 8) & 0x0f);
        encoded[referencesOffset + 5] = static_cast<uint8_t>(CURRENT_PIXEL_VALUE & 0xff);
        return encoded;
    }

    std::vector<uint8_t> makeLegacyAlternatingFrame() {
        return {
            static_cast<uint8_t>((LEGACY_EVEN_PIXEL_VALUE >> 8) & 0x0f),
            static_cast<uint8_t>(LEGACY_EVEN_PIXEL_VALUE & 0xff),
            static_cast<uint8_t>((LEGACY_ODD_PIXEL_VALUE >> 8) & 0x0f),
            static_cast<uint8_t>(LEGACY_ODD_PIXEL_VALUE & 0xff),
            0
        };
    }

    motioncam::BufferOffset writeFrame(
        std::ofstream& output,
        const int64_t timestampNs,
        const std::vector<uint8_t>& encoded,
        const std::string& metadata) {
        const motioncam::BufferOffset offset { outputOffset(output), timestampNs };
        const motioncam::Item frameItem {
            motioncam::Type::BUFFER,
            static_cast<uint32_t>(encoded.size())
        };
        const motioncam::Item metadataItem {
            motioncam::Type::METADATA,
            static_cast<uint32_t>(metadata.size())
        };

        writeValue(output, frameItem);
        writeBytes(output, encoded);
        writeValue(output, metadataItem);
        output.write(metadata.data(), static_cast<std::streamsize>(metadata.size()));
        return offset;
    }

    motioncam::BufferOffset writeAudioChunk(
        std::ofstream& output,
        const int64_t timestampNs,
        const std::vector<int16_t>& samples) {
        const motioncam::BufferOffset offset { outputOffset(output), timestampNs };
        const motioncam::Item audioItem {
            motioncam::Type::AUDIO_DATA,
            static_cast<uint32_t>(samples.size() * sizeof(int16_t))
        };
        const motioncam::Item metadataItem {
            motioncam::Type::AUDIO_DATA_METADATA,
            sizeof(motioncam::AudioMetadata)
        };
        const motioncam::AudioMetadata metadata { timestampNs };

        writeValue(output, audioItem);
        output.write(
            reinterpret_cast<const char*>(samples.data()),
            static_cast<std::streamsize>(samples.size() * sizeof(int16_t)));
        writeValue(output, metadataItem);
        writeValue(output, metadata);
        return offset;
    }

    void writeContainer(const std::filesystem::path& path, const bool validIndexMagic = true) {
        std::ofstream output(path, std::ios::binary);

        motioncam::Header header{};
        header.version = motioncam::CONTAINER_VERSION;
        std::memcpy(header.ident, motioncam::CONTAINER_ID, sizeof(motioncam::CONTAINER_ID));
        writeValue(output, header);

        const std::string containerMetadata =
            R"({"name":"decoder fixture","extraData":{"audioSampleRate":48000,"audioChannels":2}})";
        const motioncam::Item containerMetadataItem {
            motioncam::Type::METADATA,
            static_cast<uint32_t>(containerMetadata.size())
        };
        writeValue(output, containerMetadataItem);
        output.write(containerMetadata.data(), static_cast<std::streamsize>(containerMetadata.size()));

        const std::string legacyMetadata =
            R"({"width":32,"height":1,"compressionType":6,"label":"legacy"})";
        const std::string currentMetadata =
            R"({"width":64,"height":4,"compressionType":7,"label":"current"})";

        std::vector<motioncam::BufferOffset> frameOffsets;
        frameOffsets.push_back(writeFrame(
            output,
            LEGACY_FRAME_TIMESTAMP_NS,
            makeLegacyAlternatingFrame(),
            legacyMetadata));
        frameOffsets.push_back(writeFrame(
            output,
            CURRENT_FRAME_TIMESTAMP_NS,
            makeCurrentConstantFrame(),
            currentMetadata));

        const std::vector<int16_t> firstAudio = { 1, -2, 3, -4 };
        const std::vector<int16_t> secondAudio = { 5, 6 };
        std::vector<motioncam::BufferOffset> audioOffsets;
        audioOffsets.push_back(writeAudioChunk(output, 300'000'000, firstAudio));
        audioOffsets.push_back(writeAudioChunk(output, 400'000'000, secondAudio));

        const motioncam::Item audioIndexItem {
            motioncam::Type::AUDIO_INDEX,
            static_cast<uint32_t>(sizeof(motioncam::AudioIndex)
                + audioOffsets.size() * sizeof(motioncam::BufferOffset))
        };
        const motioncam::AudioIndex audioIndex {
            static_cast<int64_t>(audioOffsets.size()),
            300
        };
        writeValue(output, audioIndexItem);
        writeValue(output, audioIndex);
        output.write(
            reinterpret_cast<const char*>(audioOffsets.data()),
            static_cast<std::streamsize>(audioOffsets.size() * sizeof(motioncam::BufferOffset)));

        const int64_t frameIndexDataOffset = outputOffset(output);
        output.write(
            reinterpret_cast<const char*>(frameOffsets.data()),
            static_cast<std::streamsize>(frameOffsets.size() * sizeof(motioncam::BufferOffset)));

        const motioncam::Item frameIndexItem {
            motioncam::Type::BUFFER_INDEX,
            sizeof(motioncam::BufferIndex)
        };
        const motioncam::BufferIndex frameIndex {
            validIndexMagic ? static_cast<int32_t>(motioncam::INDEX_MAGIC_NUMBER) : 0,
            static_cast<int32_t>(frameOffsets.size()),
            frameIndexDataOffset
        };
        writeValue(output, frameIndexItem);
        writeValue(output, frameIndex);
    }

    template<typename Action>
    void expectIOException(const std::string& name, Action action) {
        bool threw = false;
        try {
            action();
        }
        catch(const motioncam::IOException&) {
            threw = true;
        }
        expectTrue(name, threw);
    }

    void expectCurrentPixels(const std::string& name, const std::vector<uint8_t>& decoded) {
        expectEq(name + " byte count", static_cast<size_t>(64 * 4 * sizeof(uint16_t)), decoded.size());
        if(decoded.size() != 64 * 4 * sizeof(uint16_t))
            return;

        for(size_t i = 0; i < 64 * 4; ++i) {
            uint16_t pixel = 0;
            std::memcpy(&pixel, decoded.data() + i * sizeof(uint16_t), sizeof(pixel));
            if(pixel != CURRENT_PIXEL_VALUE) {
                expectEq(name + " pixel " + std::to_string(i), CURRENT_PIXEL_VALUE, pixel);
                return;
            }
        }
    }

    void expectLegacyPixels(const std::string& name, const std::vector<uint8_t>& decoded) {
        expectEq(name + " byte count", static_cast<size_t>(32 * sizeof(uint16_t)), decoded.size());
        if(decoded.size() != 32 * sizeof(uint16_t))
            return;

        for(size_t i = 0; i < 32; ++i) {
            const uint16_t expected = i % 2 == 0 ? LEGACY_EVEN_PIXEL_VALUE : LEGACY_ODD_PIXEL_VALUE;
            uint16_t pixel = 0;
            std::memcpy(&pixel, decoded.data() + i * sizeof(uint16_t), sizeof(pixel));
            if(pixel != expected) {
                expectEq(name + " pixel " + std::to_string(i), expected, pixel);
                return;
            }
        }
    }

    void testRawDecoders() {
        const auto currentEncoded = makeCurrentConstantFrame();
        std::vector<uint16_t> currentDecoded(64 * 4);
        const size_t currentPixels = motioncam::raw::Decode(
            currentDecoded.data(), 64, 4, currentEncoded.data(), currentEncoded.size());
        expectEq("current raw decoded pixel count", static_cast<size_t>(64 * 4), currentPixels);
        expectTrue(
            "current raw decoded values",
            std::all_of(currentDecoded.begin(), currentDecoded.end(), [](const auto value) {
                return value == CURRENT_PIXEL_VALUE;
            }));

        const auto legacyEncoded = makeLegacyAlternatingFrame();
        std::vector<uint16_t> legacyDecoded(32);
        const size_t legacyPixels = motioncam::raw::DecodeLegacy(
            legacyDecoded.data(), 32, 1, legacyEncoded.data(), legacyEncoded.size());
        expectEq("legacy raw decoded pixel count", static_cast<size_t>(32), legacyPixels);
        for(size_t i = 0; i < legacyDecoded.size(); ++i) {
            const uint16_t expected = i % 2 == 0 ? LEGACY_EVEN_PIXEL_VALUE : LEGACY_ODD_PIXEL_VALUE;
            if(legacyDecoded[i] != expected) {
                expectEq("legacy raw decoded pixel " + std::to_string(i), expected, legacyDecoded[i]);
                break;
            }
        }
    }

    void testDecoderPublicApi() {
        TemporaryFile file;
        writeContainer(file.path());
        motioncam::Decoder decoder(file.path().string());

        expectEq("container metadata", std::string("decoder fixture"),
            decoder.getContainerMetadata().at("name").get<std::string>());
        expectEq("audio sample rate", 48000, decoder.audioSampleRateHz());
        expectEq("audio channels", 2, decoder.numAudioChannels());
        expectTrue("fixture has no gyro", !decoder.hasGyroData());

        const auto& frames = decoder.getFrames();
        expectEq("frame count", static_cast<size_t>(2), frames.size());
        if(frames.size() == 2) {
            expectEq("first sorted frame", CURRENT_FRAME_TIMESTAMP_NS, frames[0]);
            expectEq("second sorted frame", LEGACY_FRAME_TIMESTAMP_NS, frames[1]);
        }

        std::vector<uint8_t> frameData;
        nlohmann::json frameMetadata;
        decoder.loadFrame(CURRENT_FRAME_TIMESTAMP_NS, frameData, frameMetadata);
        expectEq("current frame label", std::string("current"), frameMetadata.at("label").get<std::string>());
        expectCurrentPixels("current frame", frameData);

        decoder.loadFrame(LEGACY_FRAME_TIMESTAMP_NS, frameData, frameMetadata);
        expectEq("legacy frame label", std::string("legacy"), frameMetadata.at("label").get<std::string>());
        expectLegacyPixels("legacy frame", frameData);

        decoder.loadFrameMetadata(CURRENT_FRAME_TIMESTAMP_NS, frameMetadata);
        expectEq("metadata-only frame label", std::string("current"), frameMetadata.at("label").get<std::string>());

        std::vector<motioncam::AudioChunk> audio;
        decoder.loadAudio(audio);
        expectEq("bulk audio chunk count", static_cast<size_t>(2), audio.size());
        if(audio.size() == 2) {
            expectEq("first audio timestamp", static_cast<int64_t>(300'000'000), audio[0].first);
            expectEq("first audio samples", std::vector<int16_t>({ 1, -2, 3, -4 }), audio[0].second);
            expectEq("second audio timestamp", static_cast<int64_t>(400'000'000), audio[1].first);
            expectEq("second audio samples", std::vector<int16_t>({ 5, 6 }), audio[1].second);
        }

        auto& audioLoader = decoder.loadAudio();
        motioncam::AudioChunk audioChunk;
        expectTrue("chunk loader first", audioLoader.next(audioChunk));
        expectEq("chunk loader first samples", std::vector<int16_t>({ 1, -2, 3, -4 }), audioChunk.second);
        expectTrue("chunk loader second", audioLoader.next(audioChunk));
        expectEq("chunk loader second samples", std::vector<int16_t>({ 5, 6 }), audioChunk.second);
        expectTrue("chunk loader exhausted", !audioLoader.next(audioChunk));

        expectIOException("missing frame is rejected", [&decoder, &frameData, &frameMetadata]() {
            decoder.loadFrame(-1, frameData, frameMetadata);
        });
    }

    void testInvalidContainerIsRejected() {
        TemporaryFile file;
        writeContainer(file.path(), false);
        expectIOException("invalid frame index magic is rejected", [&file]() {
            motioncam::Decoder decoder(file.path().string());
        });
    }
}

int main() {
    testRawDecoders();
    testDecoderPublicApi();
    testInvalidContainerIsRejected();

    if(failures == 0)
        std::cout << "DecoderTest passed\n";
    return failures == 0 ? 0 : 1;
}
