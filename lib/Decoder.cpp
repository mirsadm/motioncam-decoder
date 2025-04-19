#include <motioncam/Decoder.hpp>
#include <motioncam/RawData.hpp>

#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <stdexcept>

#if defined(_WIN32)
    #define FSEEK _fseeki64
    #define FTELL _ftelli64
#elif defined(__unix__) || defined(__linux__) || defined(__APPLE__)
    #define _FILE_OFFSET_BITS 64
    #define FSEEK fseeko
    #define FTELL ftello
#else
    #error Unknown platform
#endif

namespace motioncam {
    constexpr int MOTIONCAM_COMPRESSION_TYPE = 7;

    Decoder::Decoder(FILE* file) : mFile(file) {
        if (!mFile)
            throw IOException("Invalid file");
        init();
    }

    Decoder::Decoder(const std::string& path) : mFile(std::fopen(path.c_str(), "rb")) {
        if (!mFile)
            throw IOException("Failed to open " + path);
        init();
    }
    
    Decoder::~Decoder() {
        if (mFile)
            std::fclose(mFile);
    }
    
    void Decoder::init() {
        Header header{};
        read(&header, sizeof(Header));

        if (header.version != CONTAINER_VERSION)
            throw IOException("Invalid container version");
        if (std::memcmp(header.ident, CONTAINER_ID, sizeof(CONTAINER_ID)) != 0)
            throw IOException("Invalid header id");

        // Read camera metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        if (metadataItem.type != Type::METADATA)
            throw IOException("Invalid camera metadata");

        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        
        mMetadata = nlohmann::json::parse(std::string(metadataJson.begin(), metadataJson.end()));

        readIndex();
        reindexOffsets();
        readExtra();
    }
    
    const std::vector<Timestamp>& Decoder::getFrames() const {
        return mFrameList;
    }
    
    const nlohmann::json& Decoder::getContainerMetadata() const {
        return mMetadata;
    }
    
    int Decoder::audioSampleRateHz() const {
        return mMetadata["extraData"]["audioSampleRate"];
    }
    
    int Decoder::numAudioChannels() const {
        return mMetadata["extraData"]["audioChannels"];
    }
    
    void Decoder::loadAudio(std::vector<AudioChunk>& outAudioChunks) {
        for (const auto& audioOffset : mAudioOffsets) {
            if (FSEEK(mFile, audioOffset.offset, SEEK_SET) != 0)
                break;
            
            Item audioDataItem{};
            read(&audioDataItem, sizeof(Item));
            if (audioDataItem.type != Type::AUDIO_DATA)
                throw IOException("Invalid audio data");
            
            std::vector<int16_t> tmp((audioDataItem.size + 1) / 2);
            read(tmp.data(), audioDataItem.size);

            // Read metadata if present (newer files)
            Item audioMetadataItem{};
            read(&audioMetadataItem, sizeof(Item));
            Timestamp audioTimestamp = -1;
            if (audioMetadataItem.type == Type::AUDIO_DATA_METADATA) {
                AudioMetadata metadata{};
                read(&metadata, sizeof(AudioMetadata));
                audioTimestamp = metadata.timestampNs;
            }
            outAudioChunks.emplace_back(audioTimestamp, std::move(tmp));
        }
    }
    
    void Decoder::loadFrame(const Timestamp timestamp, std::vector<uint16_t>& outData, nlohmann::json& outMetadata) {
        auto it = mFrameOffsetMap.find(timestamp);
        if (it == mFrameOffsetMap.end())
            throw IOException("Frame not found (timestamp: " + std::to_string(timestamp) + ")");
        
        int64_t offset = it->second.offset;
        if (FSEEK(mFile, offset, SEEK_SET) != 0)
            throw IOException("Invalid offset");
        
        Item bufferItem{};
        read(&bufferItem, sizeof(Item));
        if (bufferItem.type != Type::BUFFER)
            throw IOException("Invalid buffer type");

        mTmpBuffer.resize(bufferItem.size);
        read(mTmpBuffer.data(), bufferItem.size);
                
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        if (metadataItem.type != Type::METADATA)
            throw IOException("Invalid metadata");
        
        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        outMetadata = nlohmann::json::parse(std::string(metadataJson.begin(), metadataJson.end()));
        
        const int width = outMetadata["width"];
        const int height = outMetadata["height"];
        const int compressionType = outMetadata["compressionType"];
        if (compressionType != MOTIONCAM_COMPRESSION_TYPE)
            throw IOException("Invalid compression type");
            
        // Resize to pixel count rather than byte count.
        outData.resize(static_cast<size_t>(width) * height);
        if (raw::Decode(outData.data(), width, height, mTmpBuffer.data(), mTmpBuffer.size()) <= 0)
            throw IOException("Failed to uncompress frame");
    }

    void Decoder::readIndex() {
        if (FSEEK(mFile, -static_cast<long>(sizeof(BufferIndex) + sizeof(Item)), SEEK_END) != 0)
            throw IOException("Failed to get end chunk");

        Item bufferIndexItem{};
        read(&bufferIndexItem, sizeof(Item));
        if (bufferIndexItem.type != Type::BUFFER_INDEX)
            throw IOException("Invalid file");
        
        BufferIndex index{};
        read(&index, sizeof(BufferIndex));
        if (index.magicNumber != INDEX_MAGIC_NUMBER)
            throw IOException("Corrupted file");
        
        mOffsets.resize(index.numOffsets);
        if (FSEEK(mFile, index.indexDataOffset, SEEK_SET) != 0)
            throw IOException("Invalid index");
        
        read(mOffsets.data(), sizeof(BufferOffset), mOffsets.size());
    }
    
    void Decoder::reindexOffsets() {
        std::sort(mOffsets.begin(), mOffsets.end(), [](const BufferOffset& a, const BufferOffset& b) {
            return a.timestamp < b.timestamp;
        });
        
        mFrameList.clear();
        mFrameOffsetMap.clear();
        mFrameList.reserve(mOffsets.size());
        for (const auto& off : mOffsets) {
            mFrameList.push_back(off.timestamp);
            mFrameOffsetMap.emplace(off.timestamp, off);
        }
    }
    
    void Decoder::readExtra() {
        if (mOffsets.empty())
            return;
        
        auto curOffset = mOffsets.back().offset;
        if (FSEEK(mFile, curOffset, SEEK_SET) != 0)
            return;

        while (true) {
            Item item{};
            if (std::fread(&item, sizeof(Item), 1, mFile) != 1)
                break;
            
            if (item.type == Type::BUFFER || item.type == Type::METADATA ||
                item.type == Type::AUDIO_DATA || item.type == Type::AUDIO_DATA_METADATA) {
                if (FSEEK(mFile, item.size, SEEK_CUR) != 0)
                    break;
            }
            else if (item.type == Type::AUDIO_INDEX) {
                AudioIndex index{};
                read(&index, sizeof(AudioIndex));
                mAudioOffsets.resize(index.numOffsets);
                read(mAudioOffsets.data(), sizeof(BufferOffset), mAudioOffsets.size());
            }
            else {
                break;
            }
        }
    }
    
    void Decoder::read(void* data, size_t size, size_t items) const {
        if (std::fread(data, size, items, mFile) != items)
            throw IOException("Failed to read data");
    }

} // namespace motioncam