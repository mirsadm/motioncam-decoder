#include <motioncam/Decoder.hpp>
#include <motioncam/RawData.hpp>

#include <cstdio>
#include <cstring>

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
        if(!mFile)
            throw IOException("Invalid file");
            
        init();
    }

    Decoder::Decoder(const std::string& path) : mFile(std::fopen(path.c_str(), "rb")) {
        if(!mFile)
            throw IOException("Failed to open " + path);
            
        init();
    }
    
    Decoder::~Decoder() {
        if(mFile)
            std::fclose(mFile);
    }
    
    void Decoder::init() {
        Header header{};
        
        // Check validity of file
        read(&header, sizeof(Header));

        // Support current version and also version 3
        if((header.version != CONTAINER_VERSION))
            throw IOException("Invalid container version");

        if(std::memcmp(header.ident, CONTAINER_ID, sizeof(CONTAINER_ID)) != 0)
            throw IOException("Invalid header id");

        // Read camera metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        
        if(metadataItem.type != Type::METADATA)
            throw IOException("Invalid camera metadata");
        
        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        
        // Keep the camera metadata
        auto cameraMetadataString = std::string(metadataJson.begin(), metadataJson.end());
        mMetadata = nlohmann::json::parse(cameraMetadataString);
  
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
        for(const auto& o : mAudioOffsets) {
            if(FSEEK(mFile, o.offset, SEEK_SET) != 0)
                break;
            
            // Get audio data header
            Item audioDataItem{};
            read(&audioDataItem, sizeof(Item));
            
            if(audioDataItem.type != Type::AUDIO_DATA)
                throw IOException("Invalid audio data");
            
            // Read into temporary buffer
            std::vector<int16_t> tmp;

            tmp.resize((audioDataItem.size + 1) / 2);
            read((void*)tmp.data(), audioDataItem.size);

            // Metadata should follow (this was added later so some files may not have it)
            Item audioMetadataItem{};
            read(&audioMetadataItem, sizeof(Item));
            
            Timestamp audioTimestamp = -1;
            
            if(audioMetadataItem.type == Type::AUDIO_DATA_METADATA) {
                AudioMetadata metadata;
                
                read(&metadata, sizeof(AudioMetadata));
                audioTimestamp = metadata.timestampNs;
            }

            // Add audio chunk
            outAudioChunks.push_back(std::make_pair(audioTimestamp, std::move(tmp)));
        }
    }
    
    void Decoder::loadFrame(const Timestamp timestamp, std::vector<uint16_t>& outData, nlohmann::json& outMetadata) {
        if(mFrameOffsetMap.find(timestamp) == mFrameOffsetMap.end())
            throw IOException("Frame not found (timestamp: " + std::to_string(timestamp) + ")");
        
        int64_t offset = mFrameOffsetMap.at(timestamp).offset;
        
        if(FSEEK(mFile, offset, SEEK_SET) != 0)
            throw IOException("Invalid offset");
        
        Item bufferItem{};
        read(&bufferItem, sizeof(Item));

        if(bufferItem.type != Type::BUFFER)
            throw IOException("Invalid buffer type");

        mTmpBuffer.resize(bufferItem.size);

        read(mTmpBuffer.data(), bufferItem.size);
                
        // Get metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        
        if(metadataItem.type != Type::METADATA)
            throw IOException("Invalid metadata");
        
        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        
        std::string metadataString = std::string(metadataJson.begin(), metadataJson.end());
        outMetadata = nlohmann::json::parse(metadataString);        
        
        const int width = outMetadata["width"];
        const int height = outMetadata["height"];
        const int compressionType = outMetadata["compressionType"];
        
        if(compressionType != MOTIONCAM_COMPRESSION_TYPE)
            throw IOException("Invalid compression type");
            
        // Decompress the buffer
        const size_t outputSizeBytes = sizeof(uint16_t) * width*height;
        outData.resize(outputSizeBytes);
        
        if(raw::Decode(outData.data(), width, height, mTmpBuffer.data(), mTmpBuffer.size()) <= 0)
            throw IOException("Failed to uncompress frame");
    }

    void Decoder::readIndex() {
        // Seek to index item
        if(FSEEK(mFile, -static_cast<long>(sizeof(BufferIndex) + sizeof(Item)), SEEK_END) != 0)
            throw IOException("Failed to get end chunk");

        Item bufferIndexItem{};
        read(&bufferIndexItem, sizeof(Item));
        
        if(bufferIndexItem.type != Type::BUFFER_INDEX)
            throw IOException("Invalid file");
        
        BufferIndex index{};
        read(&index, sizeof(BufferIndex));
        
        // Check validity of index
        if(index.magicNumber != INDEX_MAGIC_NUMBER)
            throw IOException("Corrupted file");
        
        mOffsets.resize(index.numOffsets);
        
        // Read the index
        if(FSEEK(mFile, index.indexDataOffset, SEEK_SET) != 0) {
            throw IOException("Invalid index");
            return;
        }
        
        read(mOffsets.data(), sizeof(BufferOffset), mOffsets.size());
    }
    
    void Decoder::reindexOffsets() {
        // Sort offsets so they are in order of timestamps
        std::sort(mOffsets.begin(), mOffsets.end(), [](const auto& a, const auto&b) {
            return a.timestamp < b.timestamp;
        });
        
        mFrameList.clear();
        mFrameOffsetMap.clear();
        
        for(const auto& i : mOffsets) {
            mFrameList.push_back(i.timestamp);
            mFrameOffsetMap.insert({ i.timestamp, i });
        }
    }
    
    void Decoder::readExtra() {
        if(mOffsets.empty())
            return;
        
        auto curOffset = mOffsets[mOffsets.size() - 1].offset;

        if(FSEEK(mFile, curOffset, SEEK_SET) != 0)
            return;

        while(true) {
            Item item{};

            if(std::fread(&item, sizeof(Item), 1, mFile) != 1)
                break;
            
            // Skip things we don't need
            if(item.type == Type::BUFFER || item.type == Type::METADATA || item.type == Type::AUDIO_DATA) {
                if(FSEEK(mFile, item.size, SEEK_CUR) != 0)
                    break;
            }
            else if(item.type == Type::AUDIO_INDEX) {
                AudioIndex index{};
                
                read(&index, sizeof(AudioIndex));

                // Read all audio offsets
                mAudioOffsets.resize(index.numOffsets);
                
                read(mAudioOffsets.data(), sizeof(BufferOffset), mAudioOffsets.size());
            }
            else {
                break;
            }
        }
    }
    
    void Decoder::read(void* data, size_t size, size_t items) const {
        if(std::fread(data, size, items, mFile) != items) {
            throw IOException("Failed to read data");
        }
    }

} // namespace motioncam
