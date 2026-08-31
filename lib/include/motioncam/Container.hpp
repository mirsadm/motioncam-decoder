/*
 * Copyright 2023 MotionCam
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

#ifndef Container_h
#define Container_h

#include <cstdint>
#include <type_traits>

namespace motioncam {
    const uint32_t INDEX_MAGIC_NUMBER = 0x8A905612;
    const uint32_t GYRO_DATA_VERSION = 1;
    const uint32_t GYRO_INDEX_VERSION = 1;
    
    const uint8_t CONTAINER_VERSION = 3;
    const uint8_t CONTAINER_ID[7] = {'M', 'O', 'T', 'I', 'O', 'N', ' '};

    struct Header {
        uint8_t ident[7];
        uint8_t version;
    };

    enum VideoType {
        VIDEO,
        TIMELAPSE
    };
    
    enum class Type : uint32_t {
        BUFFER_INDEX = 0,
        BUFFER_INDEX_DATA = 1,
        BUFFER = 2,
        METADATA = 3,
        AUDIO_INDEX = 4,
        AUDIO_DATA = 5,
        AUDIO_DATA_METADATA = 6,
        AUDIO_DATA_F32 = 7,
        GYRO_INDEX = 8,
        GYRO_DATA = 9
    };

    struct Item {
        Type type;
        uint32_t size;
    };

    struct BufferOffset {
        int64_t offset;
        int64_t timestamp;
    };

    struct BufferIndex {
        int32_t magicNumber;
        int32_t numOffsets;
        int64_t indexDataOffset;
    };

    struct AudioIndex {
        int64_t numOffsets;
        int64_t startTimestampMs;
    };

    struct AudioMetadata {
        int64_t timestampNs;
    };

    struct GyroDataHeader {
        uint32_t version;
        uint32_t numSamples;
    };

    struct GyroIndex {
        uint32_t version;
        uint32_t numOffsets;
    };

    static_assert(sizeof(GyroDataHeader) == 8);
    static_assert(sizeof(GyroIndex) == 8);
    static_assert(std::is_trivially_copyable_v<GyroDataHeader>);
    static_assert(std::is_trivially_copyable_v<GyroIndex>);
}

#endif /* Container_h */
