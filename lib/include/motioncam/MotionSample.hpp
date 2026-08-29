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

#ifndef MotionSample_hpp
#define MotionSample_hpp

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace motioncam {
    struct MotionSample {
        int64_t timestampNs = 0;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        uint32_t reserved = 0;
    };

    static_assert(sizeof(MotionSample) == 24);
    static_assert(offsetof(MotionSample, timestampNs) == 0);
    static_assert(offsetof(MotionSample, x) == 8);
    static_assert(offsetof(MotionSample, y) == 12);
    static_assert(offsetof(MotionSample, z) == 16);
    static_assert(offsetof(MotionSample, reserved) == 20);
    static_assert(std::is_standard_layout_v<MotionSample>);
    static_assert(std::is_trivially_copyable_v<MotionSample>);
}

#endif /* MotionSample_hpp */
