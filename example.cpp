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

#define FUSE_USE_VERSION 29

#include <fuse.h>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <mutex>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <sys/statvfs.h>
#include <cstring>    // for strdup, strerror
#include <libgen.h>   // for dirname(), basename()
#include <sys/stat.h> // for mkdir
#include <errno.h>

#include <motioncam/Decoder.hpp>
#include <audiofile/AudioFile.h>

#define TINY_DNG_WRITER_IMPLEMENTATION
    #include <tinydng/tiny_dng_writer.h>
#undef TINY_DNG_WRITER_IMPLEMENTATION

void writeAudio(
    const std::string& outputPath,
    const int sampleRateHz,
    const int numChannels,
    std::vector<motioncam::AudioChunk>& audioChunks)
{
    AudioFile<int16_t> audio;
    
    audio.setNumChannels(numChannels);
    audio.setSampleRate(sampleRateHz);
    
    if(numChannels == 2) {
        for(auto& x : audioChunks) {
            for(auto i = 0; i < x.second.size(); i+=2) {
                audio.samples[0].push_back(x.second[i]);
                audio.samples[1].push_back(x.second[i+1]);
            }
        }
    }
    else if(numChannels == 1) {
        for(auto& x : audioChunks) {
            for(auto i = 0; i < x.second.size(); i++)
                audio.samples[0].push_back(x.second[i]);
        }
    }
    
    audio.save(outputPath);
}

// globals
static motioncam::Decoder *decoder = nullptr;
static nlohmann::json containerMetadata;
static std::vector<std::string> filenames;
static std::map<std::string, std::string> frameCache;
static std::mutex frameCacheMutex;
static const size_t MAX_CACHE_FRAMES = 10;
static std::deque<std::string> frameCacheOrder;
static size_t frameSize = 0;
static std::mutex frameSizeMutex;
static std::mutex decoderMutex;
static std::vector<motioncam::Timestamp> frameList;

// container globals
static std::vector<uint16_t> blackLevels;
static double whiteLevel = 0.0;
static std::array<uint8_t, 4> cfa = {{0, 1, 1, 2}};
static bool gHasSoftware = false;
static std::string gSoftware;
static bool gHasOrientation = false;
static uint16_t gOrientation = 1;
static std::vector<float> colorMatrix1,
    colorMatrix2,
    forwardMatrix1,
    forwardMatrix2;

// call this once, right after gContainerMetadata is set:
static void cache_container_metadata()
{
    // Black levels
    std::vector<uint16_t> blackLevel = containerMetadata["blackLevel"];
    blackLevels.clear();
    blackLevels.reserve(blackLevel.size());
    for (float v : blackLevel)
        blackLevels.push_back(uint16_t(std::lround(v)));

    // White level
    whiteLevel = containerMetadata["whiteLevel"];

    // CFA pattern
    std::string sensorArrangement = containerMetadata["sensorArrangment"];
    if (sensorArrangement == "rggb")
        cfa = {{0, 1, 1, 2}};
    else if (sensorArrangement == "bggr")
        cfa = {{2, 1, 1, 0}};
    else if (sensorArrangement == "grbg")
        cfa = {{1, 0, 2, 1}};
    else if (sensorArrangement == "gbrg")
        cfa = {{1, 2, 0, 1}};
    else
        cfa = {{0, 1, 1, 2}};

    // Software & orientation (optional)
    if (containerMetadata.contains("software"))
    {
        gSoftware = containerMetadata["software"].get<std::string>();
        gHasSoftware = true;
    }
    if (containerMetadata.contains("orientation"))
    {
        gOrientation = uint16_t(containerMetadata["orientation"].get<int>());
        gHasOrientation = true;
    }

    // Color/forward matrices
    colorMatrix1 = containerMetadata["colorMatrix1"].get<std::vector<float>>();
    colorMatrix2 = containerMetadata["colorMatrix2"].get<std::vector<float>>();
    forwardMatrix1 = containerMetadata["forwardMatrix1"].get<std::vector<float>>();
    forwardMatrix2 = containerMetadata["forwardMatrix2"].get<std::vector<float>>();
}

static std::string frameName(int i)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "frame_%06d.dng", i);
    return buf;
}

// decode one frame into gCache[path]
// after writing to cache, if this is the first frame, record its size
static int load_frame(const std::string &path)
{
    { // fast‐path if cached
        std::lock_guard<std::mutex> lk(frameCacheMutex);
        if (frameCache.count(path))
            return 0;
    }

    // find the frame index
    int idx = -1;
    for (size_t i = 0; i < filenames.size(); ++i)
        if (filenames[i] == path)
        {
            idx = int(i);
            break;
        }
    if (idx < 0)
        return -ENOENT;

    // decode raw + per‐frame metadata
    std::vector<uint16_t> raw;
    nlohmann::json metadata;
    try
    {
        auto ts = frameList[idx];
        std::lock_guard<std::mutex> dlk(decoderMutex);
        decoder->loadFrame(ts, raw, metadata);
    }
    catch (std::exception &e)
    {
        std::cerr << "EIO error: " << e.what() << "\n";
        return -EIO;
    }

    // pack into a DNGImage
    tinydngwriter::DNGImage dng;
    unsigned w = metadata["width"], h = metadata["height"];
    std::vector<float> asShotNeutral = metadata["asShotNeutral"];
    dng.SetBigEndian(false);
    dng.SetDNGVersion(1, 4, 0, 0);
    dng.SetDNGBackwardVersion(1, 1, 0, 0);
    dng.SetImageData(
        (const unsigned char *)raw.data(),
        raw.size());
    dng.SetImageWidth(w);
    dng.SetImageLength(h);
    dng.SetPlanarConfig(tinydngwriter::PLANARCONFIG_CONTIG);
    dng.SetPhotometric(tinydngwriter::PHOTOMETRIC_CFA);
    dng.SetRowsPerStrip(h);
    dng.SetSamplesPerPixel(1);
    dng.SetCFARepeatPatternDim(2, 2);
    
    dng.SetBlackLevelRepeatDim(2, 2);
    dng.SetBlackLevel(uint32_t(blackLevels.size()), blackLevels.data());
    dng.SetWhiteLevel(whiteLevel);
    dng.SetCompression(tinydngwriter::COMPRESSION_NONE);

    dng.SetCFAPattern(4, cfa.data());
    
    // Rectangular
    dng.SetCFALayout(1);

    const uint16_t bps[1] = { 16 };
    dng.SetBitsPerSample(1, bps);
    
    dng.SetColorMatrix1(3, colorMatrix1.data());
    dng.SetColorMatrix2(3, colorMatrix2.data());

    dng.SetForwardMatrix1(3, forwardMatrix1.data());
    dng.SetForwardMatrix2(3, forwardMatrix2.data());
    
    dng.SetAsShotNeutral(3, asShotNeutral.data());
    
    dng.SetCalibrationIlluminant1(21);
    dng.SetCalibrationIlluminant2(17);
    
    dng.SetUniqueCameraModel("MotionCam");
    dng.SetSubfileType();
    
    const uint32_t activeArea[4] = { 0, 0, h, w };
    dng.SetActiveArea(&activeArea[0]);

    // Write DNG
    std::string err;
    tinydngwriter::DNGWriter writer(false);
    writer.AddImage(&dng);
    std::ostringstream oss;
    if (!writer.WriteToFile(oss, &err))
    {
        std::cerr << "DNG pack error: " << err << "\n";
        return -EIO;
    }

    // insert into rolling‐buffer cache
    {
        std::lock_guard<std::mutex> lk(frameCacheMutex);
        if (frameCache.size() >= MAX_CACHE_FRAMES)
        {
            frameCache.erase(frameCacheOrder.front());
            frameCacheOrder.pop_front();
        }
        frameCache[path] = oss.str();
        frameCacheOrder.push_back(path);
    }

    // record frame‐size once
    {
        std::lock_guard<std::mutex> lk(frameSizeMutex);
        if (frameSize == 0)
        {
            std::lock_guard<std::mutex> lk2(frameCacheMutex);
            frameSize = frameCache[path].size();
        }
    }

    return 0;
}

// report uniform size (0 until we have it)
static int fs_getattr(const char *path, struct stat *st)
{
    std::cout << "fs_getattr";
    std::cout << path;
    std::cout << "\n";
    memset(st, 0, sizeof(*st));
    if (strcmp(path, "/") == 0)
    {
        st->st_mode = S_IFDIR | 0555;
        st->st_nlink = 2;
        return 0;
    }

    st->st_mode = S_IFREG | 0444;
    st->st_nlink = 1;
    {
        std::lock_guard<std::mutex> lk(frameSizeMutex);
        st->st_size = (off_t)frameSize;
    }
    return 0;
}

static int fs_readdir(const char *path, void *buf,
                      fuse_fill_dir_t filler,
                      off_t offset, struct fuse_file_info *fi)
{
    std::cout << "fs_readdir";
    std::cout << path;
    std::cout << "\n";
    (void)offset;
    (void)fi;
    if (strcmp(path, "/") != 0)
        return -ENOENT;
    filler(buf, ".", nullptr, 0);
    filler(buf, "..", nullptr, 0);
    for (auto &f : filenames)
        filler(buf, f.c_str(), nullptr, 0);
    return 0;
}

static int fs_opendir(const char *path, struct fuse_file_info *fi)
{
    std::cout << "fs_opendir";
    std::cout << path;
    std::cout << "\n";
    if (strcmp(path, "/") != 0)
        return -ENOENT;
    return 0;
}

static int fs_releasedir(const char *path, struct fuse_file_info *fi)
{
    std::cout << "fs_releasedir";
    std::cout << path;
    std::cout << "\n";
    (void)path;
    (void)fi;
    return 0;
}

static int fs_open(const char *path, struct fuse_file_info *fi)
{
    std::cout << "fs_open" << path << "\n";
    std::string fn = path + 1;
    if (std::find(filenames.begin(), filenames.end(), fn) == filenames.end())
        return -ENOENT;
    if ((fi->flags & 3) != O_RDONLY)
        return -EACCES;

    return 0;
}

// do the expensive decoding here, once per file
static int fs_read(const char *path,
                   char *buf,
                   size_t size,
                   off_t offset,
                   struct fuse_file_info *fi)
{
    (void)fi;
    std::cout << "fs_read" << path << "\n";

    std::string fn = path + 1;

    // 1) trigger the lazy‐decode if we haven't already cached it
    int err = load_frame(fn);
    if (err < 0)
        return err;

    // 2) we know it's in gCache now; copy out the bytes
    std::lock_guard<std::mutex> lk(frameCacheMutex);
    auto it = frameCache.find(fn);
    if (it == frameCache.end())
        return -ENOENT; // should never happen, load_frame just inserted it

    const std::string &data = it->second;
    if ((size_t)offset >= data.size())
        return 0;

    size_t tocopy = std::min<size_t>(size, data.size() - (size_t)offset);
    memcpy(buf, data.data() + offset, tocopy);
    return (ssize_t)tocopy;
}

static int fs_statfs(const char *path, struct statvfs *st)
{
    std::cout << "fs_statfs";
    std::cout << path;
    std::cout << "\n";
    (void)path;
    memset(st, 0, sizeof(*st));
    st->f_bsize = 4096;
    st->f_frsize = 4096;
    st->f_blocks = 1024 * 1024;
    st->f_bfree = 0;
    st->f_bavail = 0;
    st->f_files = filenames.size();
    st->f_ffree = 0;
    return 0;
}

static int fs_listxattr(const char *path, char *list, size_t size)
{
    std::cout << "fs_listxattr";
    std::cout << path;
    std::cout << "\n";
    (void)path;
    (void)list;
    (void)size;
    return 0;
}

static struct fuse_operations fs_ops = {
    .getattr = fs_getattr,
    .readdir = fs_readdir,
    .opendir = fs_opendir,
    .releasedir = fs_releasedir,
    .statfs = fs_statfs,
    .listxattr = fs_listxattr,
    .open = fs_open,
    .read = fs_read,
};
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <input.motioncam>\n";
        return 1;
    }

    // 1) figure out mount‐point directory: same folder, same basename (no .motioncam)
    std::string inputPath = argv[1];
    // make writable copies for dirname()/basename()
    char *dup1 = strdup(inputPath.c_str());
    char *dup2 = strdup(inputPath.c_str());

    std::string parentDir = dirname(dup1); // e.g. "/foo/bar"
    std::string fileName = basename(dup2); // e.g. "video.mcraw"

    free(dup1);
    free(dup2);

    // strip extension from filename
    std::string base = fileName;
    auto dot = base.rfind('.');
    if (dot != std::string::npos)
        base.erase(dot);

    // assemble mount‐point path
    std::string mountPoint = parentDir + "/" + base;

    // create the directory if it doesn't exist
    if (::mkdir(mountPoint.c_str(), 0755) != 0 && errno != EEXIST)
    {
        std::cerr << "Error creating mountpoint '" << mountPoint
                  << "': " << strerror(errno) << "\n";
        return 1;
    }

    // 2) open decoder
    try
    {
        decoder = new motioncam::Decoder(inputPath);
    }
    catch (std::exception &e)
    {
        std::cerr << "Decoder error: " << e.what() << "\n";
        return 1;
    }

    // 3) preload metadata & frame‐list
    frameList = decoder->getFrames();
    containerMetadata = decoder->getContainerMetadata();
    cache_container_metadata();

    std::cerr << "DEBUG: found " << frameList.size() << " frames\n";
    for (size_t i = 0; i < frameList.size(); ++i)
        filenames.push_back(frameName(int(i)));

    // 4) warm up first frame so gFrameSize is known
    if (!filenames.empty())
    {
        if (int err = load_frame(filenames[0]); err < 0)
        {
            std::cerr << "Failed to load first frame: " << err << "\n";
            return 1;
        }
    }

    // 5) build FUSE argv and run
    int fuse_argc = 5;
    char *fuse_argv[6];
    fuse_argv[0] = argv[0];
    fuse_argv[1] = (char *)"-f";
    fuse_argv[2] = (char *)"-o";
    std::string last = mountPoint.substr(mountPoint.find_last_of('/') + 1);
    std::string mountOptions = "noappledouble,nobrowse,rdonly,noapplexattr,volname=" + last;
    fuse_argv[3] = (char *)mountOptions.c_str();
    // finally, our auto‐created mountpoint:
    fuse_argv[4] = const_cast<char *>(mountPoint.c_str());
    fuse_argv[5] = nullptr;

    return fuse_main(fuse_argc, fuse_argv, &fs_ops, nullptr);
}