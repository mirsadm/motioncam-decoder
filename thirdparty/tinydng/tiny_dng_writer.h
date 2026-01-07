//
// TinyDNGWriter, single header only DNG writer in C++11.
//

/*
The MIT License (MIT)

Copyright (c) 2016 - 2020 Syoyo Fujita.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef TINY_DNG_WRITER_H_
#define TINY_DNG_WRITER_H_

#include <sstream>
#include <vector>

#ifndef ROL32
#define ROL32(v,a) ((v) << (a) | (v) >> (32-(a)))
#endif

#ifndef ROL16
#define ROL16(v,a) ((v) << (a) | (v) >> (16-(a)))
#endif

// low-level read and write functions
#ifdef _MSC_VER
# include <io.h>
#else
# include <unistd.h>
//extern "C" {
//    int write (int fd, const char* buf, int num);
//    int read (int fd, char* buf, int num);
//}
#endif


// BEGIN namespace BOOST
namespace boost {


/************************************************************
 * fdostream
 * - a stream that writes on a file descriptor
 ************************************************************/


class fdoutbuf : public std::streambuf {
  protected:
    int fd;    // file descriptor
  public:
    // constructor
    fdoutbuf (int _fd) : fd(_fd) {
    }
  protected:
    // write one character
    virtual int_type overflow (int_type c) {
        if (c != EOF) {
            char z = c;
            if (write (fd, &z, 1) != 1) {
                return EOF;
            }
        }
        return c;
    }
    // write multiple characters
    virtual
    std::streamsize xsputn (const char* s,
                            std::streamsize num) {
        return write(fd,s,num);
    }
};

class fdostream : public std::ostream {
  protected:
    fdoutbuf buf;
  public:
    fdostream (int fd) : std::ostream(0), buf(fd) {
        rdbuf(&buf);
    }
};

} // END namespace boost

namespace tinydngwriter {

typedef enum {
  TIFFTAG_SUB_FILETYPE = 254,
  TIFFTAG_IMAGE_WIDTH = 256,
  TIFFTAG_IMAGE_LENGTH = 257,
  TIFFTAG_BITS_PER_SAMPLE = 258,
  TIFFTAG_COMPRESSION = 259,
  TIFFTAG_PHOTOMETRIC = 262,
  TIFFTAG_IMAGEDESCRIPTION = 270,
  TIFFTAG_MAKE = 271,
  TIFFTAG_CAMERA_MODEL_NAME = 272,
  TIFFTAG_STRIP_OFFSET = 273,
  TIFFTAG_SAMPLES_PER_PIXEL = 277,
  TIFFTAG_ROWS_PER_STRIP = 278,
  TIFFTAG_STRIP_BYTE_COUNTS = 279,
  TIFFTAG_PLANAR_CONFIG = 284,
  TIFFTAG_ORIENTATION = 274,

  TIFFTAG_XRESOLUTION = 282,  // rational
  TIFFTAG_YRESOLUTION = 283,  // rational
  TIFFTAG_RESOLUTION_UNIT = 296,

  TIFFTAG_SOFTWARE = 305,

  TIFFTAG_SAMPLEFORMAT = 339,

  // DNG extension
  TIFFTAG_CFA_REPEAT_PATTERN_DIM = 33421,
  TIFFTAG_CFA_PATTERN = 33422,

  TIFFTAG_CAMERA_EXPOSURE_TIME = 33434,
  TIFFTAG_CAMERA_ISO = 34855,

  TIFFTAG_CFA_LAYOUT = 50711,

  TIFFTAG_DNG_VERSION = 50706,
  TIFFTAG_DNG_BACKWARD_VERSION = 50707,
  TIFFTAG_UNIQUE_CAMERA_MODEL = 50708,
  TIFFTAG_CHRROMA_BLUR_RADIUS = 50703,
  TIFFTAG_BLACK_LEVEL_REPEAT_DIM = 50713,
  TIFFTAG_BLACK_LEVEL = 50714,
  TIFFTAG_WHITE_LEVEL = 50717,
  TIFFTAG_COLOR_MATRIX1 = 50721,
  TIFFTAG_COLOR_MATRIX2 = 50722,
  TIFFTAG_CAMERA_CALIBRATION1 = 50723,
  TIFFTAG_CAMERA_CALIBRATION2 = 50724,
  TIFFTAG_ANALOG_BALANCE = 50727,
  TIFFTAG_AS_SHOT_NEUTRAL = 50728,
  TIFFTAG_AS_SHOT_WHITE_XY = 50729,
  TIFFTAG_BASELINE_EXPOSURE = 50730, // per file exposure compensation offset recognized by davinci resolve
  TIFFTAG_CALIBRATION_ILLUMINANT1 = 50778,
  TIFFTAG_CALIBRATION_ILLUMINANT2 = 50779,
  TIFFTAG_EXTRA_CAMERA_PROFILES = 50933,
  TIFFTAG_PROFILE_NAME = 50936,
  TIFFTAG_AS_SHOT_PROFILE_NAME = 50934,
  TIFFTAG_DEFAULT_BLACK_RENDER = 51110,
  TIFFTAG_ACTIVE_AREA = 50829,
  TIFFTAG_FORWARD_MATRIX1 = 50964,
  TIFFTAG_FORWARD_MATRIX2 = 50965,
  TIFFTAG_LINEARIZATION_TABLE = 50712,

  // Noise profile
  TIFFTAG_NOISE_PROFILE = 51041,

  // CinemaDNG specific
  TIFFTAG_TIMECODE = 51043,
  TIFFTAG_FPS = 51044,

  // Opcode Lists
  TIFFTAG_OPCODE_LIST1 = 51008,
  TIFFTAG_OPCODE_LIST2 = 51009,
  TIFFTAG_OPCODE_LIST3 = 51022
} Tag;

// SUBFILETYPE(bit field)
static const int FILETYPE_REDUCEDIMAGE = 1;
static const int FILETYPE_PAGE = 2;
static const int FILETYPE_MASK = 4;

// PLANARCONFIG
static const int PLANARCONFIG_CONTIG = 1;
static const int PLANARCONFIG_SEPARATE = 2;

// COMPRESSION
// TODO(syoyo) more compressin types.
static const int COMPRESSION_NONE = 1;

// ORIENTATION
static const int ORIENTATION_TOPLEFT = 1;
static const int ORIENTATION_TOPRIGHT = 2;
static const int ORIENTATION_BOTRIGHT = 3;
static const int ORIENTATION_BOTLEFT = 4;
static const int ORIENTATION_LEFTTOP = 5;
static const int ORIENTATION_RIGHTTOP = 6;
static const int ORIENTATION_RIGHTBOT = 7;
static const int ORIENTATION_LEFTBOT = 8;

// RESOLUTIONUNIT
static const int RESUNIT_NONE = 1;
static const int RESUNIT_INCH = 2;
static const int RESUNIT_CENTIMETER = 2;

// PHOTOMETRIC
// TODO(syoyo): more photometric types.
static const int PHOTOMETRIC_WHITE_IS_ZERO = 0;  // For bilevel and grayscale
static const int PHOTOMETRIC_BLACK_IS_ZERO = 1;  // For bilevel and grayscale
static const int PHOTOMETRIC_RGB = 2;            // Default
static const int PHOTOMETRIC_CFA = 32803;        // DNG ext
static const int PHOTOMETRIC_LINEARRAW = 34892;  // DNG ext

// Sample format
static const int SAMPLEFORMAT_UINT = 1;  // Default
static const int SAMPLEFORMAT_INT = 2;
static const int SAMPLEFORMAT_IEEEFP = 3;  // floating point

// Opcode IDs
static const int OPCODE_WARP_RECTILINEAR = 1;
static const int OPCODE_FIX_BAD_PIXELS_LIST = 5;
static const int OPCODE_GAIN_MAP = 9;

struct IFDTag {
  unsigned short tag;
  unsigned short type;
  unsigned int count;
  unsigned int offset_or_value;
};
// 12 bytes.

// Opcode structures
struct BadPixel {
  unsigned int row;
  unsigned int column;
};

struct BadRect {
  unsigned int top;
  unsigned int left;
  unsigned int bottom;
  unsigned int right;
};

struct WarpRectilinearParams {
  unsigned int num_coeff_sets;
  std::vector<double> kr0, kr1, kr2, kr3;  // Radial coefficients
  std::vector<double> kt0, kt1;            // Tangential coefficients
  double cx_hat, cy_hat;                   // Normalized optical center
};

struct FixBadPixelsParams {
  unsigned int bayer_phase;
  std::vector<BadPixel> bad_pixels;
  std::vector<BadRect> bad_rects;
};

struct GainMapParams {
  unsigned int top, left, bottom, right;
  unsigned int plane, planes;
  unsigned int row_pitch, col_pitch;
  unsigned int map_points_v, map_points_h;
  double map_spacing_v, map_spacing_h;
  double map_origin_v, map_origin_h;
  unsigned int map_planes;
  std::vector<float> gain_data;
};

class OpcodeList {
public:
  OpcodeList() = default;
  
  void AddWarpRectilinear(const WarpRectilinearParams& params);
  void AddFixBadPixelsList(const FixBadPixelsParams& params);
  void AddGainMap(const GainMapParams& params);
  
  std::vector<unsigned char> Serialize() const;
  bool IsEmpty() const { return opcodes_.empty(); }

private:
  struct Opcode {
    unsigned int id;
    unsigned int version[4];  // DNG version
    unsigned int flags;
    std::vector<unsigned char> data;
  };
  
  std::vector<Opcode> opcodes_;
};

class DNGImage {
 public:
  DNGImage();
  ~DNGImage() {}

  ///
  /// Optional: Explicitly specify endian.
  /// Must be called before calling other Set methods.
  ///
  void SetBigEndian(bool big_endian);

  ///
  /// Default = 0
  ///
  bool SetSubfileType(bool reduced_image = false, bool page = false,
                      bool mask = false);

  bool SetImageWidth(unsigned int value);
  bool SetImageLength(unsigned int value);
  bool SetRowsPerStrip(unsigned int value);
  bool SetSamplesPerPixel(unsigned short value);
  // Set bits for each samples
  bool SetBitsPerSample(const unsigned int num_samples,
                        const unsigned short *values);
  bool SetPhotometric(unsigned short value);
  bool SetPlanarConfig(unsigned short value);
  bool SetOrientation(unsigned short value);
  bool SetCompression(unsigned short value);
  bool SetSampleFormat(const unsigned int num_samples,
                       const unsigned short *values);
  bool SetXResolution(float value);
  bool SetYResolution(float value);
  bool SetResolutionUnit(const unsigned short value);

  bool SetFrameRate(float value);
  bool SetTimeCode(unsigned char timecode[8]);
  bool SetExposureTime(float exposureSecs);
  bool SetIso(unsigned short iso);

  ///
  /// Set arbitrary string for image description.
  /// Currently we limit to 1024*1024 chars at max.
  ///
  bool SetImageDescription(const std::string &ascii);

  ///
  /// Set arbitrary string for unique camera model name (not localized!).
  /// Currently we limit to 1024*1024 chars at max.
  ///
  bool SetUniqueCameraModel(const std::string &ascii);

  ///
  /// Set camera make (manufacturer) string.
  /// Currently we limit to 1024*1024 chars at max.
  ///
  bool SetMake(const std::string &ascii);

  ///
  /// Set camera model name string.
  /// Currently we limit to 1024*1024 chars at max.
  ///
  bool SetCameraModelName(const std::string &ascii);

  ///
  /// Set software description(string).
  /// Currently we limit to 4095 chars at max.
  ///
  bool SetSoftware(const std::string &ascii);

  bool SetActiveArea(const unsigned int values[4]);

  bool SetChromaBlurRadius(float value);

  /// Specify black level per sample.
  bool SetBlackLevel(const unsigned int num_samples, const unsigned short *values);

  /// Specify black level per sample (as rational values).
  bool SetBlackLevelRational(unsigned int num_samples, const float *values);

  /// Specify white level per sample.
  bool SetWhiteLevel(const short value);
  bool SetWhiteLevelRational(unsigned int num_samples, const float *values);

  /// Specify analog white balance from camera for raw values.
  bool SetAnalogBalance(const unsigned int plane_count, const float *matrix_values);

  /// Specify CFA repeating pattern dimensions.
  bool SetCFARepeatPatternDim(const unsigned short width, const unsigned short height);

  /// Specify black level repeating pattern dimensions.
  bool SetBlackLevelRepeatDim(const unsigned short width, const unsigned short height);

  bool SetCalibrationIlluminant1(const unsigned short value);
  bool SetCalibrationIlluminant2(const unsigned short value);

  /// Specify DNG version.
  bool SetDNGVersion(const unsigned char a, const unsigned char b, const unsigned char c, const unsigned char d);
  bool SetDNGBackwardVersion(const unsigned char a, const unsigned char b, const unsigned char c, const unsigned char d);

  /// Specify transformation matrix (XYZ to reference camera native color space values, under the first calibration illuminant).
  bool SetColorMatrix1(const unsigned int plane_count, const float *matrix_values);

  /// Specify transformation matrix (XYZ to reference camera native color space values, under the second calibration illuminant).
  bool SetColorMatrix2(const unsigned int plane_count, const float *matrix_values);

  bool SetForwardMatrix1(const unsigned int plane_count, const float *matrix_values);
  bool SetForwardMatrix2(const unsigned int plane_count, const float *matrix_values);

  bool SetCameraCalibration1(const unsigned int plane_count, const float *matrix_values);
  bool SetCameraCalibration2(const unsigned int plane_count, const float *matrix_values);

  /// Specify CFA geometric pattern (left-to-right, top-to-bottom).
  bool SetCFAPattern(const unsigned int num_components, const unsigned char *values);
  bool SetCFALayout(const unsigned short value);
  
  /// Specify the selected white balance at time of capture, encoded as the coordinates of a perfectly neutral color in linear reference space values.
  bool SetAsShotNeutral(const unsigned int plane_count, const float *matrix_values);

  /// Specify the the selected white balance at time of capture, encoded as x-y chromaticity coordinates.
  bool SetAsShotWhiteXY(const float x, const float y);

  /// Set baseline exposure value in EV units
  bool SetBaselineExposure(float value);

  /// Set noise profile (6 double values: noise model parameters for the camera sensor)
  bool SetNoiseProfile(const double values[6]);

  /// Set linearization table for converting stored values to linear values.
  /// The table maps stored pixel values to linear values using direct mapping.
  /// @param[in] table_size Number of entries in the linearization table (should be 2^input_bits)
  /// @param[in] table_values Array of linear output values (TIFF_SHORT format, 0-65535)
  bool SetLinearizationTable(const unsigned int table_size, const unsigned short *table_values);

  /// Generate and set linearization table for log-to-linear conversion.
  /// Creates a direct mapping table from log-encoded values to 16-bit linear values.
  /// @param[in] input_bits Bit depth of input log data (e.g., 10, 12, 14)
  /// @param[in] log_base Base of the logarithm (e.g., 2.0 for log2, 10.0 for log10)
  /// @param[in] black_level Black level in linear space (0-65535)
  /// @param[in] white_level White level in linear space (0-65535)
  bool SetLogLinearizationTable(const unsigned int input_bits, const float log_base = 2.0f, 
                               const unsigned short black_level = 0, const unsigned short white_level = 65535);

  /// Set opcode list 1 (applied to raw image as read from file)
  bool SetOpcodeList1(const OpcodeList& opcode_list);
  
  /// Set opcode list 2 (applied after mapping to linear reference values)
  bool SetOpcodeList2(const OpcodeList& opcode_list);
  
  /// Set opcode list 3 (applied after demosaicing)
  bool SetOpcodeList3(const OpcodeList& opcode_list);

  /// Set image data with packing (take 16-bit values and pack them to input_bpp values).
  bool SetImageDataPacked(const unsigned short *input_buffer, const int input_count, const unsigned int input_bpp, bool big_endian);

  /// Set image data.
  bool SetImageData(const unsigned char *data, const size_t data_len);

  /// Set custom field.
  bool SetCustomFieldLong(const unsigned short tag, const int value);
  bool SetCustomFieldULong(const unsigned short tag, const unsigned int value);

  size_t GetDataSize() const { return data_os_.str().length(); }

  size_t GetStripOffset() const { return data_strip_offset_; }
  size_t GetStripBytes() const { return data_strip_bytes_; }

  /// Write aux IFD data and strip image data to stream.
  bool WriteDataToStream(std::ostream *ofs) const;

  ///
  /// Write IFD to stream.
  ///
  /// @param[in] data_base_offset : Byte offset to data
  /// @param[in] strip_offset : Byte offset to image strip data
  ///
  /// TODO(syoyo): Support multiple strips
  ///
  bool WriteIFDToStream(const unsigned int data_base_offset,
                        const unsigned int strip_offset, std::ostream *ofs) const;

  std::string Error() const { return err_; }

 private:
  std::ostringstream data_os_;
  bool swap_endian_;
  bool dng_big_endian_;
  unsigned short num_fields_;
  unsigned int samples_per_pixels_;
  std::vector<unsigned short> bits_per_samples_;

  // TODO(syoyo): Support multiple strips
  size_t data_strip_offset_{0};
  size_t data_strip_bytes_{0};

  mutable std::string err_;  // Error message

  std::vector<IFDTag> ifd_tags_;
};

class DNGWriter {
 public:
  // TODO(syoyo): Use same endian setting with DNGImage.
  DNGWriter(bool big_endian);
  ~DNGWriter() {}

  ///
  /// Add DNGImage.
  /// It just retains the pointer of the image, thus
  /// application must not free resources until `WriteToFile` has been called.
  ///
  bool AddImage(const DNGImage *image) {
    images_.push_back(image);

    return true;
  }

  /// Write DNG to a file.
  /// Return error string to `err` when Write() returns false.
  /// Returns true upon success.
  bool WriteToFile(const char *filename, std::string *err) const;
  bool WriteToFile(int fd, std::string *err) const;
  bool WriteToFile(std::ostream& stream, std::string *err) const;
  
 private:
  bool swap_endian_;
  bool dng_big_endian_;  // Endianness of DNG file.

  std::vector<const DNGImage *> images_;
};

}  // namespace tinydngwriter

#endif  // TINY_DNG_WRITER_H_

#ifdef TINY_DNG_WRITER_IMPLEMENTATION

//
// TIFF format resources.
//
// http://c0de517e.blogspot.jp/2013/07/tiny-hdr-writer.html
// http://paulbourke.net/dataformats/tiff/ and
// http://partners.adobe.com/public/developer/en/tiff/TIFF6.pdf
//

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace tinydngwriter {

#ifdef __clang__
#pragma clang diagnostic push
#if __has_warning("-Wzero-as-null-pointer-constant")
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif
#endif

//
// TinyDNGWriter stores IFD table in the end of file so that offset to
// image data can be easily computed.
//
// +----------------------+
// |    header            |
// +----------------------+
// |                      |
// |  image & meta 0      |
// |                      |
// +----------------------+
// |                      |
// |  image & meta 1      |
// |                      |
// +----------------------+
//    ...
// +----------------------+
// |                      |
// |  image & meta N      |
// |                      |
// +----------------------+
// |                      |
// |  IFD 0               |
// |                      |
// +----------------------+
// |                      |
// |  IFD 1               |
// |                      |
// +----------------------+
//    ...
// +----------------------+
// |                      |
// |  IFD 2               |
// |                      |
// +----------------------+
//

// From tiff.h
typedef enum {
  TIFF_NOTYPE = 0,     /* placeholder */
  TIFF_BYTE = 1,       /* 8-bit unsigned integer */
  TIFF_ASCII = 2,      /* 8-bit bytes w/ last byte null */
  TIFF_SHORT = 3,      /* 16-bit unsigned integer */
  TIFF_LONG = 4,       /* 32-bit unsigned integer */
  TIFF_RATIONAL = 5,   /* 64-bit unsigned fraction */
  TIFF_SBYTE = 6,      /* !8-bit signed integer */
  TIFF_UNDEFINED = 7,  /* !8-bit untyped data */
  TIFF_SSHORT = 8,     /* !16-bit signed integer */
  TIFF_SLONG = 9,      /* !32-bit signed integer */
  TIFF_SRATIONAL = 10, /* !64-bit signed fraction */
  TIFF_FLOAT = 11,     /* !32-bit IEEE floating point */
  TIFF_DOUBLE = 12,    /* !64-bit IEEE floating point */
  TIFF_IFD = 13,       /* %32-bit unsigned integer (offset) */
  TIFF_LONG8 = 16,     /* BigTIFF 64-bit unsigned integer */
  TIFF_SLONG8 = 17,    /* BigTIFF 64-bit signed integer */
  TIFF_IFD8 = 18       /* BigTIFF 64-bit unsigned integer (offset) */
} DataType;

const static int kHeaderSize = 8;  // TIFF header size.

// floating point to integer rational value conversion
// https://stackoverflow.com/questions/51142275/exact-value-of-a-floating-point-number-as-a-rational
//
// Return error flag
static int FloatToRational(float x, float *numerator, float *denominator) {
  if (!std::isfinite(x)) {
    *numerator = *denominator = 0.0f;
    if (x > 0.0f) *numerator = 1.0f;
    if (x < 0.0f) *numerator = -1.0f;
    return 1;
  }

  // TIFF Rational use two uint32's, so reduce the bits
  int bdigits = FLT_MANT_DIG;
  int expo;
  *denominator = 1.0f;
  *numerator = std::frexp(x, &expo) * std::pow(2.0f, bdigits);
  expo -= bdigits;
  if (expo > 0) {
    *numerator *= std::pow(2.0f, expo);
  } else if (expo < 0) {
    expo = -expo;
    if (expo >= FLT_MAX_EXP - 1) {
      *numerator /= std::pow(2.0f, expo - (FLT_MAX_EXP - 1));
      *denominator *= std::pow(2.0f, FLT_MAX_EXP - 1);
      return fabs(*numerator) < 1.0f;
    } else {
      *denominator *= std::pow(2.0f, expo);
    }
  }

  while ((std::fabs(*numerator) > 0.0f) &&
         (std::fabs(std::fmod(*numerator, 2)) <
          std::numeric_limits<float>::epsilon()) &&
         (std::fabs(std::fmod(*denominator, 2)) <
          std::numeric_limits<float>::epsilon())) {
    *numerator /= 2.0f;
    *denominator /= 2.0f;
  }
  return 0;
}

static inline bool IsBigEndian() {
  unsigned int i = 0x01020304;
  char c[4];
  memcpy(c, &i, 4);
  return (c[0] == 1);
}

static void swap2(unsigned short *val) {
  unsigned short tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[1];
  dst[1] = src[0];
}

static void swap4(int *val) {
  unsigned int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
}

static void swap4(unsigned int *val) {
  unsigned int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
}

static void swap8(uint64_t *val) {
  uint64_t tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
}

static void Write1(const unsigned char c, std::ostringstream *out) {
  unsigned char value = c;
  out->write(reinterpret_cast<const char *>(&value), 1);
}

static void Write2(const unsigned short c, std::ostringstream *out,
                   const bool swap_endian) {
  unsigned short value = c;
  if (swap_endian) {
    swap2(&value);
  }

  out->write(reinterpret_cast<const char *>(&value), 2);
}

static void Write4(const unsigned int c, std::ostringstream *out,
                   const bool swap_endian) {
  unsigned int value = c;
  if (swap_endian) {
    swap4(&value);
  }

  out->write(reinterpret_cast<const char *>(&value), 4);
}

static void WriteDouble(const double c, std::ostringstream *out,
                        const bool swap_endian) {
  double value = c;
  if (swap_endian && !IsBigEndian()) {
    swap8(reinterpret_cast<uint64_t*>(&value));
  }
  out->write(reinterpret_cast<const char *>(&value), sizeof(double));
}

static void WriteFloat(const float c, std::ostringstream *out,
                       const bool swap_endian) {
  float value = c;
  if (swap_endian && !IsBigEndian()) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(float));
    swap4(&bits);
    memcpy(&value, &bits, sizeof(float));
  }
  out->write(reinterpret_cast<const char *>(&value), sizeof(float));
}

static bool WriteTIFFTag(const unsigned short tag, const unsigned short type,
                         const unsigned int count, const unsigned char *data,
                         std::vector<IFDTag> *tags_out,
                         std::ostringstream *data_out) {
  assert(sizeof(IFDTag) ==
         12);  // FIXME(syoyo): Use static_assert for C++11 compiler

  IFDTag ifd;
  ifd.tag = tag;
  ifd.type = type;
  ifd.count = count;

  size_t typesize_table[] = {1, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8, 4};

  size_t len = count * (typesize_table[(type) < 14 ? (type) : 0]);
  if (len > 4) {
    assert(data_out);
    if (!data_out) {
      return false;
    }

    // Store offset value.

    unsigned int offset =
        static_cast<unsigned int>(data_out->tellp()) + kHeaderSize;
    ifd.offset_or_value = offset;

    data_out->write(reinterpret_cast<const char *>(data),
                    static_cast<std::streamsize>(len));

  } else {
    ifd.offset_or_value = 0;

    // less than 4 bytes = store data itself.
    if (len == 1) {
      unsigned char value = *(data);
      memcpy(&(ifd.offset_or_value), &value, sizeof(unsigned char));
    } else if (len == 2) {
      unsigned short value = *(reinterpret_cast<const unsigned short *>(data));
      memcpy(&(ifd.offset_or_value), &value, sizeof(unsigned short));
    } else if (len == 4) {
      unsigned int value = *(reinterpret_cast<const unsigned int *>(data));
      ifd.offset_or_value = value;
    } else {
      assert(0);
    }
  }

  tags_out->push_back(ifd);

  return true;
}

static bool WriteTIFFVersionHeader(std::ostringstream *out, bool big_endian) {
  // TODO(syoyo): Support BigTIFF?

  // 4d 4d = Big endian. 49 49 = Little endian.
  if (big_endian) {
    Write1(0x4d, out);
    Write1(0x4d, out);
    Write1(0x0, out);
    Write1(0x2a, out);  // Tiff version ID
  } else {
    Write1(0x49, out);
    Write1(0x49, out);
    Write1(0x2a, out);  // Tiff version ID
    Write1(0x0, out);
  }

  return true;
}

DNGImage::DNGImage()
    : dng_big_endian_(true),
      num_fields_(0),
      samples_per_pixels_(0),
      data_strip_offset_{0},
      data_strip_bytes_{0} {
  swap_endian_ = (IsBigEndian() != dng_big_endian_);
}

void DNGImage::SetBigEndian(bool big_endian) {
  dng_big_endian_ = big_endian;
  swap_endian_ = (IsBigEndian() != dng_big_endian_);
}

bool DNGImage::SetSubfileType(bool reduced_image, bool page, bool mask) {
  unsigned int count = 1;

  unsigned int bits = 0;
  if (reduced_image) {
    bits |= FILETYPE_REDUCEDIMAGE;
  }
  if (page) {
    bits |= FILETYPE_PAGE;
  }
  if (mask) {
    bits |= FILETYPE_MASK;
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_SUB_FILETYPE), TIFF_LONG, count,
      reinterpret_cast<const unsigned char *>(&bits), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetImageWidth(const unsigned int width) {
  unsigned int count = 1;

  unsigned int data = width;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_IMAGE_WIDTH), TIFF_LONG, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetImageLength(const unsigned int length) {
  unsigned int count = 1;

  const unsigned int data = length;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_IMAGE_LENGTH), TIFF_LONG, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetRowsPerStrip(const unsigned int rows) {
  if (rows == 0) {
    return false;
  }

  unsigned int count = 1;

  const unsigned int data = rows;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_ROWS_PER_STRIP), TIFF_LONG, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetSamplesPerPixel(const unsigned short value) {
  if (value > 4) {
    {
      std::stringstream ss;
      ss << "Samples per pixel must be less than or equal to 4, but got " << value << ".\n";
      err_ += ss.str();
    }
    return false;
  }

  unsigned int count = 1;

  const unsigned short data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_SAMPLES_PER_PIXEL), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    err_ += "Failed to write `TIFFTAG_SAMPLES_PER_PIXEL` tag.\n";
    return false;
  }

  samples_per_pixels_ = value;  // Store SPP for later use.

  num_fields_++;
  return true;
}

bool DNGImage::SetBitsPerSample(const unsigned int num_samples,
                                const unsigned short *values) {
  // `SetSamplesPerPixel()` must be called in advance and SPP shoud be equal to
  // `num_samples`.

  if (samples_per_pixels_ == 0) {
    err_ += "SetSamplesPerPixel() must be called before SetBitsPerSample().\n";
    return false;
  }

  if ((num_samples == 0) || (num_samples > 4)) {
    std::stringstream ss;
    ss << "Invalid number of samples: " << num_samples << "\n";
    err_ += ss.str();
    return false;
  } else if (num_samples != samples_per_pixels_) {
    std::stringstream ss;
    ss << "Samples per pixel mismatch. " << num_samples << " is given for SetBitsPerSample(), but SamplesPerPixel is set to " << samples_per_pixels_ << "\n";
    err_ += ss.str();
    return false;
  } else {
    // ok
  }

  unsigned short bps = values[0];

  std::vector<unsigned short> vs(num_samples);
  for (size_t i = 0; i < vs.size(); i++) {
    // FIXME(syoyo): Currently bps must be same for all samples
    if (bps != values[i]) {
      err_ += "BitsPerSample must be same among samples at the moment.\n";
      return false;
    }

    vs[i] = values[i];

    // TODO(syoyo): Swap values when writing IFD tag, not here.
    if (swap_endian_) {
      swap2(&vs[i]);
    }
  }

  unsigned int count = num_samples;

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_BITS_PER_SAMPLE),
                          TIFF_SHORT, count,
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  // Store BPS for later use.
  bits_per_samples_.resize(num_samples);
  for (size_t i = 0; i < num_samples; i++) {
    bits_per_samples_[i] = values[i];
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetPhotometric(const unsigned short value) {
  if ((value == PHOTOMETRIC_LINEARRAW) ||
      (value == PHOTOMETRIC_CFA) ||
      (value == PHOTOMETRIC_RGB) ||
      (value == PHOTOMETRIC_WHITE_IS_ZERO) ||
      (value == PHOTOMETRIC_BLACK_IS_ZERO)) {
    // OK
  } else {
    return false;
  }

  unsigned int count = 1;

  const unsigned short data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_PHOTOMETRIC), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetPlanarConfig(const unsigned short value) {
  unsigned int count = 1;

  if ((value == PLANARCONFIG_CONTIG) || (value == PLANARCONFIG_SEPARATE)) {
    // OK
  } else {
    return false;
  }

  const unsigned short data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_PLANAR_CONFIG), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCompression(const unsigned short value) {
  unsigned int count = 1;

  if ((value == COMPRESSION_NONE)) {
    // OK
  } else {
    return false;
  }

  const unsigned short data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_COMPRESSION), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetSampleFormat(const unsigned int num_samples,
                               const unsigned short *values) {
  // `SetSamplesPerPixel()` must be called in advance
  if ((num_samples > 0) && (num_samples == samples_per_pixels_)) {
    // OK
  } else {
    err_ += "SetSamplesPerPixel() must be called before SetSampleFormat().\n";
    return false;
  }

  unsigned short format = values[0];

  std::vector<unsigned short> vs(num_samples);
  for (size_t i = 0; i < vs.size(); i++) {
    // FIXME(syoyo): Currently format must be same for all samples
    if (format != values[i]) {
      err_ += "SampleFormat must be same among samples at the moment.\n";
      return false;
    }

    if ((format == SAMPLEFORMAT_UINT) || (format == SAMPLEFORMAT_INT) ||
        (format == SAMPLEFORMAT_IEEEFP)) {
      // OK
    } else {
      err_ += "Invalid format value specified for SetSampleFormat().\n";
      return false;
    }

    vs[i] = values[i];

    // TODO(syoyo): Swap values when writing IFD tag, not here.
    if (swap_endian_) {
      swap2(&vs[i]);
    }
  }

  unsigned int count = num_samples;

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_SAMPLEFORMAT),
                          TIFF_SHORT, count,
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetOrientation(const unsigned short value) {
  unsigned int count = 1;

  if ((value == ORIENTATION_TOPLEFT) || (value == ORIENTATION_TOPRIGHT) ||
      (value == ORIENTATION_BOTRIGHT) || (value == ORIENTATION_BOTLEFT) ||
      (value == ORIENTATION_LEFTTOP) || (value == ORIENTATION_RIGHTTOP) ||
      (value == ORIENTATION_RIGHTBOT) || (value == ORIENTATION_LEFTBOT)) {
    // OK
  } else {
    return false;
  }

  const unsigned int data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_ORIENTATION), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetBlackLevel(const unsigned int num_components,
                             const unsigned short *values) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_BLACK_LEVEL), TIFF_SHORT, num_components,
      reinterpret_cast<const unsigned char *>(values), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetBlackLevelRational(unsigned int num_samples,
                                     const float *values) {
  // `SetSamplesPerPixel()` must be called in advance and SPP shoud be equal to
  // `num_samples`.
  if ((num_samples > 0) && (num_samples == samples_per_pixels_)) {
    // OK
  } else {
    return false;
  }

  std::vector<unsigned int> vs(num_samples * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }

  unsigned int count = num_samples;

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_BLACK_LEVEL),
                          TIFF_RATIONAL, count,
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetWhiteLevel(const short value) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_WHITE_LEVEL), TIFF_SHORT, 1,
      reinterpret_cast<const unsigned char *>(&value),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetWhiteLevelRational(unsigned int num_samples,
                                     const float *values) {
  // `SetSamplesPerPixel()` must be called in advance and SPP shoud be equal to
  // `num_samples`.
  if ((num_samples > 0) && (num_samples == samples_per_pixels_)) {
    // OK
  } else {
    return false;
  }

  std::vector<unsigned int> vs(num_samples * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }

  unsigned int count = num_samples;

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_WHITE_LEVEL),
                          TIFF_RATIONAL, count,
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetXResolution(const float value) {
  float numerator, denominator;
  if (FloatToRational(value, &numerator, &denominator) != 0) {
    // Couldn't represent fp value as integer rational value.
    return false;
  }

  unsigned int data[2];
  data[0] = static_cast<unsigned int>(numerator);
  data[1] = static_cast<unsigned int>(denominator);

  // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
  if (swap_endian_) {
    swap4(&data[0]);
    swap4(&data[1]);
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_XRESOLUTION), TIFF_RATIONAL, 1,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetYResolution(const float value) {
  float numerator, denominator;
  if (FloatToRational(value, &numerator, &denominator) != 0) {
    // Couldn't represent fp value as integer rational value.
    return false;
  }

  unsigned int data[2];
  data[0] = static_cast<unsigned int>(numerator);
  data[1] = static_cast<unsigned int>(denominator);

  // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
  if (swap_endian_) {
    swap4(&data[0]);
    swap4(&data[1]);
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_YRESOLUTION), TIFF_RATIONAL, 1,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetResolutionUnit(const unsigned short value) {
  unsigned int count = 1;

  if ((value == RESUNIT_NONE) || (value == RESUNIT_INCH) ||
      (value == RESUNIT_CENTIMETER)) {
    // OK
  } else {
    return false;
  }

  const unsigned short data = value;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_RESOLUTION_UNIT), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetFrameRate(float value) {
  float numerator, denominator;
  if (FloatToRational(value, &numerator, &denominator) != 0) {
    // Couldn't represent fp value as integer rational value.
    return false;
  }

  unsigned int data[2];
  data[0] = static_cast<unsigned int>(numerator);
  data[1] = static_cast<unsigned int>(denominator);

  bool ret = WriteTIFFTag(
     static_cast<unsigned short>(TIFFTAG_FPS), TIFF_RATIONAL, 1,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetTimeCode(unsigned char timecode[8]) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_TIMECODE), TIFF_BYTE, 8,
      reinterpret_cast<const unsigned char *>(timecode),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetExposureTime(float exposureSecs) {
  float numerator, denominator;
  if (FloatToRational(exposureSecs, &numerator, &denominator) != 0) {
    // Couldn't represent fp value as integer rational value.
    return false;
  }

  unsigned int data[2];
  data[0] = static_cast<unsigned int>(numerator);
  data[1] = static_cast<unsigned int>(denominator);

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CAMERA_EXPOSURE_TIME), TIFF_RATIONAL, 1,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetIso(unsigned short iso) {
  unsigned int count = 1;

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CAMERA_ISO), TIFF_SHORT, count,
      reinterpret_cast<const unsigned char *>(&iso), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetImageDescription(const std::string &ascii) {
  unsigned int count =
      static_cast<unsigned int>(ascii.length() + 1);  // +1 for '\0'

  if (count < 2) {
    // empty string
    return false;
  }

  if (count > (1024 * 1024)) {
    // too large
    return false;
  }

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_IMAGEDESCRIPTION),
                          TIFF_ASCII, count,
                          reinterpret_cast<const unsigned char *>(ascii.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetUniqueCameraModel(const std::string &ascii) {
  unsigned int count =
      static_cast<unsigned int>(ascii.length() + 1);  // +1 for '\0'

  if (count < 2) {
    // empty string
    return false;
  }

  if (count > (1024 * 1024)) {
    // too large
    return false;
  }

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_UNIQUE_CAMERA_MODEL),
                          TIFF_ASCII, count,
                          reinterpret_cast<const unsigned char *>(ascii.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetMake(const std::string &ascii) {
  unsigned int count =
      static_cast<unsigned int>(ascii.length() + 1);  // +1 for '\0'

  if (count < 2) {
    // empty string
    return false;
  }

  if (count > (1024 * 1024)) {
    // too large
    return false;
  }

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_MAKE),
                          TIFF_ASCII, count,
                          reinterpret_cast<const unsigned char *>(ascii.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCameraModelName(const std::string &ascii) {
  unsigned int count =
      static_cast<unsigned int>(ascii.length() + 1);  // +1 for '\0'

  if (count < 2) {
    // empty string
    return false;
  }

  if (count > (1024 * 1024)) {
    // too large
    return false;
  }

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_CAMERA_MODEL_NAME),
                          TIFF_ASCII, count,
                          reinterpret_cast<const unsigned char *>(ascii.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetSoftware(const std::string &ascii) {
  unsigned int count =
      static_cast<unsigned int>(ascii.length() + 1);  // +1 for '\0'

  if (count < 2) {
    // empty string
    return false;
  }

  if (count > 4096) {
    // too large
    return false;
  }

  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_SOFTWARE),
                          TIFF_ASCII, count,
                          reinterpret_cast<const unsigned char *>(ascii.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}


bool DNGImage::SetActiveArea(const unsigned int values[4]) {
  unsigned int count = 4;

  const unsigned int *data = values;
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_ACTIVE_AREA), TIFF_LONG, count,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetDNGVersion(const unsigned char a,
                             const unsigned char b,
                             const unsigned char c,
                             const unsigned char d) {
  unsigned char data[4] = {a, b, c, d};

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_DNG_VERSION), TIFF_BYTE, 4,
      reinterpret_cast<const unsigned char *>(data),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetDNGBackwardVersion(const unsigned char a,
                                     const unsigned char b,
                                     const unsigned char c,
                                     const unsigned char d) {
  unsigned char data[4] = {a, b, c, d};

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_DNG_BACKWARD_VERSION), TIFF_BYTE, 4,
      reinterpret_cast<const unsigned char *>(data),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetColorMatrix1(const unsigned int plane_count,
                               const float *matrix_values) {
  std::vector<int> vs(plane_count * 3 * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<int>(numerator);
    vs[2 * i + 1] = static_cast<int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_COLOR_MATRIX1),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetColorMatrix2(const unsigned int plane_count,
                               const float *matrix_values) {
  std::vector<int> vs(plane_count * 3 * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<int>(numerator);
    vs[2 * i + 1] = static_cast<int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_COLOR_MATRIX2),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetForwardMatrix1(const unsigned int plane_count,
                                 const float *matrix_values) {
  std::vector<int> vs(plane_count * 3 * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<int>(numerator);
    vs[2 * i + 1] = static_cast<int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_FORWARD_MATRIX1),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetForwardMatrix2(const unsigned int plane_count,
                                 const float *matrix_values) {
  std::vector<int> vs(plane_count * 3 * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<int>(numerator);
    vs[2 * i + 1] = static_cast<int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_FORWARD_MATRIX2),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCameraCalibration1(const unsigned int plane_count,
                                     const float *matrix_values) {
  std::vector<unsigned int> vs(plane_count * plane_count * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_CAMERA_CALIBRATION1),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCameraCalibration2(const unsigned int plane_count,
                                     const float *matrix_values) {
  std::vector<unsigned int> vs(plane_count * plane_count * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_CAMERA_CALIBRATION2),
                          TIFF_SRATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetAnalogBalance(const unsigned int plane_count,
                                const float *matrix_values) {
  std::vector<unsigned int> vs(plane_count * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_ANALOG_BALANCE),
                          TIFF_RATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCFARepeatPatternDim(const unsigned short width,
                                      const unsigned short height) {
  unsigned short data[2] = {width, height};

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CFA_REPEAT_PATTERN_DIM), TIFF_SHORT, 2,
      reinterpret_cast<const unsigned char *>(data),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetBlackLevelRepeatDim(const unsigned short width,
                                      const unsigned short height) {
  unsigned short data[2] = {width, height};

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_BLACK_LEVEL_REPEAT_DIM), TIFF_SHORT, 2,
      reinterpret_cast<const unsigned char *>(data),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCalibrationIlluminant1(const unsigned short value) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CALIBRATION_ILLUMINANT1), TIFF_SHORT, 1,
      reinterpret_cast<const unsigned char *>(&value),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCalibrationIlluminant2(const unsigned short value) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CALIBRATION_ILLUMINANT2), TIFF_SHORT, 1,
      reinterpret_cast<const unsigned char *>(&value),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCFAPattern(const unsigned int num_components,
                             const unsigned char *values) {
  if ((values == NULL) || (num_components < 1)) {
    return false;
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CFA_PATTERN), TIFF_BYTE, num_components,
      reinterpret_cast<const unsigned char *>(values),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCFALayout(const unsigned short value) {
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_CFA_LAYOUT), TIFF_SHORT, 1,
      reinterpret_cast<const unsigned char *>(&value),
      &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetAsShotNeutral(const unsigned int plane_count,
                                const float *matrix_values) {
  std::vector<unsigned int> vs(plane_count * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(matrix_values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_AS_SHOT_NEUTRAL),
                          TIFF_RATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetAsShotWhiteXY(const float x, const float y) {
  const float values[2] = {x, y};
  std::vector<unsigned int> vs(2 * 2);
  for (size_t i = 0; i * 2 < vs.size(); i++) {
    float numerator, denominator;
    if (FloatToRational(values[i], &numerator, &denominator) != 0) {
      // Couldn't represent fp value as integer rational value.
      return false;
    }

    vs[2 * i + 0] = static_cast<unsigned int>(numerator);
    vs[2 * i + 1] = static_cast<unsigned int>(denominator);

    // TODO(syoyo): Swap rational value(8 bytes) when writing IFD tag, not here.
    if (swap_endian_) {
      swap4(&vs[2 * i + 0]);
      swap4(&vs[2 * i + 1]);
    }
  }
  bool ret = WriteTIFFTag(static_cast<unsigned short>(TIFFTAG_AS_SHOT_WHITE_XY),
                          TIFF_RATIONAL, uint32_t(vs.size() / 2),
                          reinterpret_cast<const unsigned char *>(vs.data()),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetBaselineExposure(float value) {
  float numerator, denominator;
  if (FloatToRational(value, &numerator, &denominator) != 0) {
    // Couldn't represent fp value as integer rational value.
    return false;
  }

  int data[2];
  data[0] = static_cast<int>(numerator);
  data[1] = static_cast<int>(denominator);

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_BASELINE_EXPOSURE), TIFF_SRATIONAL, 1,
      reinterpret_cast<const unsigned char *>(data), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetNoiseProfile(const double values[6]) {
  std::vector<double> vs(6);
  for (size_t i = 0; i < 6; i++) {
    vs[i] = values[i];
    if (swap_endian_) {
      swap8(reinterpret_cast<uint64_t*>(&vs[i]));
    }
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_NOISE_PROFILE), TIFF_DOUBLE, 6,
      reinterpret_cast<const unsigned char *>(vs.data()), &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetLinearizationTable(const unsigned int table_size, const unsigned short *table_values) {
  if ((table_values == NULL) || (table_size < 1)) {
    err_ += "Invalid linearization table parameters.\n";
    return false;
  }

  // Validate table size - common sizes are 256, 1024, 4096, 16384, 65536
  if (table_size > 65536) {
    err_ += "Linearization table size too large (max 65536 entries).\n";
    return false;
  }

  // Create a copy of the table values for endian swapping if needed
  std::vector<unsigned short> vs(table_size);
  for (size_t i = 0; i < table_size; i++) {
    vs[i] = table_values[i];
    
    // Swap endian if needed
    if (swap_endian_) {
      swap2(&vs[i]);
    }
  }

  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_LINEARIZATION_TABLE), TIFF_SHORT, table_size,
      reinterpret_cast<const unsigned char *>(vs.data()), &ifd_tags_, &data_os_);

  if (!ret) {
    err_ += "Failed to write linearization table tag.\n";
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetImageDataPacked(const unsigned short *input_buffer, const int input_count, const unsigned int input_bpp, bool big_endian)
{
  if (input_count <= 0) {
    return false;
  }

  if (input_bpp > 16)
    return false;
  
  unsigned int bits_free = 16 - input_bpp;
  const unsigned short *unpacked_bits = input_buffer;

  std::vector<unsigned short> output(static_cast<size_t>(input_count));
  unsigned short *packed_bits = output.data();

  packed_bits[0] = static_cast<unsigned short>(unpacked_bits[0] << bits_free);
  for (unsigned int pixel_index = 1; pixel_index < static_cast<unsigned int>(input_count); pixel_index++)
  {
    unsigned int bits_offset = (pixel_index * bits_free) % 16;
    unsigned int bits_to_rol = bits_free + bits_offset + (bits_offset > 0) * 16;
    
    unsigned int data = ROL32(static_cast<unsigned int>(unpacked_bits[pixel_index]), bits_to_rol);
    *(reinterpret_cast<unsigned int *>(packed_bits)) = (*(reinterpret_cast<unsigned int *>(packed_bits)) & 0x0000FFFF) | data;

    if(bits_offset > 0 && bits_offset <= input_bpp)
    {
      if(big_endian)
        *(reinterpret_cast<unsigned short *>(packed_bits)) = static_cast<unsigned short>(ROL16(*(reinterpret_cast<unsigned short *>(packed_bits)), 8));

      ++packed_bits;
    }
  }

  return SetImageData(reinterpret_cast<unsigned char*>(output.data()), output.size() * sizeof(unsigned short));
}

bool DNGImage::SetImageData(const unsigned char *data, const size_t data_len) {
  if ((data == NULL) || (data_len < 1)) {
    return false;
  }

  data_strip_offset_ = size_t(data_os_.tellp());
  data_strip_bytes_ = data_len;

  data_os_.write(reinterpret_cast<const char *>(data),
                 static_cast<std::streamsize>(data_len));

  // NOTE: STRIP_OFFSET tag will be written at `WriteIFDToStream()`.

  {
    unsigned int count = 1;
    unsigned int bytes = static_cast<unsigned int>(data_len);

    bool ret = WriteTIFFTag(
        static_cast<unsigned short>(TIFFTAG_STRIP_BYTE_COUNTS), TIFF_LONG,
        count, reinterpret_cast<const unsigned char *>(&bytes), &ifd_tags_,
        NULL);

    if (!ret) {
      return false;
    }

    num_fields_++;
  }

  return true;
}

bool DNGImage::SetCustomFieldLong(const unsigned short tag, const int value) {
  unsigned int count = 1;

  // TODO(syoyo): Check if `tag` value does not conflict with existing TIFF tag
  // value.

  bool ret = WriteTIFFTag(tag, TIFF_SLONG, count,
                          reinterpret_cast<const unsigned char *>(&value),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetCustomFieldULong(const unsigned short tag,
                                   const unsigned int value) {
  unsigned int count = 1;

  // TODO(syoyo): Check if `tag` value does not conflict with existing TIFF tag
  // value.

  bool ret = WriteTIFFTag(tag, TIFF_LONG, count,
                          reinterpret_cast<const unsigned char *>(&value),
                          &ifd_tags_, &data_os_);

  if (!ret) {
    return false;
  }

  num_fields_++;
  return true;
}

static bool IFDComparator(const IFDTag &a, const IFDTag &b) {
  return (a.tag < b.tag);
}

bool DNGImage::WriteDataToStream(std::ostream *ofs) const {
  if ((data_os_.str().length() == 0)) {
    err_ += "Empty IFD data and image data.\n";
    return false;
  }

  if (bits_per_samples_.empty()) {
    err_ += "BitsPerSample is not set\n";
    return false;
  }

  for (size_t i = 0; i < bits_per_samples_.size(); i++) {
    if (bits_per_samples_[i] == 0) {
      err_ += std::to_string(i) + "'th BitsPerSample is zero";
      return false;
    }
  }

  if (samples_per_pixels_ == 0) {
    err_ += "SamplesPerPixels is not set or zero.";
    return false;
  }

  std::vector<uint8_t> data(data_os_.str().length());
  memcpy(data.data(), data_os_.str().data(), data.size());

  if (data_strip_bytes_ == 0) {
    // May ok?.
  } else {
    // FIXME(syoyo): Assume all channels use sample bps
    uint32_t bps = bits_per_samples_[0];

    // We may need to swap endian for pixel data.
    if (swap_endian_) {
      if (bps == 16) {
        size_t n = data_strip_bytes_ / sizeof(uint16_t);
        uint16_t *ptr =
            reinterpret_cast<uint16_t *>(data.data() + data_strip_offset_);

        for (size_t i = 0; i < n; i++) {
          swap2(&ptr[i]);
        }

      } else if (bps == 32) {
        size_t n = data_strip_bytes_ / sizeof(uint32_t);
        uint32_t *ptr =
            reinterpret_cast<uint32_t *>(data.data() + data_strip_offset_);

        for (size_t i = 0; i < n; i++) {
          swap4(&ptr[i]);
        }

      } else if (bps == 64) {
        size_t n = data_strip_bytes_ / sizeof(uint64_t);
        uint64_t *ptr =
            reinterpret_cast<uint64_t *>(data.data() + data_strip_offset_);

        for (size_t i = 0; i < n; i++) {
          swap8(&ptr[i]);
        }
      }
    }
  }

  ofs->write(reinterpret_cast<const char *>(data.data()),
             static_cast<std::streamsize>(data.size()));

  return true;
}

bool DNGImage::WriteIFDToStream(const unsigned int data_base_offset,
                                const unsigned int strip_offset,
                                std::ostream *ofs) const {
  if ((num_fields_ == 0) || (ifd_tags_.size() < 1)) {
    err_ += "No TIFF Tags.\n";
    return false;
  }

  // add STRIP_OFFSET tag and sort IFD tags.
  std::vector<IFDTag> tags = ifd_tags_;
  {
    // For STRIP_OFFSET we need the actual offset value to data(image),
    // thus write STRIP_OFFSET here.
    unsigned int offset = strip_offset + kHeaderSize;
    IFDTag ifd;
    ifd.tag = TIFFTAG_STRIP_OFFSET;
    ifd.type = TIFF_LONG;
    ifd.count = 1;
    ifd.offset_or_value = offset;
    tags.push_back(ifd);
  }

  // TIFF expects IFD tags are sorted.
  std::sort(tags.begin(), tags.end(), IFDComparator);

  std::ostringstream ifd_os;

  unsigned short num_fields = static_cast<unsigned short>(tags.size());

  Write2(num_fields, &ifd_os, swap_endian_);

  {
    size_t typesize_table[] = {1, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8, 4};

    for (size_t i = 0; i < tags.size(); i++) {
      const IFDTag &ifd = tags[i];
      Write2(ifd.tag, &ifd_os, swap_endian_);
      Write2(ifd.type, &ifd_os, swap_endian_);
      Write4(ifd.count, &ifd_os, swap_endian_);

      size_t len =
          ifd.count * (typesize_table[(ifd.type) < 14 ? (ifd.type) : 0]);
      if (len > 4) {
        // Store offset value.
        unsigned int ifd_offt = ifd.offset_or_value + data_base_offset;
        Write4(ifd_offt, &ifd_os, swap_endian_);
      } else {
        // less than 4 bytes = store data itself.

        if (len == 1) {
          const unsigned char value =
              *(reinterpret_cast<const unsigned char *>(&ifd.offset_or_value));
          Write1(value, &ifd_os);
          unsigned char pad = 0;
          Write1(pad, &ifd_os);
          Write1(pad, &ifd_os);
          Write1(pad, &ifd_os);
        } else if (len == 2) {
          const unsigned short value =
              *(reinterpret_cast<const unsigned short *>(&ifd.offset_or_value));
          Write2(value, &ifd_os, swap_endian_);
          const unsigned short pad = 0;
          Write2(pad, &ifd_os, swap_endian_);
        } else if (len == 4) {
          const unsigned int value =
              *(reinterpret_cast<const unsigned int *>(&ifd.offset_or_value));
          Write4(value, &ifd_os, swap_endian_);
        } else {
          assert(0);
        }
      }
    }

    ofs->write(ifd_os.str().c_str(),
               static_cast<std::streamsize>(ifd_os.str().length()));
  }

  return true;
}

// -------------------------------------------

DNGWriter::DNGWriter(bool big_endian) : dng_big_endian_(big_endian) {
  swap_endian_ = (IsBigEndian() != dng_big_endian_);
}

bool DNGWriter::WriteToFile(const char *filename, std::string *err) const {
  std::ofstream ofs(filename, std::ostream::binary);

  if (!ofs) {
    if (err) {
      (*err) = "Failed to open file.\n";
    }

    return false;
  }
  
  return WriteToFile(ofs, err);
}

bool DNGWriter::WriteToFile(int fd, std::string *err) const {
  boost::fdostream ofs(fd);

  if (!ofs) {
    if (err) {
      (*err) = "Failed to open file.\n";
    }

    return false;
  }
  
  return WriteToFile(ofs, err);
}

bool DNGWriter::WriteToFile(std::ostream& ofs, std::string *err) const {
  std::ostringstream header;
  bool ret = WriteTIFFVersionHeader(&header, dng_big_endian_);
  if (!ret) {
    if (err) {
      (*err) = "Failed to write TIFF version header.\n";
    }
    return false;
  }

  if (images_.size() == 0) {
    if (err) {
      (*err) = "No image added for writing.\n";
    }

    return false;
  }

  // 1. Compute offset and data size(exclude TIFF header bytes)
  size_t data_len = 0;
  size_t strip_offset = 0;
  std::vector<size_t> data_offset_table;
  std::vector<size_t> strip_offset_table;
  for (size_t i = 0; i < images_.size(); i++) {
    strip_offset = data_len + images_[i]->GetStripOffset();
    data_offset_table.push_back(data_len);
    strip_offset_table.push_back(strip_offset);
    data_len += images_[i]->GetDataSize();
  }

  // 2. Write offset to ifd table.
  const unsigned int ifd_offset =
      kHeaderSize + static_cast<unsigned int>(data_len);
  Write4(ifd_offset, &header, swap_endian_);

  assert(header.str().length() == 8);

  // std::cout << "ifd_offset " << ifd_offset << std::endl;
  // std::cout << "data_len " << data_os_.str().length() << std::endl;
  // std::cout << "ifd_len " << ifd_os_.str().length() << std::endl;
  // std::cout << "swap endian " << swap_endian_ << std::endl;

  // 3. Write header
  ofs.write(header.str().c_str(),
            static_cast<std::streamsize>(header.str().length()));

  // 4. Write image and meta data
  // TODO(syoyo): Write IFD first, then image/meta data
  for (size_t i = 0; i < images_.size(); i++) {
    bool ok = images_[i]->WriteDataToStream(&ofs);
    if (!ok) {
      if (err) {
        std::stringstream ss;
        ss << "Failed to write data at image[" << i << "]. err = " << images_[i]->Error() << "\n";
        (*err) += ss.str();
      }
      return false;
    }
  }

  // 5. Write IFD entries;
  for (size_t i = 0; i < images_.size(); i++) {
    bool ok = images_[i]->WriteIFDToStream(
        static_cast<unsigned int>(data_offset_table[i]),
        static_cast<unsigned int>(strip_offset_table[i]), &ofs);
    if (!ok) {
      if (err) {
        std::stringstream ss;
        ss << "Failed to write IFD at image[" << i << "]. err = " << images_[i]->Error() << "\n";
        (*err) += ss.str();
      }
      return false;
    }

    unsigned int next_ifd_offset =
        static_cast<unsigned int>(ofs.tellp()) + sizeof(unsigned int);

    if (i == (images_.size() - 1)) {
      // Write zero as IFD offset(= end of data)
      next_ifd_offset = 0;
    }

    if (swap_endian_) {
      swap4(&next_ifd_offset);
    }

    ofs.write(reinterpret_cast<const char *>(&next_ifd_offset), 4);
  }

  return true;
}

// Opcode List implementations

void OpcodeList::AddWarpRectilinear(const WarpRectilinearParams& params) {
  Opcode opcode;
  opcode.id = OPCODE_WARP_RECTILINEAR;
  opcode.version[0] = 1; opcode.version[1] = 3; opcode.version[2] = 0; opcode.version[3] = 0;
  opcode.flags = 0;
  
  std::ostringstream data_stream;
  
  // Write parameters (big-endian as per DNG spec for opcode blocks)
  Write4(params.num_coeff_sets, &data_stream, true);
  
  for (size_t i = 0; i < params.num_coeff_sets; i++) {
    // Write radial coefficients (kr0, kr1, kr2, kr3)
    double kr0 = i < params.kr0.size() ? params.kr0[i] : 1.0;
    double kr1 = i < params.kr1.size() ? params.kr1[i] : 0.0;
    double kr2 = i < params.kr2.size() ? params.kr2[i] : 0.0;
    double kr3 = i < params.kr3.size() ? params.kr3[i] : 0.0;
    
    // Write doubles in big-endian
    WriteDouble(kr0, &data_stream, true);
    WriteDouble(kr1, &data_stream, true);
    WriteDouble(kr2, &data_stream, true);
    WriteDouble(kr3, &data_stream, true);
    
    // Write tangential coefficients (kt0, kt1)
    double kt0 = i < params.kt0.size() ? params.kt0[i] : 0.0;
    double kt1 = i < params.kt1.size() ? params.kt1[i] : 0.0;
    
    WriteDouble(kt0, &data_stream, true);
    WriteDouble(kt1, &data_stream, true);
  }
  
  // Write optical center
  WriteDouble(params.cx_hat, &data_stream, true);
  WriteDouble(params.cy_hat, &data_stream, true);
  
  std::string data_str = data_stream.str();
  opcode.data.assign(data_str.begin(), data_str.end());
  
  opcodes_.push_back(opcode);
}

void OpcodeList::AddFixBadPixelsList(const FixBadPixelsParams& params) {
  Opcode opcode;
  opcode.id = OPCODE_FIX_BAD_PIXELS_LIST;
  opcode.version[0] = 1; opcode.version[1] = 3; opcode.version[2] = 0; opcode.version[3] = 0;
  opcode.flags = 0;
  
  std::ostringstream data_stream;
  
  // Write parameters (big-endian)
  Write4(params.bayer_phase, &data_stream, true);
  Write4(static_cast<unsigned int>(params.bad_pixels.size()), &data_stream, true);
  Write4(static_cast<unsigned int>(params.bad_rects.size()), &data_stream, true);
  
  // Write bad pixels
  for (const auto& pixel : params.bad_pixels) {
    Write4(pixel.row, &data_stream, true);
    Write4(pixel.column, &data_stream, true);
  }
  
  // Write bad rectangles
  for (const auto& rect : params.bad_rects) {
    Write4(rect.top, &data_stream, true);
    Write4(rect.left, &data_stream, true);
    Write4(rect.bottom, &data_stream, true);
    Write4(rect.right, &data_stream, true);
  }
  
  std::string data_str = data_stream.str();
  opcode.data.assign(data_str.begin(), data_str.end());
  
  opcodes_.push_back(opcode);
}

void OpcodeList::AddGainMap(const GainMapParams& params) {
  Opcode opcode;
  opcode.id = OPCODE_GAIN_MAP;
  // DNG version 1.3.0.0
  opcode.version[0] = 1; opcode.version[1] = 3; opcode.version[2] = 0; opcode.version[3] = 0;
  opcode.flags = 0;
  
  std::ostringstream data_stream;
  
  // Write area of interest (LONG: top, left, bottom, right)
  Write4(params.top, &data_stream, true);
  Write4(params.left, &data_stream, true);
  Write4(params.bottom, &data_stream, true);
  Write4(params.right, &data_stream, true);
  
  // Write plane info (LONG: plane, planes)
  Write4(params.plane, &data_stream, true);
  Write4(params.planes, &data_stream, true);
  
  // Write pitch (LONG: row_pitch, col_pitch)
  Write4(params.row_pitch, &data_stream, true);
  Write4(params.col_pitch, &data_stream, true);
  
  // Write map dimensions (LONG: map_points_v, map_points_h)
  Write4(params.map_points_v, &data_stream, true);
  Write4(params.map_points_h, &data_stream, true);
  
  // Write spacing (DOUBLE: map_spacing_v, map_spacing_h)
  WriteDouble(params.map_spacing_v, &data_stream, true);
  WriteDouble(params.map_spacing_h, &data_stream, true);
  
  // Write origin (DOUBLE: map_origin_v, map_origin_h)
  WriteDouble(params.map_origin_v, &data_stream, true);
  WriteDouble(params.map_origin_h, &data_stream, true);
  
  // Write map planes (LONG: map_planes)
  Write4(params.map_planes, &data_stream, true);
  
  // Write gain map data (FLOAT array)
  // For each MapPointsV, for each MapPointsH, for each MapPlanes: MapGain (FLOAT)
  size_t expected_size = params.map_points_v * params.map_points_h * params.map_planes;
  for (size_t i = 0; i < params.gain_data.size() && i < expected_size; i++) {
    WriteFloat(params.gain_data[i], &data_stream, true);
  }
  
  std::string data_str = data_stream.str();
  opcode.data.assign(data_str.begin(), data_str.end());
  
  opcodes_.push_back(opcode);
}

std::vector<unsigned char> OpcodeList::Serialize() const {
  if (opcodes_.empty()) {
    return std::vector<unsigned char>();
  }
  
  std::ostringstream stream;
  
  // Write opcode count (big-endian, 32-bit uint)
  Write4(static_cast<unsigned int>(opcodes_.size()), &stream, true);
  
  for (const auto& opcode : opcodes_) {
    // Write opcode ID (32-bit uint)
    Write4(opcode.id, &stream, true);
    
    // Write DNG version as single 32-bit value (e.g., 0x01030000 for version 1.3.0.0)
    unsigned int dng_version = (opcode.version[0] << 24) | (opcode.version[1] << 16) | 
                               (opcode.version[2] << 8) | opcode.version[3];
    Write4(dng_version, &stream, true);
    
    // Write flags (32-bit uint)
    Write4(opcode.flags, &stream, true);
    
    // Write data size (32-bit uint)
    Write4(static_cast<unsigned int>(opcode.data.size()), &stream, true);
    
    // Write opcode data
    stream.write(reinterpret_cast<const char*>(opcode.data.data()), opcode.data.size());
  }
  
  std::string data_str = stream.str();
  return std::vector<unsigned char>(data_str.begin(), data_str.end());
}

// DNGImage opcode list methods

bool DNGImage::SetOpcodeList1(const OpcodeList& opcode_list) {
  if (opcode_list.IsEmpty()) {
    return true; // Empty opcode list is valid
  }
  
  std::vector<unsigned char> data = opcode_list.Serialize();
  
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_OPCODE_LIST1), TIFF_UNDEFINED, 
      static_cast<unsigned int>(data.size()),
      data.data(), &ifd_tags_, &data_os_);

  if (!ret) {
    err_ += "Failed to write OpcodeList1 tag.\n";
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetOpcodeList2(const OpcodeList& opcode_list) {
  if (opcode_list.IsEmpty()) {
    return true; // Empty opcode list is valid
  }
  
  std::vector<unsigned char> data = opcode_list.Serialize();
  
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_OPCODE_LIST2), TIFF_UNDEFINED, 
      static_cast<unsigned int>(data.size()),
      data.data(), &ifd_tags_, &data_os_);

  if (!ret) {
    err_ += "Failed to write OpcodeList2 tag.\n";
    return false;
  }

  num_fields_++;
  return true;
}

bool DNGImage::SetOpcodeList3(const OpcodeList& opcode_list) {
  if (opcode_list.IsEmpty()) {
    return true; // Empty opcode list is valid
  }
  
  std::vector<unsigned char> data = opcode_list.Serialize();
  
  bool ret = WriteTIFFTag(
      static_cast<unsigned short>(TIFFTAG_OPCODE_LIST3), TIFF_UNDEFINED, 
      static_cast<unsigned int>(data.size()),
      data.data(), &ifd_tags_, &data_os_);

  if (!ret) {
    err_ += "Failed to write OpcodeList3 tag.\n";
    return false;
  }

  num_fields_++;
  return true;
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif

}  // namespace tinydngwriter

#endif  // TINY_DNG_WRITER_IMPLEMENTATION
