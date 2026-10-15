# MotionCam MCRAW decoder

A simple library for decoding files recorded by [MotionCam Pro](https://www.motioncamapp.com/).

## Usage

Look in `example.cpp` for a simple example on how to extract the RAW frames into DNGs and the audio into a WAV file.

To build the example:

```
mkdir build

cd build

cmake ..

make
```

To extract the first frame and audio from a `.mcraw` file run:

`./example <path to mcraw file> -n 1`

### Encoded-frame callback

The four-argument `loadFrame()` overload lets an application inspect a frame's
compressed payload and original JSON before parsing the JSON or decoding pixels:

```cpp
motioncam::EncodedFrameCallback inspect = [](motioncam::Timestamp timestamp,
    const uint8_t* payload, size_t payloadSize,
    const uint8_t* json, size_t jsonSize) {
    // Inspect the original bytes, collect statistics, or apply application policy.
    // Return false to cancel this load; true continues decoding.
    return true;
};
decoder.loadFrame(timestamp, pixels, metadata, inspect);
```

The callback receives the frame timestamp, compressed bytes, and exact on-disk
frame JSON bytes (not parsed or reserialized JSON). It runs once per load, and
the decoder consumes the same payload buffer afterward, with no second payload
read or copy. Views are valid only during the callback and must not be retained
or modified. Keep the file unchanged while it is being read.

An empty callback or `false` result throws `motioncam::IOException` before JSON
parsing or pixel decoding. Callback exceptions propagate. Cancellation leaves
output arguments unchanged; handle the exception before using outputs, which
may still contain an earlier frame. Decoder instances are not safe for concurrent
operations. The existing three-argument `loadFrame()` and metadata-only APIs
retain their behavior and do not invoke a callback. No extra dependency is needed.

### Embedded gyro data

Recent `.mcraw` files can contain raw gyroscope samples. The timestamps use the
same nanosecond timeline as frame timestamps, and the axis values are radians per
second:

```cpp
motioncam::Decoder decoder("input.mcraw");
std::vector<motioncam::MotionSample> gyroSamples;

if(decoder.hasGyroData())
    decoder.loadGyroData(gyroSamples);
```

### Embedded accelerometer data

New `.mcraw` files can also contain accelerometer samples. They use the same
nanosecond timeline as frames and gyro samples. Values are **m/s² including
gravity**, retaining the source platform's sensor axes and sign convention.
MotionCam already converts iOS values from g when recording; do not convert them
again when decoding. The decoder preserves sample timestamps and values without
resampling or rotating them into image coordinates.

```cpp
motioncam::Decoder decoder("input.mcraw");
std::vector<motioncam::MotionSample> accelerometerSamples;

if(decoder.hasAccelerometerData())
    decoder.loadAccelerometerData(accelerometerSamples);
```

Like `loadGyroData`, `loadAccelerometerData` appends to the supplied vector. Files
without accelerometer data leave the vector unchanged. OIS chunks are safely
skipped so they do not prevent discovery of the accelerometer index; OIS samples
are not exposed by this API.

The stream uses item IDs 12 (index) and 13 (data), with version-1 headers and the
existing 24-byte `MotionSample` layout. The container version remains 3.

To run the decoder regression tests:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```


## Sample Files

You can download a sample file from [here](https://storage.googleapis.com/motioncamapp.com/samples/007-VIDEO_24mm-240328_141729.0.mcraw).

## MotionCam Pro

MotionCam Pro is an app for Android that provides the ability to record RAW videos. Get it from the [Play Store](https://play.google.com/store/apps/details?id=com.motioncam.pro&hl=en&gl=US).
