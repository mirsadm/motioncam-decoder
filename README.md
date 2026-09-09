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
