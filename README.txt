This is 971's cuda apriltag library extracted into a standalone application (jni ripped out in this fork)

`mkdir build && cd build && cmake .. && cmake --build .` should be enough to build it? Needs a proper cmake cuda setup obviously.

Two binaries are produced: main and camera. The former takes in images while the latter takes in a camera stream.
They're pretty crude and don't take any arguments (except for main taking in the image file as its only argument), so edit source code to change settings.
