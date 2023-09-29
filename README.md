Foobar2000 HackRF Transmitter Plugin
========================================

This is a fork of [shenglin00](https://github.com/shenglin00/foo_hackrf)'s version who forked the [original](https://github.com/jocover/foo_hackrf) one.
I just updated it to the lastest Foobar2k SDK, fixed the stereo WBFM and allowed you to change settings while running.

##### Installing

You have to download the dll from the [releases tab](https://github.com/djpadbit/foo_hackrf/releases). Then put it in a foo_hackrf folder in your user-components.
You'll also need to have libhackrf compiled on Windows, i'll leave it up to you to figure it out (it's kinda of a pain). in the end you should have libusb-1.0.dll, 
pthreadVC2.dll, hackrf.dll and foo_hackrf.dll in that folder.

#### Building

This is based on the foo_sample example component, so it compiles the same way.
It was made for the Foobar2k SDK2023-09-13. I built it using Visual Studio 2022

You need libhackrf's header and library file in the libhackrf/ folder in the root of the project.
I compiled it myself but you might be able to get the library somewhere.