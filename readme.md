# KOBAYAXI

> This repo's current purpose is to make bgfx(cross-platform 3D library) build as thirdparty dependency.

With KOBAYAXI you can setup a 3D visualize environment quite easy, it prevents you from writing BORING infrastructural codes like 
open a window, initialize OpenGL/DX or any low level drawing api functions to merely `draw a triangle on the screen`. Just free
your hand to write codes you(or your boss) really care about.   :)

## How To Build
``` bash
$ git clone --recursive https://github.com/CallmeNezha/KOBAYAXI.git
$ cd KOBAYAXI
$ mkdir build && cd build
$ cmake ..
$ make
```
