MuJoCo 3.10.0 WASM — third-party attribution supplement

The bundled, unmodified @mujoco/mujoco 3.10.0 JavaScript/WASM runtime is built
upstream with third-party components. The following notices accompany it.
This list preserves source-level license information; it is not a new license.

Build and exact dependency evidence:
https://github.com/google-deepmind/mujoco/blob/3.10.0/wasm/CMakeLists.txt
https://github.com/google-deepmind/mujoco/blob/3.10.0/cmake/MujocoDependencies.cmake
https://github.com/google-deepmind/mujoco/blob/3.10.0/.github/workflows/build_steps.sh

LodePNG
Copyright (c) 2005-2025 Lode Vandevenne; zlib-style license.
Source: https://github.com/lvandeve/lodepng/tree/17d08dd26cac4d63f43af217ebd70318bfb8189c
Complete license notice: lodepng-LICENSE.txt.

TinyXML-2
Original code by Lee Thomason (www.grinninglizard.com); zlib-style license.
Source: https://github.com/leethomason/tinyxml2/tree/e6caeae85799003f4ca74ff26ee16a789bc2af48
License: tinyxml2-LICENSE.txt.

tinyobjloader
Copyright (c) 2012-2019 Syoyo Fujita and many contributors; MIT.
Source: https://github.com/tinyobjloader/tinyobjloader/tree/1421a10d6ed9742f5b2c1766d22faa6cfbc56248
License: tinyobjloader-LICENSE.txt.

libccd
Copyright (c) 2010-2012 Daniel Fiser and additional rights holders identified in
libccd-BSD-LICENSE.txt; BSD terms in that file.
Source: https://github.com/danfis/libccd/tree/7931e764a19ef6b21b443376c699bbc9c6d4fba8
The MuJoCo build applies an Emscripten support patch to libccd's CMake file.
Patch: https://github.com/google-deepmind/mujoco/blob/3.10.0/cmake/ccd-support-emscripten.patch

Qhull
Copyright (c) 1993-2020 C.B. Barber and The Geometry Center, University of Minnesota.
Source code may be obtained from http://www.qhull.org and
https://github.com/qhull/qhull/tree/d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6
Required license/copyright notice: qhull-COPYING.txt.
MuJoCo's build modifies the upstream CMake file to avoid unsupported shared-library
targets with Emscripten. Modification introduced 2025-10-31 by Matias Manevi, with
co-authors Matija Kecman, Sebastian Noreña Rendón and Kyle Bayes:
https://github.com/google-deepmind/mujoco/commit/4086261714d7cfbc1745d4c6cb0aa2116df45312
Exact modification: https://github.com/google-deepmind/mujoco/blob/3.10.0/cmake/qhull-support-emscripten.patch
This project does not make further modifications to the distributed Qhull/MuJoCo binary.

miniz
Copyright 2013-2014 RAD Game Tools and Valve Software; Copyright 2010-2014
Rich Geldreich and Tenacious Software LLC; terms in miniz-LICENSE.txt.
Source: https://github.com/richgel999/miniz/tree/d10b03cc73475af673df40f06e5cefd1d5f940d9

MarchingCubeCpp
Source repository owner: aparis69.
The upstream README offers public domain or MIT, at the user's choice. This release
relies on the public-domain option and preserves that declaration in
MarchingCubeCpp-README.txt.
Source: https://github.com/aparis69/MarchingCubeCpp/tree/f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d

Emscripten 4.0.10 runtime and system libraries
MuJoCo's official build_steps.sh pins Emscripten 4.0.10.
Source: https://github.com/emscripten-core/emscripten/tree/4.0.10
Runtime: Emscripten-LICENSE.txt (MIT or University of Illinois/NCSA, including
the retained Node.js notice).
C library: Emscripten-musl-COPYRIGHT.txt.
C++ libraries: Emscripten-libcxx-LICENSE.txt and Emscripten-libcxxabi-LICENSE.txt.
Compiler runtime: Emscripten-compiler-rt-LICENSE.txt.

