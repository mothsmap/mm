#mm
---
mm is a __map matching__ algorithm.

##PREREQUISITES
---
mm need following libraries:

__boost__(several libraries) : http://www.boost.org/

__gdal__: http://www.gdal.org/

You need to either install the packages for your distribution or install those
libraries from source. All libraries should be available in all distributions.

##FILES
---

- __3rd/__          
 - Third party libraries.


- __CMakeLists.txt__

 - CMake file.

- __data/__            
 - Data folder.

- __map/__
 - Simple server to demo match result.

- __papers/__
 - Reference papers.

- __README__
 - the file you are reading.

- __src/__             
 - Source file.

- __test/__
  - Tests.

##BUILDING
---

$ git clone https://github.com/mothsmap/mm.git

$ cd mm

$ mkdir build && cd build

$ cmake ..

$ make

##USAGE

After building, you can get two executable, called __mm_prepare__ and __mm_match__.

First, use __mm_prepare__ to generate graph, rtree, and history trace.

Then, use __mm_match__ to match a GPS log.

The result can be showed in the simple map service.

##LICENSE

MIT LISCENSE.

