# View Planning Library

> CONACyT

> Instituto Nacional de Astrofísica Óptica y Electrónica.

> Centro de Innovación y Desarrollo Tecnológico 

VPL(The acronym of view planning library) provides a platform to develop view planning algorithms and perform comparisons quickly. VPL is written in C++ and it is based on a set of well known libraries: octomap and pcl. VPL provides the data structures to represent the space, provides visibility algorithms, implements several view planning algorithms reported in the literature and provides flexibility to link with range sensor simulators and motion planning algorithms. VPL was developed by [J. Irving Vasquez-Gomez] under New BSD licence.

VPL is composed of 2 core libraries: PartialModel and ViewPlanning. PartialModel stores the information about object that is being reconstructed and provides a set of functions to handle visibility for next best view calculation. 
ViewPlanning provides planning algorithms to achieve an automated reconstruction.


### Requirements

Before installing VPL you need to install the following libraries:
- octomap
- iniparser
- boost


### Installation

First, download and install [octomap] and [iniparser]. Modify the CMakeList.txt file to specify your installation addresses or follow the folder hierarchy that I have used. See "Full VPL Installation". 


### Full VPL installation

we recommend that VPL will be installed in the same folder that the required libraries:

- octomap-devel
- iniparser
- VPL

To compile the library move to the VPL top folder and run:

```sh
cd VPL
mkdir build
cd build    
cmake ..
```
`Note:if you are using your custom folder hierarchy you should modify the CMakeLists.txt in order to match the folders`



   [octomap]: <https://octomap.github.io/>
   [iniparser]: <https://github.com/ndevilla/iniparser>
   [J. Irving Vasquez-Gomez]: <https://jivasquez.wordpress.com>

