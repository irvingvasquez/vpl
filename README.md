# View Planning Library

Partial Model library provides a set of functions to handle visibility for next best view calculation. This library implements hierarchical and uniform ray tracing. Also this library implements the incorporation of sensor readings. PartialModelLib was developed by [J. Irving Vasquez-Gomez] under New BSD licence. This Library is based on the octomap library.
    
> CONACyT

> Instituto Nacional de Astrofísica Óptica y Electrónica.

> Centro de Innovación y Desarrollo Tecnológico 

### Requirements

Before installing Partial Model Library you need to install the following libraries:
- octomap
- iniparser
- boost


### Installation

First, download and install [octomap] and [iniparser]. Modify the CMakeList.txt file to specify your installation addresses or follow the folder hierarchy that I have used. See "Full Next Best View Installation". 


### Full Next Best View installation

we recommend the following folder hierarchy:

- ViewPlanning
    - octomap-devel
    - partialmodel
    - iniparser

To compile the library move to the partial model top folder and run:

```sh
cd partialmodel
mkdir build
cd build    
cmake ..
```
`Note:if you are using your custom folder hierarchy you should modify the CMakeLists.txt in order to match the folders`



   [octomap]: <https://octomap.github.io/>
   [iniparser]: <https://github.com/ndevilla/iniparser>
   [J. Irving Vasquez-Gomez]: <https://jivasquez.wordpress.com>

