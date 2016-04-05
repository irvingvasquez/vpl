# Partial Model Library

Partial Model library provides a set of functions to handle visibility into the next best view calculation. This library implements hierarchical and uniform ray tracing. Also this library implements the incorporation of sensor readings. PartialModelLib was developed by [J. Irving Vasquez-Gomez] under New BSD licence.

> Laboratorio de Robótica. Instituto Nacional de Astrofísica Óptica y Electrónica

### Requirements

Before installing partialmodellib you need to install the following libraries:
- octomap
- iniparser
- boost

### Installation

First, download and install [octomap] and [iniparser]. Modify the CMakeList.txt file to specify your installation addresses or follow the folder hierarchy that I have used. See "Full Next Best View Installation". 


### Full Next Best View installation

This is the folder hierarchy:

- NBV Planning
    - octomap
    - PartialModel
    - iniparser
    - 

   [octomap]: <https://octomap.github.io/>
   [iniparser]: <https://github.com/ndevilla/iniparser>
   [J. Irving Vasquez-Gomez]: <https://jivasquez.wordpress.com>


