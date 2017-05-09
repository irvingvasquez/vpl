# View Planning Library

> CONACyT

> Instituto Nacional de Astrofísica Óptica y Electrónica.

> Centro de Innovación y Desarrollo Tecnológico 

VPL(The acronym of view planning library) provides a platform to develop view planning algorithms and perform comparisons quickly. VPL is written in C++ and it is based on a set of well known libraries: octomap and pcl. VPL provides the data structures to represent the space, provides visibility algorithms, implements several view planning algorithms reported in the literature and provides flexibility to link with range sensor simulators and motion planning algorithms. VPL was developed by [J. Irving Vasquez-Gomez] under New BSD licence.

VPL is composed of 2 core libraries: PartialModel and ViewPlanning. PartialModel stores the information about object that is being reconstructed and provides a set of functions to handle visibility for next best view calculation. 
ViewPlanning provides planning algorithms to achieve an automated reconstruction.

More information is in our paper (under development):

If your are using VPL in an academy work please cite:

Vasquez-Gomez, J. I., Sucar, L. E., & Murrieta-Cid, R. (2017). View/state planning for three-dimensional object reconstruction under uncertainty. Autonomous Robots, 41(1), 89-109.

```
@article{vasquez2017view,
  title={View/state planning for three-dimensional object reconstruction under uncertainty},
  author={Vasquez-Gomez, J Irving and Sucar, L Enrique and Murrieta-Cid, Rafael},
  journal={Autonomous Robots},
  volume={41},
  number={1},
  pages={89--109},
  year={2017},
  publisher={Springer}
}
```


### Requirements

Before installing VPL you need to install the following libraries:
- octomap
- MRPT
- boost


### Installation

First, download and install [octomap] and [MRPT]. Modify the CMakeList.txt file to specify your installation addresses or follow the folder hierarchy that I have used. See "Full VPL Installation". 

### Full VPL installation

we recommend that VPL will be installed in the same folder that the required libraries:

- octomap-devel

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
   [MRPT]: <http://www.mrpt.org/>
   [J. Irving Vasquez-Gomez]: <https://jivasquez.wordpress.com>

