# Parking Management System
![SmartEPark-logo](doc/Logo_Smart-E-Park_RGB.png)

## Overview
This repository contains the source code of the parking management system developed in the project SmartEPark by the FZI. The source code is made public in accordance with the projects open source strategy.

The parking management system is implemented as a C++ ROS package and should be installed within a catkin workspace. It is designed to automate a valet parking system. Multiple autonomous driving grades with different function distributions are supported.


## Installation
The Parking Management System relies on [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html) for building and is targeted towards Linux. At least **C++14** is required.

### Dependencies
Besides [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html), the dependencies are
* `Boost`
* [`lanelet2`](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
* several ROS messages which can be found in the [package.xml](package.xml)

* A separate file called [required_functions.md](required_funcitons.md) which can be found in this repository lists and describes all proprietary functions, structs and libraries which are used in this package. They will have to be implemented by the user before using this package.

The catkin workspace can be built with the [catkin build-tools](https://catkin-tools.readthedocs.io/en/latest/index.html).

## Citation
If you are using ROS Parking Management for scientific research, please cite our [publication](https://ieeexplore.ieee.org/abstract/document/9565095).

```latex
@INPROCEEDINGS{9565095,
  author={Schörner, Philip and Conzelmann, Marcus and Fleck, Tobias and Zofka, Marc and Zöllner, J. Marius},
  booktitle={2021 IEEE International Intelligent Transportation Systems Conference (ITSC)},
  title={Park my Car! Automated Valet Parking with Different Vehicle Automation Levels by V2X Connected Smart Infrastructure},
  year={2021},
  pages={836-843},
  doi={10.1109/ITSC48978.2021.9565095}}
```

# Credits
The provided code was developed and created by the [FZI Research Center for Information Technology](https://www.fzi.de).

The [Ministry of Transport (VM) of Baden-Württemberg](https://mwk.baden-wuerttemberg.de/de/startseite/) and the [Ministry of Science, Research and Art (MWK) of Baden-Württemberg](https://mwk.baden-wuerttemberg.de/de/startseite/) are 
sponsors and supporters of the [Smart Mobility](https://www.e-mobilbw.de/ueber-uns/projektaktivitaeten/smart-mobility) program. SmartEPark is one of the five supported projects aimed to increase understanding of autonomous driving. 
The SmartMobility programm was coordinated by [E-Mobil](https://www.e-mobilbw.de/en/).

<img src="doc/FZI_Logo.svg" height="100"/>
<img src="doc/VM-BW-Logo.png" alt="Logo of the Ministry of Transport (VM) of Baden-Württemberg" height="100"/>
<img src="doc/MWK-BW-Logo.png" alt="Logo of the Ministry of Science, Research and Art (MWK) of Baden-Württemberg" height="120"/>
<img src="doc/Logo_E-Mobil.png" alt="Logo of the Ministry of Science, Research and Art (MWK) of Baden-Württemberg" height="020"/>
