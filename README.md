3D Systems Touch ROS Driver
============

ROS Packages for connecting *one* or *more* 3D Systems Touch (previously known as Phantom Omni or Geomagic Touch) haptic devices, **USB** version.
This repository is forked from the [Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) repository.


## Environment
- **Confirmed Device**:  
  This package is tested with the **3D Systems Touch HID** haptic device.
  While it has not been specifically tested with the **Touch X**, it is likely to work with it as well.  
- **Supported OS**:  
  This package is tested on **Ubuntu 20.04** and **Ubuntu 22.04**.  

- **ROS Version**:  
  Tested with **ROS Noetic**.  

- **Notes for Ubuntu 22.04**:  
  The package has been tested only in a Docker environment running a **ROS Noetic setup on an Ubuntu 20.04 image**.  


## Install Device Driver and SDK (OpenHaptics)

Run the following command to install the device driver.  
**Driver Version**: `TouchDriver_2024_09_19`  
**Compatibility**: Ubuntu 20.04 and 22.04  

> **Note**: Although the official documentation does not explicitly mention support for Ubuntu 20.04, confirmation from 3D Systems indicates that this driver is compatible with Ubuntu 20.04 as well.  

```bash
./touch_hid_driver_install.sh
```

During the installation process, you can test the connection of the device by responding to the prompt:  

```bash
Do you want to run TouchCheckup for connection test? (OpenHaptics setup is included) [y/n]: y
```

## Launch ROS Node

Clone and build this repository.
```bash
roslaunch omni_common omni_state.launch
```

Data from the haptic device can be read from the following topics:

  /phantom/button
  
  /phantom/force_feedback
  
  /phantom/joint_states
  
  /phantom/pose
  
  /phantom/state

## Use Multiple Devices

Multiple devices can be connected by adding a unique name to each device. **Following settings need to be made every time the devices are reconnected**:

1. Run Geomagic_Touch_Setup in /opt/geomagic_touch_device_driver/

2. Add a new name by pressing `Add...` button in `Device Name` section and typing that name in the pop-up window

3. Select that name in the drop-down list of `Device Name`

4. Select `Port Num` of the device which you want to add that name to

5. Press `Apply`

Example to run two devices (`Left Device` and `Right Device`):
```
roslaunch omni_common dual_phantom.launch
```

## Acknowledgements and Citation
This repository is forked from the [Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) repository.
The original repository by Francisco Suárez Ruiz, [http://fsuarez6.github.io](http://fsuarez6.github.io) for the Sensable PHANToM haptic device (https://github.com/fsuarez6/phantom_omni).

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC).


Please cite these papers in your publications if this repository helps your research.

```
@INPROCEEDINGS{8941790,
  author={Mathur, Bharat and Topiwala, Anirudh and Schaffer, Saul and Kam, Michael and Saeidi, Hamed and Fleiter, Thorsten and Krieger, Axel},
  booktitle={2019 IEEE 19th International Conference on Bioinformatics and Bioengineering (BIBE)},
  title={A Semi-Autonomous Robotic System for Remote Trauma Assessment},
  year={2019},
  volume={},
  number={},
  pages={649-656},
  doi={10.1109/BIBE.2019.00122}}
  
@inbook{doi:10.1137/1.9781611975758.2,
author = {B. Mathur and A. Topiwala and H. Saeidi and T. Fleiter and A. Krieger},
title = {Evaluation of Control Strategies for a Tele-manipulated Robotic System for Remote Trauma Assessment},
booktitle = {2019 Proceedings of the Conference on Control and its Applications (CT)},
chapter = {},
pages = {7-14},
doi = {10.1137/1.9781611975758.2},
URL = {https://epubs.siam.org/doi/abs/10.1137/1.9781611975758.2},
eprint = {https://epubs.siam.org/doi/pdf/10.1137/1.9781611975758.2}}
```

