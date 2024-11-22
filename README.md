# Cooperative Locomotion Framework with Quadrupedal Robots
- This repository was created for the paper [Layered Control for Cooperative Locomotion of Two Quadrupedal Robots: Centralized and Distributed Approaches](https://ieeexplore.ieee.org/abstract/document/10281391)


## Overview
This framework utilizes Model Predictive Control (MPC) with Single Rigid Body (SRB) Dynamics in the context of quadratic programs (QPs) to address cooperative locomotion in interconnected quadrupedal robots.  Experiments - https://www.youtube.com/watch?v=mzAFemO0XeI

- Controller of the A1 Robot in the Hybrid Dynamic Systems and Robot Locomotion Lab at Virginia Tech (Lab Website: https://www.kavehakbarihamed.com/)
- The planner employes interconnected SRB to represent the interconnected dynamics
- When using this code/method, please cite our paper (details at the end)
- This code uses the [RaiSim](https://github.com/raisimTech/raisimLib) physics engine for simulation
- The visualization is done using the [RaisimOgre](https://github.com/raisimTech/raisimOgre)
- The code is written in C++ and uses the Eigen library for linear algebra
- The code is designed to be run on Linux, and has been tested on Ubuntu 20.04/22.04


## Requirements:
- CMake
- Raisim and RaisimOgre (if using the Raisim environment)
- Eigen


## Where to git clone
Clone this repository any directory and please revise CmakeList based on one's specific env

## Prerequisite
- In bash, the following must be defined - it is a prerequisite when installing Raisim
```sh
export WORKSPACE=${HOME}/raisimenv
export LOCAL_INSTALL=${HOME}/raisimenv/raisim_build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim_build/lib
```
- HEADSUP: The name of these env variables are being reused in CmakeList in this repo. If you decide to use different env variable name, please also revise the CmakeList in this repo.
- Be sure to add your license in this repo. (Please relace the activation_placeholder.raisim in this repo to yours)
- Make sure to provide correct paths to RaiSim, Ogre, and RaiSimOgre in the CMakeLists.txt file

# Building
```sh
mkdir build
./cmakemake.sh
```
## Citation
If you found either the code or the paper useful, please cite our work:
**J. Kim, R. T. Fawcett, V. R. Kamidi, A. D. Ames and K. Akbari Hamed,,**"[Layered Control for Cooperative Locomotion of Two Quadrupedal Robots: Centralized and Distributed Approaches](https://ieeexplore.ieee.org/abstract/document/10281391)," **in IEEE Transactions on Robotics, vol. 39, no. 6, pp. 4728-4748, Dec. 2023, doi: 10.1109/TRO.2023.3319896.**


```
@ARTICLE{kim2023_cooperativeloco,
  author={Kim, Jeeseop and Fawcett, Randall T. and Kamidi, Vinay R. and Ames, Aaron D. and Hamed, Kaveh Akbari},
  journal={IEEE Transactions on Robotics}, 
  title={Layered Control for Cooperative Locomotion of Two Quadrupedal Robots: Centralized and Distributed Approaches}, 
  year={2023},
  volume={39},
  number={6},
  pages={4728-4748},
  doi={10.1109/TRO.2023.3319896}}
```