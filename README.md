# RoboRTS-Edu

![MelodicBuildCI](https://github.com/Artinx-Robotics/RoboRTS-Edu/workflows/MelodicBuildCI/badge.svg)

This project is an education version of [RoboRTS](https://github.com/RoboMaster/RoboRTS) with only navigation-related modules.

## Build and Run

```bash
mkdir -p roborts_edu_ws/src
cd roborts_edu_ws/src
git clone https://github.com/SUSTech-Robotics/RoboRTS-Edu.git
cd ..

sudo rosdep init
#The command ‘sudo rosdep init’ will print an error if you have already executed it since installing ROS. 
#This error can be ignored.

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make


```

## Tutorial

For more information about RoboMaster AI Robot platform and RoboRTS framework, please refer to [RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/)

## Competition

RoboMaster holds AI Challenge since 2017. In the challenge, multiple robots should fight with each other on a game field automatically under different rules.

For more information, please refer to

- [DJI RoboMaster 2019 ICRA AI Challenge](https://icra2019.org/competitions/dji-robomaster-ai-challenge)

- [DJI RoboMaster 2018 ICRA AI Challenge](https://icra2018.org/dji-robomaster-ai-challenge/)


## Acknowledgements

* [amcl](http://wiki.ros.org/amcl)
* [costmap_2d](http://wiki.ros.org/costmap_2d)
* [teb_local_planner](http://wiki.ros.org/teb_local_planner)

## Copyright and License

RoboRTS is provided under the [GPL-v3](COPYING).


