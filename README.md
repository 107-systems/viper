<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `viper`
=====================
[![Build Status](https://github.com/107-systems/viper/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/viper/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/viper/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/viper/actions/workflows/spell-check.yml)

Generic ROS based drone flight stack for the [Pika Spark](https://pika-spark.io/).

<p align="center">
  <a href="https://github.com/107-systems/viper"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="40%"></a>
</p>

#### How-to-build
* Install `gsl-lite`
```bash
git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `Catch2`
```bash
git clone https://github.com/catchorg/Catch2 && cd Catch2
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `fmt`
```bash
git clone https://github.com/fmtlib/fmt && cd fmt
mkdir build && cd build
cmake -DFMT_TEST=OFF ..
make -j8
sudo make install
```
* Install `mp-units`
```bash
git clone https://github.com/mpusz/mp-units && cd mp-units
mkdir build && cd build
cmake -DMP_UNITS_AS_SYSTEM_HEADERS=ON -DMP_UNITS_BUILD_LA=OFF ..
make -j8
sudo make install
```
* Build with `colcon`
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/viper
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select viper
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch viper viper.py
```

#### Interface Documentation
##### Published Topics
| Default name |                                      Type                                      |
|:------------:|:------------------------------------------------------------------------------:|

##### Subscribed Topics
| Default name |                                          Type                                          | Description                                    |
|:------------:|:--------------------------------------------------------------------------------------:|------------------------------------------------|
| `/cmd_vel`   | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)  | Linear/angular drone target velocity setpoints |

##### Parameters
|                      Name                      |       Default        | Description                                                             |
|:----------------------------------------------:|:--------------------:|-------------------------------------------------------------------------|

