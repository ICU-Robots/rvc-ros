# rvc-ros

## Installation on Raspbian (Buster)

1. Follow steps 1-3 from [this guide](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi) to install ROS Melodic (ROS-Comm) from source. 
2. Use step 4.2 from the above guid to add the `sensor_msgs` package to your installation.
3. Install the `{fmt}` library.
    1. Using `git clone https://github.com/fmtlib/fmt.git` download the source.
    2. Enter the fmt directory and follow [these instructions](https://fmt.dev/latest/usage.html) to build and install the library.
4. Enter your ROS workspace, then enter your `src` directory.
5. Download the `serial` library with `git clone https://github.com/wjwwood/serial.git`.
6. Download this package with `git clone https://github.com/RedRussianBear/rvc-ros.git`.
7. Move back to your ROS workspace and build the packages with `catkin_make`.
8. Source your workspace with `source devel/setup.bash`.
