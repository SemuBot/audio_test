# audio_test

ROS2 test package for getting the data from the `/audio` topic <br/>

Used with the following packages: <br/>
https://github.com/SemuBot/respeaker_ros.git <br/>
https://github.com/ros-drivers/audio_common.git (ros2 branch) <br/>

In your ROS2 workspace: <br/>

* cd /src <br/>
* git clone this repo <br/>
* cd ..
* rosdep ? <br/>
* colcon packages select --... <br/>

Then, run in separate terminals: <br/>

* ros2 run respeaker_node respeaker_ros <br/>
* ros2 run semubot_eyes ... <br/>
* ros2 run audio_test ... <br/>

