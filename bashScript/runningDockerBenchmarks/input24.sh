#! /bin/bash
ROS_DOMAIN_ID=24
source /opt/ros/humble/setup.bash
source /home/tim-external/ros_ws/install/setup.bash
ros2 run fsregistration ros2ServiceRegistrationFS3D & >/dev/null 2>&1
ros2 run underwaterslam conversionGPStoXYZ.py & >/dev/null 2>&1
#cd /home/tim-external/ros_ws/src/fsregistration/pythonScripts/matchingProfiling/

ros2 run underwaterslam odometryTest --ros-args --params-file /home/tim-external/ros_ws/src/UnderwaterSlam/params/radius1/Bob/run4.yaml & >/dev/null 2>&1
pid1=$!

sleep 60

ros2 bag play /home/tim-external/dataFolder/S3E/S3Ev1/S3E_Campus_Road_1/ -r 1.0

wait $pid1





