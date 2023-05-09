#sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 & sleep 5;
roslaunch planner mocap_G305.launch_real & sleep 3;
roslaunch planner occ_plan_real.launch & sleep 2;
wait;