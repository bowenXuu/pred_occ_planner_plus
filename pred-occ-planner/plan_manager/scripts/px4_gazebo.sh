roslaunch mavlink_sitl_gazebo simulation_sts_iris.launch & sleep 5;
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" & sleep 3;
roslaunch planner occ_plan.launch & sleep 2;
wait;

