roslaunch traj_server traj_server.launch & sleep 2;
roslaunch plan_manage tracking_model.launch & sleep 1;
#roslaunch rplidar_ros rplidar.launch & sleep 2;
wait

