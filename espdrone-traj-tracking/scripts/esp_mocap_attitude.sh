roslaunch espdrone-traj-tracking mocap_G305.launch & sleep 3;
roslaunch espdrone-traj-tracking esp_traj.launch & sleep 5;
roslaunch espdrone-traj-tracking lee_controller.launch & sleep 2;
wait;