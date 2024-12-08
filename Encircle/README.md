## 基于位姿真值的多机巡逻
cd ~/PX4-Autopilot/
roslaunch px4 robocup.launch
## 建立通信
cd ~/XTDrone/communication
bash multi_vehicle_communication.sh
## 获取位置
cd ~/XTDrone/sensing/pose_ground_truth/
python get_local_pose.py typhoon_h480 6
话题名称：vehicle_type+'_'+str(i)+'/mavros/local/pose'
## 运行脚本解锁6台无人机
cd ~/catkin_ws/src/JoeSanders_cpp_package/
rosrun JoeSanders_cpp_package arm_node typhoon_h
## 运行脚本
cd ~/catkin_ws/src/JoeSanders_cpp_package/

rosrun JoeSanders_cpp_package Encircle_node_1 typhoon_h
rosrun JoeSanders_cpp_package Encircle_node typhoon_h
rosrun JoeSanders_cpp_package Encircle_node_2 typhoon_h
rosrun JoeSanders_cpp_package Encircle_node_3 typhoon_h
rosrun JoeSanders_cpp_package Encircle_node_4 typhoon_h
rosrun JoeSanders_cpp_package Encircle_node_5 typhoon_h
