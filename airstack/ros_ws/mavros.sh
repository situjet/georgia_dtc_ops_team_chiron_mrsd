export ROBOT_NAMESPACE=dtc_mrsd_
source install/setup.bash
export ROS_DOMAIN_ID=70

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/dtc_mrsd_/mavros \
  -p fcu_url:="serial:///dev/ttyACM0:115200" \
  -p tgt_system:=1 \
  -p tgt_component:=1 \
  -p fcu_protocol:="v2.0" \
  --params-file /root/ros_ws/src/autonomy/0_interface/mavros_interface/px4_pluginlists.yaml \
  --disable-stdout-logs \
  --disable-rosout-logs \
