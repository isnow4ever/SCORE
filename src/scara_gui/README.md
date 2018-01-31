source ${workspace_name}/devel/setup.bash
rqt --force-discover
mv x50.rviz ~/.rviz/default.rviz
rosrun rqt_gui rqt_gui
