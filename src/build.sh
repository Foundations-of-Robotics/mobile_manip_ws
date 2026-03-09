 #!/bin/bash
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rmf_gazebo
colcon build --symlink-install
source install/setup.bash
