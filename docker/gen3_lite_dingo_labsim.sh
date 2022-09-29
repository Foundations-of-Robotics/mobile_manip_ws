#!/bin/bash
# Launches the (gen3_lite_dingo_labsim) simulation -> accessible through gzweb
#
# Usage: 
#   ./gen3_lite_dingo_labsim.sh
# -----------------------------------------------------------------------------

# getting the container id
image_id=$(docker images book_simulations --format="{{.ID}}")
container_id=$(docker container ps --filter "ancestor=$image_id" --format "{{.ID}}")

# vnc server configuration
docker exec -it $container_id bash -c '/opt/TurboVNC/bin/vncserver :99'

# launching the simulation
docker exec -it $container_id bash -c '\
    source /opt/ros/noetic/setup.sh;\
    source /robot_manip_ws/devel/setup.sh;\
    source ~/.nvm/nvm.sh;\
    export DISPLAY=:99.0;\
    xhost +local: ;\
    roslaunch mobile_manip gen3_lite_dingo_labsim.launch &\
    sleep 10;\
    cd /robot_manip_ws/gzweb && npm start -p ${GZWEB_PORT}'