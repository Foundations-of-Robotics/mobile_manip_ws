#!/bin/bash
# Start and stop the (book_simulations) docker container
#
# Requirements:
#   This script needs to be executed from the /mobile_manip_ws/docker folder
#
# Usage: 
#   ./set_container_state.sh 0 -> stops the container
#   ./set_container_state.sh 1 -> starts the container
# -----------------------------------------------------------------------------

# getting the launch policy
container_state=$1

if [ "$container_state" -eq 1 ]; then

    # launching the container with the mounted workspace
    work_env_path="$(pwd | rev | cut -d'/' -f3- | rev)/mobile_manip_notebooks"
    docker run -p 8887:8887 -p 8888:8888 -v $work_env_path:/mobile_manip_notebooks book_simulations

else

    # stoping the container via id
    image_id=$(docker images book_simulations --format="{{.ID}}")
    container_id=$(docker container ps --filter "ancestor=$image_id" --format "{{.ID}}")
    docker container stop $container_id

fi