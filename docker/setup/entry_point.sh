#!/bin/bash
# Entry point for services running inside the container

source ~/.bashrc

# launching rosmaster + jupyter notebook in the background
cd /mobile_manip_notebooks
roscore > /dev/null &
jupyter notebook --ip 0.0.0.0 --port ${JUPYTER_PORT} --NotebookApp.token='' --NotebookApp.password='' --allow-root --no-browser > /dev/null &

# preventing the docker "CMD" from ending
tail -f /dev/null