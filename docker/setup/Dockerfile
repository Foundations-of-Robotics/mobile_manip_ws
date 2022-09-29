FROM osrf/ros:noetic-desktop-full
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
 
WORKDIR /robot_manip_ws
COPY repo_files/ /robot_manip_ws/
COPY ./entry_point.sh/ /robot_manip_ws/

RUN apt-get update
RUN apt-get install -y git curl wget xauth

# installing essential requirements (based on : https://git.initrobots.ca/Nerea/robot_manip_ws/-/blob/noetic_docker/doc/install.md)

# installing realsense-ros (most likely the ROS package should be sufficent)
RUN apt-get install -y ros-noetic-realsense2-camera

# installing python3 + setting up conan (required for building mobile_manip_ws)
RUN apt install -y python3 python3-pip
RUN python3 -m pip install conan
RUN python3 -m pip install --upgrade requests
RUN conan config set general.revisions_enabled=1
RUN conan profile new default --detect > /dev/null
RUN conan profile update settings.compiler.libcxx=libstdc++17 default
ENV CONAN_REVISIONS_ENABLED=1

RUN apt install -y python3-rosdep ros-noetic-apriltag* ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-teleop-twist-keyboard ros-noetic-jackal-msgs ros-noetic-depthimage-to-laserscan
RUN python3 -m pip install filterpy ipywidgets jupyros notebook
RUN python3 -m pip install --upgrade numpy

# installing dependencies + building mobile_manip_ws
RUN rosdep install --from-paths src --ignore-src -y
RUN bash -c 'source scripts/add_remotes.sh && source /opt/ros/noetic/setup.bash && catkin_make'

# defining required env variables + writing to .bashrc
RUN echo "source /opt/ros/noetic/setup.sh" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /robot_manip_ws/devel/setup.sh" >> ~/.bashrc
RUN echo "source /robot_manip_ws/devel/setup.bash" >> ~/.bashrc

# installing and configuring the Gazebo web (browser-based) for visualization (based on : https://git.initrobots.ca/Nerea/robot_manip_ws/-/blob/noetic_docker/doc/install_gzweb.md)

# setting up python 2 for the gazebo setup steps
RUN apt install -y python2
RUN ln -sf /usr/bin/python2 /usr/bin/python

# making sure the prerequisites are installed
RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher mercurial build-essential cmake
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN bash -c 'source ~/.nvm/nvm.sh && nvm install 11'
RUN echo "source ~/.nvm/nvm.sh" >> ~/.bashrc
RUN apt install -y npm
RUN bash -c 'source ~/.nvm/nvm.sh && n=$(which node);n=${n%/bin/node}; chmod -R 755 $n/bin/*; sudo cp -r $n/{bin,lib,share} /usr/local'

# cloning the (mercurial) gazebo web repository +  first run of the installer to get all default Gazebo models
RUN git clone https://github.com/osrf/gzweb
RUN bash -c 'source ~/.nvm/nvm.sh && export GAZEBO_DIR=$(ls /usr/share/ | grep "gazebo-") && source /usr/share/$GAZEBO_DIR/setup.sh && ./gzweb/deploy.sh -m'

# copying the models specific to the lab environment, and running the installer again
RUN bash -c 'source /opt/ros/noetic/setup.sh && source /robot_manip_ws/devel/setup.sh &&\
cp -r `rospack find dingo_description` gzweb/http/client/assets/ &&\
cp -r `rospack find kortex_description`/arms/gen3_lite/6dof gzweb/http/client/assets/ &&\
cp -r `rospack find kortex_description`/grippers/gen3_lite_2f gzweb/http/client/assets/ &&\
cp -r `rospack find mobile_manip`/models/* gzweb/http/client/assets/ &&\
cp -r `rospack find mobile_manip`/models/april_tags/* gzweb/http/client/assets/ &&\
cp -r `rospack find common_gazebo_models` gzweb/http/client/assets/'
RUN bash -c 'source ~/.nvm/nvm.sh && export GAZEBO_DIR=$(ls /usr/share/ | grep "gazebo-") && source /usr/share/$GAZEBO_DIR/setup.sh && ./gzweb/deploy.sh -m local'

# setting up VNC to allow rendering of camera frames
RUN apt install -y libegl1-mesa x11-xkb-utils
RUN wget https://sourceforge.net/projects/virtualgl/files/3.0/virtualgl_3.0_amd64.deb/download -O virtualgl_3.0_amd64.deb
RUN wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb/download -O turbovnc_2.2.6_amd64.deb
RUN dpkg -i virtualgl*.deb 
RUN dpkg -i turbovnc*.deb
RUN rm virtualgl*.deb turbovnc*.deb

# defining the necessary env variables

ENV ROS_IP=127.0.0.1
ENV ROS_MASTER_URI=http://localhost:11311

ENV GZWEB_PORT=8887
ENV GAZEBO_MASTER_URI=http://localhost:1147

ENV JUPYTER_PORT=8888

# setting proper permissions and running the entry point
RUN chmod 777 entry_point.sh
CMD ./entry_point.sh