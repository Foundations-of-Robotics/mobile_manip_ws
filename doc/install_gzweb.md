### Install GzWeb

These instructions are adapted from the official [Gazebo web tutorial](https://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb).

Ensure you have nodejs version 11 and the prerequisites:
```
sudo apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
source ~/.bashrc
nvm install 11
```
If you want all users on that machine to have access to this local manual install of NPM, run:
```
n=$(which node);n=${n%/bin/node}; chmod -R 755 $n/bin/*; sudo cp -r $n/{bin,lib,share} /usr/local
```
then clone the (mercurial) repository:
```
cd ~; git clone https://github.com/osrf/gzweb.git
```
Source the Gazebo files and do a first run of the installer to get all default Gazebo models:
```
source /usr/share/gazebo/setup.sh
cd gzweb; ./deploy.sh -m
```
NOTE: the deploy script runs on Python 2. If you are on a fresh Ubuntu20 installation, you may have to `sudo apt install python2` and then force the operating system into using Python 2 by default `cd /usr/bin; sudo ln -sf python2 python`. You may be better to `sudo rm /usr/bin/python` afterwards.


Then, copy the models specific to our lab environment, and run the installer again:
```
cp -r `rospack find dingo_description` http/client/assets/
cp -r `rospack find kortex_description`/arms/gen3_lite/6dof http/client/assets/
cp -r `rospack find kortex_description`/grippers/gen3_lite_2f http/client/assets/
cp -r `rospack find mobile_manip`/models/* http/client/assets/
cp -r `rospack find mobile_manip`/models/april_tags/* http/client/assets/
cp -r `rospack find common_gazebo_models` http/client/assets/
./deploy.sh -m local
```

You are done! Whenever a Gazebo simulation is running(with or without GUI), you can access it in a browser by launching GzWeb with:
```
cd ~/gzweb; npm start
```

## VNC
For remote visualisation of simulations runing on server but also launch remotely, we must provide Gazebo with a display. That way the simulator does render frames and output the simulated camera feeds. We use [TurboVNC together with VirtualGL](https://kitware.github.io/paraviewweb/docs/virtualgl_turbovnc_howto.html):
```
sudo apt install libegl1-mesa
wget https://sourceforge.net/projects/virtualgl/files/3.0/virtualgl_3.0_amd64.deb/download -O virtualgl_3.0_amd64.deb
sudo dpkg -i virtualgl*.deb
rm virtualgl*.deb

wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb/download -O turbovnc_2.2.6_amd64.deb
sudo dpkg -i turbovnc*.deb
rm turbovnc*.deb
```
Then stop the window manager and unload the video modules:
```
sudo service gdm stop
sudo rmmod nvidia_drm nvidia_modeset nvidia_uvm nvidia
```
and configure vgl:
```
sudo /opt/VirtualGL/bin/vglserver_config
```
Reboot to apply the changes. You can now start a simulation using:
```
/opt/TurboVNC/bin/vncserver :99
export DISPLAY=:99
vglrun roslaunch mobile_manip gen3_lite_sim.launch
```
If a prompt ask you for a password, just hit Enter (leave it empty).
