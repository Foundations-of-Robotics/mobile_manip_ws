# Mobile Manipulator Lab Users management
This ROS workspace is designed for academic teaching laboratories in robotics, from Gazebo simulation to physical deployment on robotic platforms. The students are not expected to install or deploy this workspace on their personal computer, but rather use a server infrastructure at the university. The laboratories were designed at École de technologie supérieure, in Montréal, Canada, where we host the physical infrastructure to which these tools are tailored. We have 8 robotic platforms for the students to test their algorithms and several shared desktop stations. The stations are accessible remotely. We tested up to 3 teams running their Gazebo simulation in parallel on a single station without degrading performances. Each station is accessible to all teams and so teams accounts are set on each station. To ease maintenance and updates each station has only one user (*admin_mec*) hosting the ROS workspace and all teams launch from that workspace. Since most of the work is done in Jupyter notebooks, we deploy on each station (the littlest) [JupyterHub](https://tljh.jupyter.org/en/latest/).

## Deployment scripts
These scripts are either mentioned in the installation instruction or in the assignment instructions.
- `add_remotes.sh`: create the git remotes for all subtree modules of this repository;
- `build_real.sh`: configure catkin_make to build onboard the Dingo;
- `remote_dingo.sh`: set up the environment variables to easily connect to a Dingo ROS master;
- `start_gzweb.sh`: launch the Gazebo web visualization on a unique port define in the user `.bashrc`;
- `start_remoteviz.sh`: launch a Gazebo ROS simulation remotely using a random VNC server port.

## User management scripts
The files inside the `users_mgt` folder are shared to inspire other deployments but are heavily tailored to ÉTS infrastructure and not well documented. The notebooks can be run locally on the station using the standard `jupyter notebook` command, but to run them remotely, JupyterHub is used. The instructions to deploy JupyterHub are in [doc/jupyterhub.md](doc/jupyterhub.md). Then a new user can be created using these instructions:

- In a SSH terminal `sudo adduser mec745gXX` and create a password for the user.
- In the admin panel of jupyterhub, create a user with the same login.
- Logout of JupyeterHub and connect using the newuser credentials.
- In the SSH terminal, `cd /home/mec745XX; ln -sf jupyter-notebooks ../jupyter-mec745gXX`
- Then `sudo usermod -a -G mec745gXX jupyter-mec745gXX` and `sudo chmod g+wr -R ../jupyter-mec745gXX`
- Finally, copy these lines to the .bashrc of the new user, changing the last digits with the team number:
```
export GAZEBO_MASTER_URI=http://localhost:1134<team_number>
export ROS_MASTER_URI=http://localhost:1131<team_number>
export GZWEB_PORT=808<team_number>
export ROSBRIDGE_PORT=909<team_number>
```

However, we built scripts to automatise these steps:
- `.local`: a folder with all the default Python packages requires for JupyterHub to run the assignments notebooks;
- `.bashrc`: a default user configuration file;
- `create_users.sh`: a script to generate users on the station with random safe passwords and add teams to JupyterHub configuration;
- `config_users.sh`: a script to copy the essential launch files and add the team-specific environment variables;
- `clean_user.sh`: a script to remove the environment variable and some run scripts;
- `remove_user.sh`: a script to delete the users from the station along with their home folder.

### Other useful commandline Linux tools
- `who`: to see who's connected to the current station
- `w username`: to see what user AP3333 is currently running
- `pkill -u username`: to kill all process of a user


### Specific commands to check user's jupyterhub config
```
sudo nano /opt/tljh/config/config.yaml
sudo tljh-config show
sudo tljh-config reload
```
