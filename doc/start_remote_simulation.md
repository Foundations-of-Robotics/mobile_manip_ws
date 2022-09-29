# Run a simulation remotely

*You need to be connected to your remote computer's network to be able to connect to it via SSH and run the simulation*

- Open a session via SSH on your remote computer using your normal login account (use the computer's IP address to connect).
*On MacOS or Linux you can simply use the command `ssh <user>@stationX.domain` in a terminal. On Windows, *Putty* is the best client ([free here](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)):*

- Run the simulation by its launch file using the script `remote_viz`, for example (see your project instructions) :

`./start_remoteviz.sh dingo_arenasim.launch`

- In another terminal, start the gazebo web script : 

`./start_gzweb.sh`

- If Gazebo web started correctly, you should see this output in the terminal : 

```
mecbotg1@MEC-A2230-01:~$ ./start_gzweb.sh

> gzweb@1.3.0 start /home/admin_mec/gzweb
> if [ $npm_config_port ]; then port=$npm_config_port; fi; cd gzbridge && ./server.js $port "8081"

Wed Feb 09 2022 09:20:26 GMT-0500 (Eastern Standard Time) Static server listening on port: 8081
--------------------------------------------------------------
Gazebo transport node connected to gzserver.
Pose message filter parameters between successive messages: 
  minimum seconds: 0.02
  minimum XYZ distance squared: 0.00001
  minimum Quartenion distance squared: 0.00001
--------------------------------------------------------------
```

- You can now see the simulation in your web browser (Chrome is recommended) at the address **stationX.domain:808X** where X is your team number.

- To access ROS base commands (rostopic, rosservice, rosbag, etc.) open another SSH terminal.

- The python jupyter notebooks are accessible with the same user login at the address **stationX.domain**. 
*NOTE* : it is possible the notebooks show a warning because of the ROS logger, you can ignore that warning. *NOTE2* : it is possible your browser shows a warning because of the site security (without SSL), you can ignore this warning.

- To retrieve the files from the remote computer (rosbags for example) it is recommended to use the utilitary *Filezilla* ([free here](https://dl2.cdn.filezilla-project.org/server/FileZilla_Server-0_9_60_2.exe?h=FBIALVdGHIi3WNim1XLFIQ&x=1587753925)) with your connection information (address and user login, port 22). On MacOs and Linux, you can simply use the command `scp <user>@stationX.domain:your_file` in a terminal.

##### To restart the simulation, you can stop the one in progress by using `Control-C` in the terminals (`remoteviz` and `gzweb`). The complete stop can take a few moments. When the prompt comes back, you can start the simulation again. *NOTE*: To ensure there is no connection problem, it is strongly recommended that you use the command `exit` when closing your session on the remote computer.
