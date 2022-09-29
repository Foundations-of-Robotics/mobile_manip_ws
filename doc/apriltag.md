## April tag detection
### Modify the apriltag config file
Move to the config file:
```
roscd mobile_manip/config
sudo gedit tags.yaml
```

Look for the `standalone_tags` array in the file and add your tag information, for instance:
```
standalone_tags:
 [
{id: 0, size: 0.144, name: ETS_Target_small}
{id: 0, size: 0.21, name: ETS_Target}
 ]
```

### usage
As long as a camera is running (simulated or real), you can launch individually the April tag detection with:
```
$ roslaunch apriltag_ros continuous_detection.launch camera_name:="/mobile_manip/t265/fisheye1" camera_frame:="t265_fisheye1_optical_frame" image_topic:="image_raw"
```
This command uses one of the fisheye camera from the Realsense T265. Note that all assignment launch files already include this node. Using this manual launching command should not be required for the assignments.
