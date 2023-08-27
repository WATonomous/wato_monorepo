# Playing ROS2 Bags

A bag is a file that stores serialized ROS2 message data. We can play a bag to make data available on various topics.
More on bags can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html.

## To Use

Add `data_stream` as an `ACTIVE_PROFILE` in `watod2-config.sh`. 

Run `watod2 up` (or however you want to launch the `data_stream` service). 

The working directory of the `data_stream` container should have a `nuscenes` directory, which contains the NuScenes dataset converted to ros2bag format. To confirm this, run `watod run data_stream ls nuscenes` to view the available bags. Each bag has its own directory. The location of the `.mcap` file is `<name>/<name>_0.mcap`. For example, one of the bags is in `nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`. 

Now, using `watod run data_stream [ROS2 BAG COMMAND]` you can run any `ros2 bag ...` command as documented here: http://wiki.ros.org/rosbag/Commandline. You probably want to explore `ros2 bag play ...`: http://wiki.ros.org/rosbag/Commandline#rosbag_play. (Since this documentation is in ROS1, you can replace `rosbag` with `ros2 bag` to run the equivalent command in ROS2)

Example: `watod2 run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`