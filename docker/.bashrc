# After building a ROS2 workspace certain environment variables need to
# be set to find packages and load libraries. This is accomplished by sourcing
# the generated setup file. To avoid manually running this command everytime a
# container is brought up, this command is automatically executed when the
# .bashrc is sourced.
source /home/docker/ament_ws/install/setup.bash