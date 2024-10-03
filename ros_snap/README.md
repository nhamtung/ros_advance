
# Require
- Linux kernel 4.15.0-136-generic
- Ubuntu 20.04 Focal Fossa
- ROS1 Noetic
- Install snap:
    + $sudo apt update
    + $sudo apt install snapcraft

- Create snapcraft.yaml file
    + Direct to workspace
    + Create and edit to file: $sudo nano ./snapcraft.yaml

- Build Snap:
    + Direct to workspace
    + $snapcraft
    + $sudo snap install my-ros-project_0.1_amd64.snap --devmode

- Run Snap:
    + $my-ros-project.my_ros_node
