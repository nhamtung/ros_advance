# Reference link
- https://wiki.ros.org/ROS/Tutorials/Packaging%20your%20ROS%20project%20as%20a%20snap#:~:text=Packaging%20your%20ROS%20project%20as%20a%20snap.%20Description:

# Require
- Linux kernel 4.15.0-136-generic
- Ubuntu 20.04 Focal Fossa
- ROS1 Noetic
- Install snap:
    + $sudo apt update
    + $sudo snap install --classic snapcraft

- Create snapcraft.yaml file
    + Direct to workspace
    + Create and edit to file: $snapcraft init

- Build Snap:
    + Direct to workspace
    + $snapcraft --enable-experimental-extensions
    + $sudo snap install test_snap-project_1.1_amd64.snap --devmode

- Run Snap:
    + $test_snap-project.tungnv-test-snap

# Share Snap to snapstore
- Sharing the snap: 
    + Direct to workspace
    + $snapcraft login
    + $snapcraft register test_snap-project
    + $snapcraft push ./test_snap-project.snap --release=stable
- Install the snap:
    + $sudo snap install test_snap-project

