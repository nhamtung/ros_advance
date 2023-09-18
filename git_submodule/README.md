# git_submodule

# Reference link: https://dominhhai.github.io/vi/2017/09/git-submodule-multi-repo/

# User guide:
- Create and clone a submodule:
    + Direct to folder to save submodule: $cd src/ros_advance/git_submodule
    + Create and clone git rplidar_ros: $git submodule add https://github.com/Slamtec/rplidar_ros.git
    + Check status: $git status

- Update submodule in git (new clone the git): 
    + Direct to git folder: $cd src/ros_advance 
    + Init: $git submodule init
    + Update: $git submodule update

- Remove submodule: $git rm -f <path-to-submodule-file>

