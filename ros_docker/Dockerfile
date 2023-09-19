# Reference link: https://viblo.asia/p/tim-hieu-ve-dockerfile-va-tao-docker-image-V3m5WWag5O7

# Sử dụng một hệ điều hành cơ sở
# FROM ubuntu:20.04
FROM ros:noetic

LABEL version="1.0.0" description="Example about using docker"
# MAINTAINER nhamtung <nhamtung125@gmail.com>

ENV USERNAME="robotics" PASSWORD="robotics" 
ENV ROBOT_TYPE="SASs_AMR" ROBOT_NAME="SASs_AMR_100"
ENV ROS_MASTER_URI="http://localhost:11311" ROS_HOSTNAME="localhost"

# Cài đặt các gói cơ bản
RUN apt-get update && apt-get install -y ros-noetic-desktop && rm -rf /var/lib/apt/lists/
# RUN apt-get update && apt-get install -y build-essential cmake python3 python3-pip

# Cài đặt các gói ROS bổ sung thông qua rosdep
RUN apt-get update && apt-get install -y python-rosdep && rosdep init && rosdep update
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

# Cài đặt ROS (Thay 'noetic' bằng phiên bản ROS bạn muốn cài đặt)
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt install curl
# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# RUN apt-get update
# RUN ros-noetic-desktop
# RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# RUN apt install python3-rosdep
# RUN rosdep init && rosdep update

# Khởi động môi trường ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Cài đặt và khởi động một số goi phu thuoc
# RUN apt-get update
# Install GRPC (for server):
# RUN apt-get install ros-noetic-grpc
# Install spnav (for joystick_drivers):
# RUN apt-get install libspnav-dev && pip install ds4drv && apt-get install ros-noetic-joy

# Tạo thư mục làm việc và sao lưu workspace ROS
WORKDIR /SASs_AMR_ws
RUN mkdir -p /SASs_AMR_ws/src
COPY $HOME/TungNV/ros_ws /SASs_AMR_ws
# Xây dựng catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /SASs_AMR_ws && catkin_make"

# Mở cổng cho ROS Master (nếu cần)
EXPOSE 11311

RUN source devel/setup.bash
RUN rosclean purge
RUN roslaunch topic_pkg topic_cpp.launch
# RUN roslaunch sass_psx psx_control_manual.launch

# Khởi động lệnh mặc định khi container được chạy
CMD ["bash"]