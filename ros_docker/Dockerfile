# Reference link: https://viblo.asia/p/tim-hieu-ve-dockerfile-va-tao-docker-image-V3m5WWag5O7

# Sử dụng một hệ điều hành cơ sở
FROM ros:noetic

LABEL version="1.0.0" description="Example about using docker"

ENV USERNAME="nhamtung" PASSWORD="12051994" 
ENV ROBOT_TYPE="SASs_AMR" ROBOT_NAME="SASs_AMR_100"
ENV ROS_MASTER_URI="http://localhost:11311" ROS_HOSTNAME="localhost"

# Add User
RUN useradd -m $USERNAME && echo "$USERNAME:$PASSWORD" | chpasswd && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER $USERNAME

# Khởi động môi trường ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Cai mot so goi phu thuoc
RUN sudo apt-get update
RUN sudo apt-get install -y python3 python3-pip
RUN sudo pip3 install ds4drv
RUN sudo apt-get install -y ros-$ROS_DISTRO-joy
RUN sudo apt-get install -y ros-$ROS_DISTRO-grpc
RUN sudo apt-get install -y libspnav-dev && sudo apt install -y libx11-dev
RUN sudo apt-get install -y ros-$ROS_DISTRO-roslint
RUN sudo apt-get install -y ros-$ROS_DISTRO-tf2-ros

# Remove lib/apt
RUN sudo rm -rf /var/lib/apt/lists/*

# Tạo thư mục làm việc và sao lưu workspace ROS
WORKDIR /SASs_AMR_ws
RUN mkdir -p /SASs_AMR_ws/src
COPY $HOME/TungNV/ros_ws/src /SASs_AMR_ws/src
RUN cd /SASs_AMR_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"
RUN /bin/bash -c "source devel/setup.bash"
# RUN roscd topic_pkg

# Mở cổng cho ROS Master (nếu cần)
EXPOSE 11311

# Khởi động lệnh mặc định khi container được chạy
CMD ["bash"]
# CMD ["roslaunch", "topic_pkg", "topic_cpp.launch"]