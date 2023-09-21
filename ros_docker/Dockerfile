# Reference link: https://viblo.asia/p/tim-hieu-ve-dockerfile-va-tao-docker-image-V3m5WWag5O7

# Sử dụng một hệ điều hành cơ sở
FROM ros:noetic
LABEL version="1.0.0" description="Example about using docker"

ENV GROUPNAME="PNKX" USERNAME="robotics" PASSWORD="robotics" 
ENV ROBOT_TYPE="SASs_AMR" ROBOT_NAME="SASs_AMR_100"
ENV ROS_MASTER_URI="http://localhost:11311" ROS_HOSTNAME="localhost"

# Sử dụng htpasswd để tạo tệp chứa mật khẩu
RUN apt-get update
RUN apt-get install -y nano
RUN apt-get install -y net-tools
RUN apt-get install -y iputils-ping
RUN apt-get install -y apache2-utils apache2
RUN sudo mkdir -p /etc/docker
RUN htpasswd -cb /etc/docker/htpasswd $USERNAME $PASSWORD
COPY check_password.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/check_password.sh

# Thiết lập người dùng và nhóm người dùng. Dược dùng khi chạy các lệnh ở chỉ thị RUN CMD WORKDIR (ben duoi)
RUN groupadd -r $GROUPNAME
RUN useradd -m -r -g $GROUPNAME $USERNAME && echo "$USERNAME:$PASSWORD" | chpasswd
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER $USERNAME

# Khởi động môi trường ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /usr/local/bin/check_password.sh" >> ~/.bashrc
RUN echo 'if [ $check_pass_result != true ]; then' >> ~/.bashrc
RUN echo "\texit" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc

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
# Đặt quyền chi user moi duoc doc cho file
RUN chown $USERNAME:$GROUPNAME /SASs_AMR_ws/src
RUN chmod 500 /SASs_AMR_ws/src
# Build source code
RUN cd /SASs_AMR_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Mở cổng cho ROS Master. Thiết lập cổng mà container lắng nghe, cho phép các container khác trên cùng mạng liên lạc qua cổng này hoặc đỉ ánh xạ cổng host vào cổng này.
EXPOSE 11311
# EXPOSE 80

# Tạo một ổ đĩa chia sẻ được giữa các container.
# VOLUME /dir_vol

# Khởi động lệnh mặc định khi container được chạy
ENTRYPOINT /usr/local/bin/check_password.sh $USERNAME $PASSWORD true
# CMD ["/bin/bash"]
# CMD ["roslaunch", "topic_pkg", "topic_cpp.launch"]
# CMD ["/bin/bash", "-c", "source devel/setup.bash && roslaunch topic_pkg topic_cpp.launch"]