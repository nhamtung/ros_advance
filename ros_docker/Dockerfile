# Reference link: https://viblo.asia/p/tim-hieu-ve-dockerfile-va-tao-docker-image-V3m5WWag5O7

# Sử dụng một hệ điều hành cơ sở
FROM ros:noetic

LABEL version="1.0.0" description="Example about using docker"

ENV USERNAME="robotics" PASSWORD="robotics" 
ENV ROBOT_TYPE="SASs_AMR" ROBOT_NAME="SASs_AMR_100"
ENV ROS_MASTER_URI="http://localhost:11311" ROS_HOSTNAME="localhost"

# Khởi động môi trường ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Cai mot so goi phu thuoc
RUN apt-get update
RUN apt-get install -y python3 python3-pip
RUN pip3 install ds4drv
RUN apt-get install -y ros-noetic-joy
RUN apt-get install -y ros-noetic-grpc
RUN apt-get install -y libspnav-dev && apt install -y libx11-dev
RUN apt-get install -y ros-noetic-roslint
RUN apt-get install -y ros-noetic-tf2-ros

# Tạo thư mục làm việc và sao lưu workspace ROS
WORKDIR /SASs_AMR_ws
RUN mkdir -p /SASs_AMR_ws/src
COPY $HOME/TungNV/ros_ws/src /SASs_AMR_ws/src
RUN  cd /SASs_AMR_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
# RUN catkin_make

# Run example program
# RUN roslaunch topic_pkg topic_cpp.launch

# Mở cổng cho ROS Master (nếu cần)
EXPOSE 11311

# Khởi động lệnh mặc định khi container được chạy
CMD ["bash"]