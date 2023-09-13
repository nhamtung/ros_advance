# Reference link: https://viblo.asia/p/tim-hieu-ve-dockerfile-va-tao-docker-image-V3m5WWag5O7

# Sử dụng một hệ điều hành cơ sở
FROM ubuntu:20.04

LABEL version="1.0.0" description="Example about using docker"
MAINTAINER nhamtung <nhamtung125@gmail.com>

ENV USERNAME="robotics" PASSWORD="robotics"

# Cài đặt các gói cơ bản
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    python3 \
    python3-pip

# Cài đặt ROS
# Thay 'noetic' bằng phiên bản ROS bạn muốn cài đặt
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-desktop

# Khởi động môi trường ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Cài đặt và khởi động một số tiện ích ROS
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-rosbridge-suite

# Tạo thư mục làm việc và sao lưu workspace ROS
WORKDIR /workspace
RUN mkdir -p /catkin_ws/src
COPY ./catkin_ws /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make"

# Khởi động lệnh mặc định khi container được chạy
CMD ["bash"]