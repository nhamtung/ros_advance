#!/bin/bash

htpasswd_file="/etc/docker/htpasswd"

username="$1"
entered_password="$2"
arg_param="$3"

check_pass_result=false

# Hàm xử lý khi nhận tín hiệu SIGINT (CTRL+C)
trap ctrl_c INT
function ctrl_c() {
    echo "CTRL+C pressed! Script terminated."
    check_pass_result=false
    exit 1
}

# Hàm kiểm tra mật khẩu
check_password() {
  # Kiểm tra xem mật khẩu từ người dùng có khớp với mật khẩu đã lưu trữ
  htpasswd -vb /etc/docker/htpasswd $username $entered_password
  if [ $? -eq 0 ]; then
    echo "Mật khẩu đúng. Truy cập được chấp nhận."
    if [ "$arg_param" != "" ]; then
      echo "Chay chuong trinh roslaunch"
      source devel/setup.bash
      roslaunch topic_pkg topic_cpp.launch
    fi
    check_pass_result=true
  else
    echo "Mật khẩu sai. Exit terminal"
    check_pass_result=false
  fi
}

if [ "$arg_param" == "" ]; then
  # Nhập tên người dùng và mật khẩu từ người dùng
  read -p "Nhập tên người dùng: " username
  read -s -p "Nhập mật khẩu: " entered_password
  echo
fi

# Gọi hàm kiểm tra mật khẩu
check_password "$username" "$entered_password"
