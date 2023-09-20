#!/bin/bash

htpasswd_file="/etc/docker/htpasswd"
# Hàm kiểm tra mật khẩu
check_password() {
    username="$1"
    entered_password="$2"
    
    # Kiểm tra xem mật khẩu từ người dùng có khớp với mật khẩu đã lưu trữ
    htpasswd -vb /etc/docker/htpasswd $username $entered_password
    if [ $? -eq 0 ]; then
      echo "Mật khẩu đúng. Truy cập được chấp nhận."
      source devel/setup.bash
      roslaunch topic_pkg topic_cpp.launch
    else
      echo "Mật khẩu sai."
    fi
}

# Nhập tên người dùng và mật khẩu từ người dùng
read -p "Nhập tên người dùng: " username
read -s -p "Nhập mật khẩu: " entered_password
echo

# Gọi hàm kiểm tra mật khẩu
check_password "$username" "$entered_password"

