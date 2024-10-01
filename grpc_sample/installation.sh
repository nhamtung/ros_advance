# Cài đặt các phụ thuộc
sudo apt-get install -y cmake build-essential autoconf libtool pkg-config
sudo apt-get install -y protobuf-compiler libprotobuf-dev
git clone -b v1.47.0 https://github.com/grpc/grpc
cd grpc
git submodule update --init
mkdir -p cmake/build
cd cmake/build
cmake ../..
make -j4
sudo make install
cd ../../..
sudo rm -r grpc
