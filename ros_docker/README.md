# Guide to package using docker

# Install docker
- Reference link: https://docs.docker.com/desktop/install/ubuntu/
- Install docker desktop on Ubuntu: 
    + $sudo apt-get update
    + $sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
    + $curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    + $sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    + $sudo apt-get update
    + $sudo apt-get install docker-ce
- Start docker: $sudo systemctl start docker
- Enable docker: $sudo systemctl enable docker
- Stop docker: $sudo systemctl stop docker
- Check docker status: $sudo systemctl status docker
- Restart docker after installation: $sudo systemctl restart docker
- Check docker version: $docker version
- Check docker compose version: $docker compose version

# Build Image
- Connect Internet.
- Copy Dockerfile to home: sudo cp $HOME/.../ros_ws/src/ros_advance/ros_docker/Dockerfile $HOME
- Direct: $cd $HOME
- Build the docker images: $docker build -t image_name .
- Check Docker images: $sudo docker images

# Install Image
- Install docker Image (basic): $sudo docker run -it image_name

# Delete Image
- Delete docker images: $sudo docker rmi -f image_name