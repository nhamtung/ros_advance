# Guide to package using docker

# Install
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

# Build 
- Direct to folder where contain the Dockerfile file
- Build the docker images: $docker build -t <my-ros-image> .

# Run
- Run docker (basic): $docker run -it <my-ros-image>
