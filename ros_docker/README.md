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
- Build the docker images: $docker build -t test_image .
- Check Docker images: $sudo docker ps

# Delete Image
- Delete docker images: $sudo docker rmi -f test_image

# Run container
- Run docker Image: $sudo docker run --privileged -it --device=/dev/ttyUSB0:/dev/ttyUSB0 -p 8080:11311 --name test_container test_image
- Access to container: $sudo docker exec -it test_image <command>
- Explain the option: 
    + -it: -i allows you to interact with the container by providing input, -t allocates a pseudo-TTY (terminal) for the container. 
        + Example: $sudo docker run -it ubuntu /bin/bash
    + --device: Match port (device on host) to port (device on container). 
    Example: $sudo docker run --device=/dev/ttyUSB1:/dev/ttyUSB0 my-web-server-image
    + --privileged: Allow access all device on host machine. 
    Example: $sudo docker run --privileged my-container-image
    + -p <host_port>:<container_port>: (--publish) This option allows you to publish (or map) ports from the container to the host. It is used for network port mapping.
    Example: $sudo docker run -p 8080:80 my-web-server-image
    + -e: (--env) This option allows you to set environment variables within the container. It is used for configuring application settings. 
    Example: $sudo docker run -e DATABASE_URL=postgres://user:password@host:5432/database my-app-image
    + --name <name>: Allow to specify a custom name for the container when it's created. It makes it easier to identify and manage containers. 
    Example: $sudo docker run --name my-container my-container-image
    + -d: (--detach) This option runs the container in the background, detaching it from your terminal so that you can continue using the terminal for other tasks.
    Example: $sudo docker run -d my-container-image
    + -v: (--volume) This option allows you to create a volume or bind mount between the host system and the container, enabling data sharing or persistence.
    Example: $sudo docker run -v /host/folder:/container/folder my-container-image

