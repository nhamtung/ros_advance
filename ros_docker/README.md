# Guide to package using docker
- Reference link: https://xuanthulab.net/gioi-thieu-ve-docker-lam-quen-voi-docker-tao-container.html

# Install docker
- Reference link: https://docs.docker.com/desktop/install/ubuntu/
- Install docker on Ubuntu: 
    + $sudo apt-get update
    + Install dependence package: $sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
    + $curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    + $echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    + $echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    + $sudo apt-get update
    + Install docker: $sudo apt-get install docker-ce docker-ce-cli containerd.io
    + Install docker compose: $sudo apt install docker-compose
    + Add user to docker group: $sudo usermod -aG docker $USER
- Start docker: $sudo systemctl start docker
- Enable docker: $sudo systemctl enable docker
- Check docker status: $sudo systemctl status docker
- Restart docker after installation: $sudo systemctl restart docker
- Stop docker: $sudo systemctl stop docker
- Check docker version: $docker version
- Check docker compose version: $docker compose version
- Remove docker: $sudo apt-get purge docker-ce docker-ce-cli containerd.io

# Build Image
- Connect Internet (Host machine)
- Copy Dockerfile and check_password.sh file to home: 
    + $sudo cp $HOME/TungNV/ros_ws/src/ros_advance/ros_docker/Dockerfile $HOME
    + $sudo cp $HOME/TungNV/ros_ws/src/ros_advance/ros_docker/check_password.sh $HOME
- Direct: $cd $HOME
- Build the docker images: $sudo docker build -t test_image:1.0.0 --force-rm -f Dockerfile .
- Check list of docker images: $sudo docker images

# Delete Image
- Check list of images: $sudo docker images
- Delete docker images: $sudo docker rmi -f <image_id>

# Run container
- Create the new container: 
    + Basic: $sudo docker run -it --rm --user nhamtung --name test_container test_image
    + Option: $sudo docker run --privileged -it --rm --user nhamtung --device=/dev/ttyUSB0:/dev/ttyUSB0 -p 8080:11311 --name test_container test_image
- Access to container: $sudo docker exec -it test_container /bin/bash
- Check the list of container running: $sudo docker ps
- Explain the option: 
    + -it: -i allows you to interact with the container by providing input, -t allocates a pseudo-TTY (terminal) for the container. 
        + Example: $sudo docker run -it ubuntu /bin/bash
    + --device: Match port (device on host) to port (device on container). 
        + Example: $sudo docker run --device=/dev/ttyUSB1:/dev/ttyUSB0 my-web-server-image
    + --privileged: Allow access all device on host machine. 
        + Example: $sudo docker run --privileged my-container-image
    + -p <host_port>:<container_port>: (--publish) This option allows you to publish (or map) ports from the container to the host. It is used for network port mapping.
        + Example: $sudo docker run -p 8080:80 my-web-server-image
    + -e: (--env) This option allows you to set environment variables within the container. It is used for configuring application settings. 
        + Example: $sudo docker run -e DATABASE_URL=postgres://user:password@host:5432/database my-app-image
    + --name <name>: Allow to specify a custom name for the container when it's created. It makes it easier to identify and manage containers. 
        + Example: $sudo docker run --name my-container my-container-image
    + -d: (--detach) This option runs the container in the background, detaching it from your terminal so that you can continue using the terminal for other tasks.
        + Example: $sudo docker run -d my-container-image
    + --rm: Remove container after stop
    + -v: (--volume) This option allows you to create a volume or bind mount between the host system and the container, enabling data sharing or persistence.
        + Example: $sudo docker run -v /host/folder:/container/folder my-container-image
    + -volumes-from: This option allows a container to access the volumes of another container. It's often used when you want one container to share data with another.
        + Example: $sudo docker run --volumes-from=my-data-container my-app-image
- Enter the termial container running: $sudo docker container attach <container_id>
- Run the container which stopped: $sudo docker container start -i <container_id>
- Run the conmand in running container: $sudo docker exec -it <container_id> <command>

# Exit Container
- Exit container: exit or CTRL + Q
- Check the list of container: $sudo docker ps -a
- Stop container: $sudo docker stop test_container
- Remove container: $sudo docker rm <container_id>

# Download the docker images
- Download: $sudo docker pull nameimage:tag

# Save the container to image
- Stop the container: $sudo docker stop <container_id>
- Save the container to image: $sudo docker commit <container_id> <image_id>:<version>