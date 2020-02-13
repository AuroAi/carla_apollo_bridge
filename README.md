# Carla Cyber Bridge

This python package provides a bridge for communicating between Apollo's Cyber Python API and Carla.  Besides the source code, a Dockerfile and scripts are provided for getting setup quickly and easily.  This package was tested with Carla version 0.9.6, and Apollo v5.0.0.

This is a cyber port of the work done here: https://github.com/carla-simulator/ros-bridge

## Installation

### Pre-requisites

For the simplest setup, we will run Carla in Docker.  You can run Carla from source if you would like, but the setup is more involved: https://carla.readthedocs.io/en/latest/how_to_build_on_linux/

#### docker

https://docs.docker.com/install/linux/docker-ce/ubuntu/

#### nvidia-docker

https://github.com/nvidia/nvidia-docker

## Setup / Getting Started

The following commands will be run with 3 containers:

- carla-server: this container will run the Carla simulator
- carla-cyber: bridge between apollo and carla-server containers, has cyber_py and carla python packages installed and unlike apollo container, can easily display gui applications on local machine
- apollo_dev_user: runs the apollo stack

![containers-diagram](https://user-images.githubusercontent.com/3516571/69467349-66ac7b00-0d3c-11ea-9a81-ef87cfbb6b21.png)

### Clone and build Apollo

Our fork of Apollo has a few changes that make it work with this Carla bridge.  You can see those changes here: https://github.com/ApolloAuto/apollo/compare/v5.0.0...AuroAi:carla

```
# run on local machine:

git clone https://github.com/auroai/apollo --single-branch -b carla
cd apollo
./docker/scripts/dev_start.sh
./docker/scripts/dev_into.sh
```

Now in the apollo container, build apollo...
```
# run in apollo_dev_user container:

./apollo.sh build_gpu
```

### Run Carla docker container

This container will run the carla simulator.

```
# run on local machine:

docker run --gpus=all --name=carla-server --net=host -d carlasim/carla:0.9.6
```

### Build docker image / run container for Carla-Cyber bridge

This container will run the bridge and sim clients.

```
# run on local machine, starting from the root of this repo:

cd docker
./build_docker.sh
./run_docker.sh
```

## Usage

### Run Carla client and bridge

#### Enter carla-cyber docker container

```
# run on local machine:

docker exec -ti carla-cyber bash
```

#### Update /apollo/cyber/setup.bash

Change CYBER_IP in /apollo/cyber/setup.bash to the carla-cyber container IP address

To find out the ip address to use, run this command outside of the container:

```
# run on local machine:

docker inspect carla-cyber | grep IPAddress
```

Then update the file in your preferred editor

```
# run in carla-cyber container:

vim /apollo/cyber/setup.bash
# and so on to edit the text file

# then source your ~/.bashrc file to apply the changes:
source ~/.bashrc
```

#### Create an ego vehicle and client

Run these commands inside the carla-cyber container

```
# run in carla-cyber container:

cd ~/carla_cyber_bridge
python examples/run_bridge.py
```

In another terminal...

```
# run in carla-cyber container in another terminal:

cd ~/carla_cyber_bridge
python examples/manual_control.py
```

### Run Apollo Dreamview & modules

Now, in the apollo container, run dreamview:

```
# run in apollo_dev_user container:

. /apollo/scripts/dreamview.sh start_fe
```

Then, in a web browser, go to: `localhost:8888`

#### View camera feed

#### View point cloud

#### Routing

#### Perception & Prediction

#### Planning

#### Control

## Known Issues

- Traffic lights, stop signs not in the HD Map.  This is because they are not included in the Carla OpenDRIVE maps.
- When closing the bridge sometimes objects aren't cleaned up properly and cyber will error due to duplicate nodes.  The easiest solution is to reload the map in Carla:
```
root@7243ed7667bd:~/carla_cyber_bridge# python carla-python-0.9.6/util/config.py --host 172.17.0.1 -r
```
- Running the Carla server and Apollo on the same machine requires a lot of resources, so performance may be choppy.
- Ego vehicle movement sometimes gets jumpy when being moved along planned trajectory.
