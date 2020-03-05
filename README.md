# Carla Apollo Bridge

This python package provides a bridge for communicating between Apollo's Python API and Carla.  Besides the source code, a Dockerfile and scripts are provided for getting setup quickly and easily.  This package was tested with Carla version 0.9.6, and Apollo v5.0.0.

Apollo runs on the [Cyber RT](https://medium.com/@apollo.baidu/apollo-cyber-rt-the-runtime-framework-youve-been-waiting-for-70cfed04eade) framework. This is a cyber port of the work done here: [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)

## Installation

### Pre-requisites

For the simplest setup, we will run Carla in Docker.  You can run Carla from source if you would like, but the setup is more involved: [https://carla.readthedocs.io/en/latest/how_to_build_on_linux/](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/)

#### docker

[https://docs.docker.com/install/linux/docker-ce/ubuntu/](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

#### nvidia-docker

[https://github.com/nvidia/nvidia-docker](https://github.com/nvidia/nvidia-docker)

## Setup / Getting Started

The following commands will be run with 3 containers:

- carla-server: this container will run the Carla simulator
- carla-apollo: bridge between apollo and carla-server containers, has cyber_py and carla python packages installed and unlike apollo container, can easily display gui applications on local machine
- apollo_dev_user: runs the apollo stack

![containers-diagram](https://user-images.githubusercontent.com/3516571/76017110-dea94600-5ed2-11ea-9879-5777eff9f1dd.png)

### Clone and build Apollo

Our fork of Apollo has a few changes that make it work with this Carla bridge.  You can see those changes here: [https://github.com/ApolloAuto/apollo/compare/v5.0.0...AuroAi:carla](https://github.com/ApolloAuto/apollo/compare/v5.0.0...AuroAi:carla)

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

### Build docker image / run container for Carla-Apollo bridge

This container will run the bridge and sim clients.

```
# run on local machine, starting from the root of this repo:

cd docker
./build_docker.sh
./run_docker.sh
```

## Usage

### Run Carla client and bridge

#### Enter carla-apollo docker container

```
# run on local machine:

docker exec -ti carla-apollo bash
```

#### Update /apollo/cyber/setup.bash

Change CYBER_IP in /apollo/cyber/setup.bash to the carla-apollo container IP address

To find out the ip address to use, run this command outside of the container:

```
# run on local machine:

docker inspect carla-apollo | grep IPAddress
```

Then update the file in your preferred editor

```
# run in carla-apollo container:

vim /apollo/cyber/setup.bash
# and so on to edit the text file

# then source your ~/.bashrc file to apply the changes:
source ~/.bashrc
```

#### Create an ego vehicle and client

Run these commands inside the carla-apollo container

```
# run in carla-apollo container:

cd ~/carla_apollo_bridge
python examples/run_bridge.py
```

In another terminal...

```
# run in carla-apollo container in another terminal:

cd ~/carla_apollo_bridge
python examples/manual_control.py
```

#### Interfacing with the simulation

For interfacing with the simulator, a copy of the Carla PythonAPI is included in the carla-apollo container.  Some uses:

```
# run in another carla-apollo container terminal:
cd ~/carla_apollo_bridge/carla-python-0.9.6

# change the map
python util/config.py -m Town04 --host 172.17.0.1

# spawn traffic
python examples/spawn_npc.py -n 50 --host 172.17.0.1

```

### Run Apollo Dreamview & modules

Now, in the apollo container, run dreamview:

```
# run in apollo_dev_user container:

. /apollo/scripts/dreamview.sh start_fe
```

Then, in a web browser, go to: `localhost:8888`

#### View camera feed

Click the 'Tasks' button on the sidebar to open the Tasks Toolbar.  Click the 'Camera Sensor' switch.
![camera](https://user-images.githubusercontent.com/3516571/75204973-5931d300-5727-11ea-828c-68ecb5ba2063.png)

#### View point cloud

Click the 'Layer Menu' button on the sidebar.  Click the 'Point Cloud' switch under 'Perception'.
![pointcloud](https://user-images.githubusercontent.com/3516571/75205481-c003bc00-5728-11ea-8177-d75a46978470.png)

#### Routing

Click the 'Module Controller' button on the sidebar.  Click the 'Routing' switch to enable Routing.
![routing](https://user-images.githubusercontent.com/3516571/75205804-9303d900-5729-11ea-9d9c-fffc2d847a3b.png)

Click the 'Route Editing' button on the sidebar.  Click on the map to place points.  Place one point to route from the vehicle's current location.  Place two points to route from the first point to the second.  Then, click 'Send Routing Request'.  If routing is successful, a red line appears showing the planned route.
![route_editing](https://user-images.githubusercontent.com/3516571/75205919-f7bf3380-5729-11ea-9c10-1ebc4f7fc3e8.png)

#### Perception

##### Ground truth obstacle sensor

The example scripts use a ground truth obstacle sensor instead of Apollo perception.  This is enable by including object_sensor in [config/settings.yaml](config/settings.yaml).
![groundtruth](https://user-images.githubusercontent.com/3516571/75207429-8b463380-572d-11ea-8179-32603690531c.png)

##### Apollo perception module

To use Apollo perception, remove obstacle_sensor from [config/settings.yaml](config/settings.yaml) and enable the 'Transform' and 'Perception' modules.
![apollo_percpetion](https://user-images.githubusercontent.com/3516571/75207991-225fbb00-572f-11ea-856c-09f7c4e977a4.png)

#### Prediction

Once obstacles are being published, either from Apollo perception or the ground truth obstacle sensor, you can enable the 'Prediction' module.
![prediction](https://user-images.githubusercontent.com/3516571/75208064-59ce6780-572f-11ea-9362-92dc38145a8f.png)

#### Planning

Once a route has been planned, and prediction output is received you can enable the 'Planning' module.  The bridge will automatically move the vehicle along the planned path unless output from the 'Control' module is received.
![planning](https://user-images.githubusercontent.com/3516571/75208171-ab76f200-572f-11ea-8a58-910659fb6f93.png)

#### Control
If the 'Control' module is enabled, the bridge will apply its output to the ego vehicle, but this feature has not been fully developed yet.  So, the ego's movement may be erratic.  The recommended way to use the bridge is to allow it to use the planner output for moving the vehicle.

## Known Issues

- Traffic lights, stop signs not in the HD Map.  This is because they are not included in the Carla OpenDRIVE maps.
- When closing the bridge sometimes objects aren't cleaned up properly and cyber will error due to duplicate nodes.  The easiest solution is to reload the map in Carla:
```
root@7243ed7667bd:~/carla_apollo_bridge# python carla-python-0.9.6/util/config.py --host 172.17.0.1 -r
```
- Running the Carla server and Apollo on the same machine requires a lot of resources, so performance may be choppy.
- Ego vehicle movement sometimes gets jumpy when being moved along planned trajectory.
