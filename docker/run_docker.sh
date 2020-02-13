./build_docker.sh
xhost +
docker run \
    -e DISPLAY \
    --gpus=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -tid \
    -v $PWD/..:/root/carla_cyber_bridge \
    --name carla-cyber \
    carla-cyber
