FROM auroai/cyber:v5.0.0
# cyber / bridge requirements
RUN apt install -y python-pip && \
    pip install \
            numpy \
            opencv-python \
            protobuf \
            pygame \
            pyproj \
            pyyaml

RUN echo "source /apollo/cyber/setup.bash" >> ~/.bashrc && \
    echo "CARLA_PYTHON_ROOT=~/carla_apollo_bridge/carla-python-0.9.6" >> ~/.bashrc && \
    echo "PYTHONPATH=/apollo/py_proto:\$PYTHONPATH" >> ~/.bashrc && \
    echo "PYTHONPATH=\$CARLA_PYTHON_ROOT/carla:\$CARLA_PYTHON_ROOT/carla/dist/carla-0.9.6-py2.7-linux-x86_64.egg:\$PYTHONPATH" >> ~/.bashrc
