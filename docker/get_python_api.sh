PYTHON_API_DIR='../carla-python-0.9.6'
if [ -d $PYTHON_API_DIR ]
then
    echo 'Python api already exists.'
else
    mkdir $PYTHON_API_DIR
    docker run --rm carlasim/carla:0.9.6 tar cC /home/carla/PythonAPI . | tar xvC $PYTHON_API_DIR
    echo 'Python api successfully extracted from docker image.'
fi